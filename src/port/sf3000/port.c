/*
 * pcsx4all SF3000 port
 * Display: /dev/dis ioctl + /dev/fb0 direct mmap (same as picoarch)
 * Input:   cubevol shared memory at /tmp/joy_key
 * Menu:    ShallotsOS keys (FN / SELECT+START / hold), see "ShallotsOS" below;
 *          Exit Game returns to FrogUI (icube relaunches it)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <arpa/inet.h>
#include <linux/fb.h>
#include <time.h>
#include <math.h>
#include <dirent.h>
#include <signal.h>

#include "SDL.h"
#include "port.h"
#include "tf_driver.h"
#include "dspr2_prof.h"
#include "r3000a.h"
#include "plugins.h"
#include "sio.h"
#include "plugin_lib.h"
#include "perfmon.h"
#include "cdrom_hacks.h"
#include "cheat.h"
#include "hwdisp.h"

#ifdef SPU_PCSXREARMED
  #include "spu/spu_pcsxrearmed/spu_config.h"
#endif
#ifdef USE_GPULIB
  #include "gpu/gpulib/gpu.h"
#endif
#ifdef GPU_UNAI
  #include "gpu/gpu_unai/gpu.h"
#endif
#ifdef PSXREC
  extern u32 cycle_multiplier;
#endif

static void plog(const char *msg);

#ifdef DSPR2_PROFILE
static int profile_read_one_line(const char *path, char *out, size_t out_size) {
    FILE *f = fopen(path, "r");
    if (!f) return 0;
    if (!fgets(out, out_size, f)) {
        fclose(f);
        return 0;
    }
    fclose(f);
    out[strcspn(out, "\r\n")] = '\0';
    return 1;
}

static void profile_dump_cpu_policy(void) {
    for (int cpu = 0; cpu < 8; cpu++) {
        char path[160], governor[32], cur[32], min[32], max[32];
        snprintf(path, sizeof path,
                 "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_governor", cpu);
        if (!profile_read_one_line(path, governor, sizeof governor)) continue;
        snprintf(path, sizeof path,
                 "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_cur_freq", cpu);
        if (!profile_read_one_line(path, cur, sizeof cur)) strcpy(cur, "?");
        snprintf(path, sizeof path,
                 "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_min_freq", cpu);
        if (!profile_read_one_line(path, min, sizeof min)) strcpy(min, "?");
        snprintf(path, sizeof path,
                 "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_max_freq", cpu);
        if (!profile_read_one_line(path, max, sizeof max)) strcpy(max, "?");
        fprintf(stderr,
                "PCSX4ALL_PROFILE_CPUFREQ: cpu%d governor=%s cur=%s min=%s max=%s\n",
                cpu, governor, cur, min, max);
    }
}
#endif

uint8_t use_speedup = 0;
void update_window_size(int w, int h) {
    if (w == SCREEN_WIDTH && h == SCREEN_HEIGHT) return;
    free(SCREEN);
    SCREEN_WIDTH  = w;
    SCREEN_HEIGHT = h;
    SCREEN = calloc((size_t)w * h, 2);
}

/* --------------------------------------------------------------------------
 * Display state
 * -------------------------------------------------------------------------- */
static int          dis_fd  = -1;
static int          fb_fd   = -1;
static uint32_t    *fb_mem  = NULL;
static size_t       fb_size = 0;
static struct fb_var_screeninfo vinfo;
static struct fb_fix_screeninfo finfo;
static int          current_page = 0;

#define PHYS_W      854
#define PHYS_H      480
#define PAGE_Y_SIZE 1280
#define FB_X_VIS    180
#define FB_Y_TOT    1014  /* fb_y range 0..1013 maps to PHYS_W=854 physical px */

static uint16_t *rotated_buf    = NULL;
static int       rotated_cap    = 0;
static int       last_fb_y_off  = -1;
static int       last_fb_y_len  = -1;

struct blend_data { int16_t gy1, gy2; uint8_t w1, w2; };
static struct blend_data blend_lut[FB_X_VIS];
static int last_h_blend = -1;

unsigned short *SCREEN        = NULL;
int             SCREEN_WIDTH  = 320;
int             SCREEN_HEIGHT = 240;
int             scale_mode    = 0;   /* 0=aspect-correct, 1=fullscreen */
int             hires_fix     = 0;   /* 0=transparent (raw frame -> HW scaler, full speed, natural default); 1=SW-decimate tall (>256) frames + exact 4:3 so disp_frame can present hi-res without wedging (fixes Colin/Worms freeze/black, costs per-frame CPU) */
int             scale_filter  = 1;   /* 0=nearest (SW fb0 — broken on this HW), 1=bilinear (HW HCGE) */
int             use_hwdisp    = 0;   /* 0=software path, 1=HW via driver.so */

/* Gamma / black-lift: 100 = off (linear). >100 lifts crushed blacks (PS1
 * true-black games unreadable on this panel). Applied to SCREEN (RGB565) just
 * before blit. Safe in-place: vout re-copies the whole visible area from VRAM
 * every frame, so the LUT never accumulates. */
int             gamma_percent = 100;
static uint8_t  glut5[32], glut6[64];
static int      glut_built_for = -1;
static void gamma_build_lut(void) {
    if (glut_built_for == gamma_percent) return;
    double g = gamma_percent / 100.0;            /* >1 brightens shadows */
    for (int i = 0; i < 32; i++) {
        int v = (int)(pow(i / 31.0, 1.0 / g) * 31.0 + 0.5);
        glut5[i] = v > 31 ? 31 : v;
    }
    for (int i = 0; i < 64; i++) {
        int v = (int)(pow(i / 63.0, 1.0 / g) * 63.0 + 0.5);
        glut6[i] = v > 63 ? 63 : v;
    }
    glut_built_for = gamma_percent;
}
static void gamma_apply(uint16_t *px, int count) {
    if (gamma_percent <= 100 || !px) return;
    gamma_build_lut();
    for (int i = 0; i < count; i++) {
        uint16_t p = px[i];
        px[i] = (glut5[(p >> 11) & 31] << 11)
              | (glut6[(p >>  5) & 63] <<  5)
              |  glut5[ p        & 31];
    }
}

/* RGB565 → ARGB8888 with full 8-bit channel expansion */
static inline uint32_t cvt565(uint16_t c) {
    return 0xFF000000u
         | (uint32_t)((c & 0xF800) << 8) | (uint32_t)((c & 0xE000) << 3)
         | (uint32_t)((c & 0x07E0) << 5) | (uint32_t)((c & 0x0600) >> 1)
         | (uint32_t)((c & 0x001F) << 3) | (uint32_t)((c & 0x001C) >> 2);
}

static void dis_assert(void) {
    if (dis_fd < 0) dis_fd = open("/dev/dis", O_RDWR);
    if (dis_fd >= 0) {
        struct { int a, b, c; } buf = {1, 0, 0};
        ioctl(dis_fd, 0xc00c0e0c, &buf);
    }
}

/* OSD overlay (battery, volume) is drawn by cubevol on /dev/fb1 (ARGB plane).
 * Hide: memset fb1 to zero (alpha=0 → invisible). cubevol has no signal
 *       handler so it won't redraw on its own.
 * Show: restart cubevol via killall + spawn; new instance draws OSD on init. */
#include <sys/mman.h>
#include <linux/fb.h>
#include <stdlib.h>
static void fb1_clear(void) {
    int fd = open("/dev/fb1", O_RDWR);
    if (fd < 0) return;
    struct fb_fix_screeninfo finfo;
    if (ioctl(fd, FBIOGET_FSCREENINFO, &finfo) == 0 && finfo.smem_len > 0) {
        void *mem = mmap(NULL, finfo.smem_len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
        if (mem != MAP_FAILED) { memset(mem, 0, finfo.smem_len); munmap(mem, finfo.smem_len); }
    }
    close(fd);
}
/* Only FrogUI restores OSD on its own init. pcsx4all keeps fb1 cleared. */
static void fb1_blank(int blank) { if (blank) fb1_clear(); }

static int display_init(void) {
    dis_assert();

    fb_fd = open("/dev/fb0", O_RDWR);
    if (fb_fd < 0) { perror("open /dev/fb0"); return -1; }

    ioctl(fb_fd, FBIOGET_VSCREENINFO, &vinfo);
    ioctl(fb_fd, FBIOGET_FSCREENINFO, &finfo);

    fb_size = finfo.smem_len;
    fb_mem  = mmap(NULL, fb_size, PROT_READ|PROT_WRITE, MAP_SHARED, fb_fd, 0);
    if (fb_mem == MAP_FAILED) {
        perror("mmap fb0");
        close(fb_fd); fb_fd = -1;
        return -1;
    }

    fprintf(stderr, "SF3000 fb: %ux%u bpp=%u line=%u mem=%p\n",
            vinfo.xres, vinfo.yres, vinfo.bits_per_pixel,
            finfo.line_length, (void*)fb_mem);

    /* Clear to black */
    for (size_t i = 0; i < fb_size / 4; i++) fb_mem[i] = 0xFF000000u;

    /* Off-screen 320x240 RGB565 buffer for GPU to render into */
    SCREEN       = calloc(SCREEN_WIDTH * SCREEN_HEIGHT, 2);
    SCREEN_WIDTH  = 320;
    SCREEN_HEIGHT = 240;

    /* HW display via driver.so. Fail loudly if unavailable. */
    if (hwdisp_init() != 0) {
        plog("display_init: hwdisp_init FAILED");
        fprintf(stderr, "display_init: hwdisp_init FAILED — aborting\n");
        return -1;
    }
    use_hwdisp = 1;
    fb1_blank(1);   /* hide battery/volume OSD during gameplay */
    /* scale_mode 0 = Aspect-Correct (nearest SW-upscale to 1280×720 with
     *   16:9 pad — driver does uniform downscale, letterbox bars survive)
     * scale_mode 1 = Fullscreen (HW bilinear direct, driver stretches per axis)
     * Driver-side aspect padding alone is not respected — bars get scaled
     * away. Putting bars into the 1280×720 staging buffer works. */
    if (scale_mode == 1) {
        hwdisp_set_target_aspect(0, 0);
        hwdisp_set_filter(0);
    } else {
        hwdisp_set_target_aspect(16, 9);
        hwdisp_set_filter(1);
    }
    plog("display_init: HW path active");
    fprintf(stderr, "display_init: HW path active\n");
    return 0;
}

static void reopen_speaker_amp(void) {
    char env[512] = {0};
    int fd = open("/tmp/tfdevice.env", O_RDONLY);
    if (fd < 0) return;
    int n = read(fd, env, sizeof(env) - 1);
    close(fd);
    if (n <= 0 || (!strstr(env, "TF_DEVICE=R36HD\n") &&
                   !strstr(env, "TF_DEVICE=R36SX\n"))) {
        plog("audio amp: unsupported device; left unchanged");
        return;
    }

    uint32_t cell;
    fd = open("/proc/device-tree/panel/speaker-output", O_RDONLY);
    if (fd < 0 || read(fd, &cell, sizeof(cell)) != sizeof(cell)) {
        if (fd >= 0) close(fd);
        plog("audio amp: speaker-output unavailable; left unchanged");
        return;
    }
    close(fd);
    uint32_t pin = ntohl(cell);
    if (pin > 31) {
        plog("audio amp: speaker pin out of range; left unchanged");
        return;
    }

    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) return;
    void *mapping = mmap(NULL, 0x2020, PROT_READ | PROT_WRITE, MAP_SHARED,
                         fd, 0x18800000);
    close(fd);
    if (mapping == MAP_FAILED) return;
    volatile uint32_t *gpio_l = (volatile uint32_t *)((char *)mapping + 0x44);
    uint32_t bit = 1u << pin;
    gpio_l[5] |= bit;
    gpio_l[4] &= ~bit;
    munmap(mapping, 0x2020);
    plog("audio amp: R36 speaker output enabled");
}

/* PSX aspect normalize to 4:3. PSX pixels are non-square and every mode is meant
 * to display at 4:3: WIDE modes (512×240 / 640×240 / 384×240, w > 4/3·h) must be
 * squeezed horizontally, NARROW modes (256×240, w < 4/3·h) must be stretched
 * horizontally. Doing only the wide case left narrow modes squished in width
 * (res-dependent). We resample the width to 4/3·h either way, so the presented
 * buffer is true 4:3; the driver then scales it uniformly to the panel with the
 * correct pillarbox. Only an exact-4:3 source (320×240) skips the pass.
 *
 * Cost: one horizontal resample (~76-150KB/frame); mirrors how the libretro
 * cores normalize their output. */
static int tf_is_r36sx(void);   /* fwd: panel aspect differs by device */

/* Present-time panel aspect. The frame is already normalized to 4:3 content, so:
 *   SF-class (16:9 panel) -> pad it to 16:9 (pillarbox), driver fills the panel.
 *   R36SX   (4:3 panel)   -> no pad; the 4:3 content full-stretches to the 4:3
 *                            panel and fills it. Padding to 16:9 here was what
 *                            double-letterboxed R36SX into huge black bars. */
static void set_panel_aspect(void) {
    if (tf_is_r36sx()) hwdisp_set_target_aspect(0, 0);
    else               hwdisp_set_target_aspect(16, 9);
}

static void compose_aspect_4to3(const void *src, int w, int h, int pitch) {
    int target_w = h * 4 / 3;
    if (target_w > 640) target_w = 640;        /* buf cap */

    if (target_w == w) {
        /* Exactly 4:3 already (320×240): pass straight through. */
        set_panel_aspect();
        hwdisp_set_filter(0);
        hwdisp_present(src, w, h, pitch);
        return;
    }

    /* Resample W → target_w (down for wide, up for narrow) via nearest. */
    static uint16_t buf[640 * 512];            /* fits up to 640×480 PSX hi-res */
    static int x_map[640];
    static int last_w = -1, last_tgt = -1;
    if (w != last_w || target_w != last_tgt) {
        for (int dx = 0; dx < target_w; dx++) x_map[dx] = dx * w / target_w;
        last_w = w; last_tgt = target_w;
    }

    const uint16_t *s = (const uint16_t *)src;
    const int sp = pitch / 2;
    for (int y = 0; y < h && y < 512; y++) {
        const uint16_t *srow = s + y * sp;
        uint16_t *drow = buf + y * target_w;
        for (int dx = 0; dx < target_w; dx++) drow[dx] = srow[x_map[dx]];
    }

    set_panel_aspect();
    hwdisp_set_filter(0);
    hwdisp_present(buf, target_w, h, target_w * 2);
}

static int tf_is_r36sx(void) {
    static int v = -1;
    if (v < 0) { const char *d = tf_driver_path(); v = strstr(d, "r36sx") ? 1 : 0; }
    return v;
}

/* Tallest game frame disp_frame presents without wedging/blacking. All these
 * drivers choke on panel-height (480-tall) game frames: R36SX blanks, SF-class
 * wedges the display pipeline after the hi-res storm (Colin McRae 640x480). PSX
 * hi-res is the interlaced 2x of a <=256 base mode, so cap at 256 and decimate
 * the tall modes back under it. (The FrogUI menu uses a different present path,
 * so this doesn't affect it.) */
static int tf_max_present_h(void) {
    return 256;
}

static void display_blit(const void *src, int w, int h, int pitch) {
    /* Log resolution changes (gated on pcsx_log.txt) for diagnostics. */
    { static int lw = -1, lh = -1;
      if (w != lw || h != lh) {
          char b[80]; snprintf(b, sizeof b, "blit res %dx%d scale=%d", w, h, scale_mode);
          plog(b); lw = w; lh = h; } }
    /* Some devices' disp_frame blacks out on frames taller than a per-device
     * boundary (R36SX: fine at 320x240, black at 640x480 hi-res -> Worms). PSX
     * hi-res is the interlaced 2x of the base mode, so decimate 2x back under the
     * boundary. Boundary-driven, not device-hardcoded: SF-class has no effective
     * cap, so it keeps full-res; any device that can't present tall frames does. */
    static uint16_t hbuf[320 * 256];
    if (hires_fix && h > tf_max_present_h()) {
        int dw = w / 2, dh = h / 2;
        if (dw > 320) dw = 320;
        if (dh > 256) dh = 256;
        const uint16_t *s = (const uint16_t *)src; int sp = pitch / 2;
        for (int y = 0; y < dh; y++) {
            const uint16_t *sr = s + (size_t)(y * 2) * sp;
            uint16_t *dr = hbuf + (size_t)y * dw;
            for (int x = 0; x < dw; x++) dr[x] = sr[x * 2];
        }
        src = hbuf; w = dw; h = dh; pitch = dw * 2;
    }
    /* Always use HW path: SW fb0 writes are invisible once hwdisp_init has
     * activated HCGE in this process, because per-frame dis_assert() can't
     * override HCGE display routing on this hardware. */
    if (use_hwdisp) {
        if (scale_mode == 0 && hires_fix) {
            /* Aspect-correct: PSX 4:3 regardless of internal resolution (SW resample). */
            compose_aspect_4to3(src, w, h, pitch);
        } else if (scale_mode == 0) {
            /* Hi-Res Fix OFF => fully transparent: NO software resampling at all.
             * Hand the raw frame to the driver and let the HW scaler fit it to the
             * panel (aspect via the driver flag). Non-square modes lose exact 4:3,
             * but there is zero per-frame CPU cost. */
            set_panel_aspect();
            hwdisp_set_filter(0);
            hwdisp_present(src, w, h, pitch);
        } else {
            /* Fullscreen stretch */
            hwdisp_set_target_aspect(0, 0);
            hwdisp_set_filter(0);
            hwdisp_present(src, w, h, pitch);
        }
        return;
    }
    dis_assert();
    if (!fb_mem || !src) return;

    const int dst_stride = (int)(finfo.line_length / 4);

    /* Transpose src(w×h) → rotated_buf(h×w) column-major.
       gy-outer/gx-inner: reads are sequential (row[gx]), writes are strided. */
    if (w * h > rotated_cap) {
        free(rotated_buf);
        rotated_cap = w * h + 4096;
        rotated_buf = malloc(rotated_cap * sizeof(uint16_t));
    }
    const uint16_t *s = src;
    for (int gy = 0; gy < h; gy++) {
        const uint16_t *row = (const uint16_t *)((const char *)s + gy * pitch);
        for (int gx = 0; gx < w; gx++)
            rotated_buf[gx * h + gy] = row[gx];
    }

    /* Nearest-neighbor LUT: gy[fx] = source row for output column fx */
    static int16_t gy_lut[FB_X_VIS];
    static int last_h_lut = -1;
    if (h != last_h_lut) {
        for (int fx = 0; fx < FB_X_VIS; fx++)
            gy_lut[fx] = (int16_t)((FB_X_VIS - 1 - fx) * h / FB_X_VIS);
        last_h_lut = h;
    }

    /* Bilinear blend LUT (rebuilt when h changes) */
    extern int scale_filter;
    if (scale_filter && h != last_h_blend) {
        for (int fx = 0; fx < FB_X_VIS; fx++) {
            int gf = ((FB_X_VIS - 1 - fx) * h * 256) / FB_X_VIS;
            int gy = gf >> 8;
            blend_lut[fx].gy1 = (int16_t)gy;
            blend_lut[fx].gy2 = (int16_t)((gy + 1 < h) ? gy + 1 : gy);
            blend_lut[fx].w2  = (uint8_t)((gf & 0xFF) >> 1);
            blend_lut[fx].w1  = (uint8_t)(128 - blend_lut[fx].w2);
        }
        last_h_blend = h;
    }

    /* Scale mode: 0=aspect-correct, 1=fullscreen-stretch */
    extern int scale_mode;
    int fb_y_len, fb_y_off;
    /* Physical display width for this frame (capped at PHYS_W=854) */
    /* PS1 outputs 4:3 on TV regardless of internal hres */
    int disp_w = (scale_mode == 1) ? PHYS_W : (PHYS_H * 4 / 3);
    if (disp_w > PHYS_W) disp_w = PHYS_W;
    /* fb_y range 0..FB_Y_TOT-1 maps to PHYS_W physical pixels (same formula as picoarch) */
    fb_y_len = disp_w * FB_Y_TOT / PHYS_W;
    fb_y_off = (FB_Y_TOT - fb_y_len) / 2;

    /* Double-buffer page flip */
    current_page = (current_page + 1) % 2;
    int page_y   = current_page * PAGE_Y_SIZE;

    /* Clear borders once when geometry changes */
    if (fb_y_off != last_fb_y_off || fb_y_len != last_fb_y_len) {
        for (int p = 0; p < 2; p++)
            for (int fy = 0; fy < PAGE_Y_SIZE; fy++)
                for (int fx = 0; fx < FB_X_VIS; fx++)
                    fb_mem[(p * PAGE_Y_SIZE + fy) * dst_stride + fx] = 0xFF000000u;
        last_fb_y_off = fb_y_off;
        last_fb_y_len = fb_y_len;
    }

    /* Blit with bilinear-like vertical blend */
    static uint32_t row_cache[FB_X_VIS];
    int fb_y = fb_y_off;
    while (fb_y < fb_y_off + fb_y_len) {
        int gx = (fb_y - fb_y_off) * w / fb_y_len;
        int count = 1;
        while (fb_y + count < fb_y_off + fb_y_len &&
               ((fb_y + count - fb_y_off) * w / fb_y_len) == gx)
            count++;

        const uint16_t *col = rotated_buf + gx * h;
        if (!scale_filter) {
            for (int fx = 0; fx < FB_X_VIS; fx++)
                row_cache[fx] = cvt565(col[gy_lut[fx]]);
        } else {
            for (int fx = 0; fx < FB_X_VIS; fx++) {
                uint16_t p1 = col[blend_lut[fx].gy1];
                if (blend_lut[fx].w2 == 0) {
                    row_cache[fx] = cvt565(p1);
                } else {
                    uint16_t p2 = col[blend_lut[fx].gy2];
                    if (p1 == p2) {
                        row_cache[fx] = cvt565(p1);
                    } else {
                        uint32_t c1 = cvt565(p1), c2 = cvt565(p2);
                        uint32_t w1 = blend_lut[fx].w1, w2 = blend_lut[fx].w2;
                        uint32_t rb = (((c1&0xFF00FF)*w1+(c2&0xFF00FF)*w2)>>7)&0xFF00FF;
                        uint32_t g  = (((c1&0x00FF00)*w1+(c2&0x00FF00)*w2)>>7)&0x00FF00;
                        row_cache[fx] = 0xFF000000u | rb | g;
                    }
                }
            }
        }
        uint32_t *dst = fb_mem + (page_y + fb_y) * dst_stride;
        for (int i = 0; i < count; i++) {
            for (int k = 0; k < 18; k++) {
                dst[k*10+0]=row_cache[k*10+0]; dst[k*10+1]=row_cache[k*10+1];
                dst[k*10+2]=row_cache[k*10+2]; dst[k*10+3]=row_cache[k*10+3];
                dst[k*10+4]=row_cache[k*10+4]; dst[k*10+5]=row_cache[k*10+5];
                dst[k*10+6]=row_cache[k*10+6]; dst[k*10+7]=row_cache[k*10+7];
                dst[k*10+8]=row_cache[k*10+8]; dst[k*10+9]=row_cache[k*10+9];
            }
            dst += dst_stride;
        }
        fb_y += count;
    }

    /* Page flip — timing handled by pl_frame_limit() in EmuUpdate() */
    struct fb_var_screeninfo vi = vinfo;
    vi.xoffset = 0; vi.yoffset = page_y;
    ioctl(fb_fd, FBIOPAN_DISPLAY, &vi);
}

static void shutdown_display(void) {
    fb1_blank(0);   /* restore OSD on exit so FrogUI shows battery */
    if (use_hwdisp) {
        hwdisp_deinit();
        use_hwdisp = 0;
    }
    if (fb_fd >= 0) {
        struct fb_var_screeninfo vi = vinfo;
        vi.xoffset = 0; vi.yoffset = 0;
        ioctl(fb_fd, FBIOPAN_DISPLAY, &vi);
        if (fb_mem && fb_mem != MAP_FAILED) { munmap(fb_mem, fb_size); fb_mem = NULL; }
        close(fb_fd); fb_fd = -1;
    }
    if (dis_fd >= 0) { close(dis_fd); dis_fd = -1; }
}

/* --------------------------------------------------------------------------
 * Input: cubevol shared memory
 * -------------------------------------------------------------------------- */
#define CV_UP     4
#define CV_DOWN   6
#define CV_LEFT   7
#define CV_RIGHT  5
#define CV_A     13   /* physical A - south */
#define CV_B     14   /* physical B - east  */
#define CV_X     12   /* physical X - north */
#define CV_Y     15   /* physical Y - west  */
#define CV_L     10
#define CV_R     11
#define CV_L2     8
#define CV_R2     9
#define CV_SEL    0
#define CV_START  3

/* PSX pad bits (0=pressed) */
#define PSX_SELECT    (1<<0)
#define PSX_START     (1<<3)
#define PSX_UP        (1<<4)
#define PSX_RIGHT     (1<<5)
#define PSX_DOWN      (1<<6)
#define PSX_LEFT      (1<<7)
#define PSX_L2        (1<<8)
#define PSX_R2        (1<<9)
#define PSX_L1        (1<<10)
#define PSX_R1        (1<<11)
#define PSX_TRIANGLE  (1<<12)
#define PSX_CIRCLE    (1<<13)
#define PSX_CROSS     (1<<14)
#define PSX_SQUARE    (1<<15)

static volatile uint32_t *cv_keys = NULL;
static uint16_t pad1 = 0xFFFF; /* all released */

/* ---- Remappable controls --------------------------------------------------
 * Each PSX face/shoulder button is driven by a physical CV bit, reassignable in
 * the in-game menu (Controls) and saved to pcsx4all.cfg. Dpad/Select/Start stay
 * fixed (standard). Defaults match the old hardcoded layout. */
enum { PB_CROSS, PB_CIRCLE, PB_SQUARE, PB_TRIANGLE, PB_L1, PB_R1, PB_L2, PB_R2, PB_COUNT };
static const char *pb_name[PB_COUNT] =
    { "Cross (X)", "Circle (O)", "Square", "Triangle", "L1", "R1", "L2", "R2" };
static const uint16_t pb_mask[PB_COUNT] =
    { PSX_CROSS, PSX_CIRCLE, PSX_SQUARE, PSX_TRIANGLE, PSX_L1, PSX_R1, PSX_L2, PSX_R2 };
/* Positional: PSX face button -> physical button in the same position. On these
 * pads A=south, B=east, X=north, Y=west, so: Cross(S)->A, Circle(E)->B,
 * Square(W)->Y, Triangle(N)->X. The on-screen "press Cross" = the south button. */
static int pb_bind[PB_COUNT] = { CV_A, CV_B, CV_Y, CV_X, CV_L, CV_R, CV_L2, CV_R2 };

/* Physical buttons the user can bind to (label + CV bit). */
static const struct { const char *name; int bit; } cv_btns[] = {
    { "A",  CV_A }, { "B",  CV_B }, { "X",  CV_X }, { "Y",  CV_Y },
    { "L1", CV_L }, { "R1", CV_R }, { "L2", CV_L2 }, { "R2", CV_R2 },
};
#define CV_BTNS_N ((int)(sizeof(cv_btns)/sizeof(cv_btns[0])))
static const char *cv_bit_name(int bit) {
    for (int i = 0; i < CV_BTNS_N; i++) if (cv_btns[i].bit == bit) return cv_btns[i].name;
    return "?";
}

static void input_init(void) {
    key_t k = ftok("/tmp/joy_key", 'a');
    if (k == (key_t)-1) { fprintf(stderr, "SF3000: ftok failed\n"); return; }
    int id = shmget(k, 4, 0666);
    if (id < 0) { fprintf(stderr, "SF3000: shmget failed\n"); return; }
    void *p = shmat(id, NULL, 0);
    if (p == (void*)-1) { fprintf(stderr, "SF3000: shmat failed\n"); return; }
    cv_keys = (volatile uint32_t*)p;
}

static inline int btn(uint32_t state, int bit) { return (state >> bit) & 1; }

void config_save(void);

/* Themed-menu renderer (defined after the Video API section, below). */
static void sf_font_init(void);
static int  sf_font_ready;
static int  sf_font_h(void);
static int  sf_text_w(const char *s);
static void sf_text(int x, int y, const char *s, uint16_t color);
static void sf_fill_round(int x, int y, int w, int h, int r, uint16_t c);
static uint16_t sf_col_text, sf_col_sel, sf_col_seltext;
static int sf_tw, sf_th;   /* current draw-target dims (panel res in menus) */
static void sf_draw_battery(void);
static int  sf_batt_pct(int *charging);
static char g_rom_path[1024];   /* current ROM path (defined below) */

static uint32_t sdl_prev = 0;
void sdl_poll_reset(void) { sdl_prev = cv_keys ? *cv_keys : 0; }

/* SDL_PollEvent — translates cubevol buttons into SDL key events for frontend.c menu */
int SDL_PollEvent(SDL_Event *e) {
    uint32_t prev = sdl_prev;
    static SDLKey queue[32];
    static uint8_t  qtype[32]; /* SDL_KEYDOWN or SDL_KEYUP */
    static int qhead = 0, qtail = 0;

    static const struct { int cv_bit; SDLKey sym; } map[] = {
        { CV_UP,    SDLK_UP    }, { CV_DOWN,  SDLK_DOWN  },
        { CV_LEFT,  SDLK_LEFT  }, { CV_RIGHT, SDLK_RIGHT },
        { CV_A,     SDLK_LCTRL }, { CV_B,     SDLK_LALT  },
        { CV_X,     SDLK_SPACE }, { CV_Y,     SDLK_LSHIFT},
        { CV_L,     SDLK_TAB   }, { CV_R,     SDLK_BACKSPACE },
        { CV_START, SDLK_RETURN}, { CV_SEL,   SDLK_ESCAPE},
    };
    static const int NMAP = sizeof(map)/sizeof(map[0]);

    if (!e) return 0;

    /* Refill queue from current cv_keys state */
    if (qhead == qtail && cv_keys) {
        uint32_t k = *cv_keys;
        uint32_t changed = k ^ prev;
        sdl_prev = k;
        for (int i = 0; i < NMAP && qtail < 32; i++) {
            if (!((changed >> map[i].cv_bit) & 1)) continue;
            queue[qtail] = map[i].sym;
            qtype[qtail] = ((k >> map[i].cv_bit) & 1) ? SDL_KEYDOWN : SDL_KEYUP;
            qtail++;
        }
    }

    if (qhead == qtail) return 0;

    e->type = qtype[qhead];
    e->key.keysym.sym = queue[qhead];
    qhead++;
    if (qhead == qtail) { qhead = qtail = 0; }
    return 1;
}

uint16_t pad_read(int num) { return (num == 0 ? pad1 : 0xFFFF); }

/* --------------------------------------------------------------------------
 * Video API
 * -------------------------------------------------------------------------- */
static void sh_toast_draw(void);
void video_flip(void) {
    if (Config.ShowFps) port_printf(2, 2, pl_data.stats_msg);
    gamma_apply(SCREEN, SCREEN_WIDTH * SCREEN_HEIGHT);
    sh_toast_draw();
    PROF_PRESENT_BEGIN();
    display_blit(SCREEN, SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_WIDTH * 2);
    PROF_PRESENT_END();
    PROF_FRAME_TICK();
}

void video_clear(void) {
    memset(SCREEN, 0, SCREEN_WIDTH * SCREEN_HEIGHT * 2);
}

void port_printf(int x, int y, const char *text) {
    if (!SCREEN || !text) return;
    extern void basic_text_out16_nf(void *fb, int w, int x, int y, const char *text);
    basic_text_out16_nf(SCREEN, SCREEN_WIDTH, x, y, text);
}

/* ==========================================================================
 * Themed menu rendering (TTF + skin.txt colours) to match picoarch/FrogUI.
 * Loads BPreplayBold (FrogUI's font) via stb_truetype; reads theme colours from
 * cubegm/skin/skin.txt (written by FrogUI). Falls back to the bitmap font.
 * ========================================================================== */
#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

#define SF_FONT_PX 26.0f   /* default size; the menus set their own */
static stbtt_fontinfo sf_font;
static unsigned char *sf_font_buf = NULL;
static float sf_font_scale = 0;         /* of the current size */
static int   sf_font_asc = 0;           /* scaled ascent (baseline offset) */
static int   sf_font_ready = 0;
/* Like picoarch: an OnionOS theme's font is drawn as written, the default
 * font in capitals. */
static int   sf_keep_case = 0;

/* Rasterize menu glyphs once per size. Rebuilding every glyph on every redraw
 * made SF3500 firmware 1.1 vulnerable to transient stb_truetype allocation
 * failures and lengthened the display-DMA race window. The menus use a few
 * sizes (rows, status bar, toast), each with its own cache. */
#define SF_GLYPH_FIRST 32
#define SF_GLYPH_LAST  126
#define SF_GLYPH_COUNT (SF_GLYPH_LAST - SF_GLYPH_FIRST + 1)
typedef struct {
    unsigned char ready;
    short w, h, x0, y0;
    int glyph_index;
    int advance;
    unsigned char *bitmap;
} sf_glyph_t;
typedef struct {
    int px;                 /* 0 = unused */
    float scale;
    int asc, cap;           /* ascent and capital height, in pixels */
    unsigned used;
    sf_glyph_t g[SF_GLYPH_COUNT];
} sf_fsize_t;
#define SF_SIZES 4
static sf_fsize_t sf_sizes[SF_SIZES];
static sf_fsize_t *sf_cur;
static unsigned sf_size_clock;

static sf_glyph_t *sf_glyph_get(unsigned char c) {
    if (!sf_keep_case && c >= 'a' && c <= 'z') c = c - 'a' + 'A';
    if (c < SF_GLYPH_FIRST || c > SF_GLYPH_LAST || !sf_cur) return NULL;
    sf_glyph_t *g = &sf_cur->g[c - SF_GLYPH_FIRST];
    if (g->ready) return g;

    g->glyph_index = stbtt_FindGlyphIndex(&sf_font, c);
    if (g->glyph_index) {
        int lsb, x1, y1, x0, y0;
        stbtt_GetGlyphHMetrics(&sf_font, g->glyph_index, &g->advance, &lsb);
        stbtt_GetGlyphBitmapBox(&sf_font, g->glyph_index,
                                sf_font_scale, sf_font_scale, &x0, &y0, &x1, &y1);
        g->x0 = x0; g->y0 = y0;
        g->w = x1 - x0;
        g->h = y1 - y0;
        g->bitmap = (g->w > 0 && g->h > 0) ? (unsigned char *)malloc((size_t)g->w * g->h) : NULL;
        if (g->bitmap)
            stbtt_MakeGlyphBitmap(&sf_font, g->bitmap, g->w, g->h, g->w,
                                  sf_font_scale, sf_font_scale, g->glyph_index);
        else
            g->w = g->h = 0;
    }
    g->ready = 1;
    return g;
}

/* Switch to a pixel size: reuse its cache, or replace the least recently used
 * one and rasterize every glyph now, never during a visible redraw. */
static void sf_set_px(float px) {
    int ipx = (int)(px + 0.5f);
    if (ipx < 6) ipx = 6;
    if (sf_font_ready != 1) return;
    if (sf_cur && sf_cur->px == ipx) { sf_cur->used = ++sf_size_clock; return; }
    sf_fsize_t *slot = NULL, *lru = &sf_sizes[0];
    for (int i = 0; i < SF_SIZES; i++) {
        if (sf_sizes[i].px == ipx) slot = &sf_sizes[i];
        if (sf_sizes[i].used < lru->used) lru = &sf_sizes[i];
    }
    int fresh = !slot;
    if (fresh) {
        slot = lru;
        for (int i = 0; i < SF_GLYPH_COUNT; i++) free(slot->g[i].bitmap);
        memset(slot, 0, sizeof *slot);
        slot->px = ipx;
        slot->scale = stbtt_ScaleForPixelHeight(&sf_font, (float)ipx);
        int asc, desc, gap, x0, y0, x1, y1;
        stbtt_GetFontVMetrics(&sf_font, &asc, &desc, &gap);
        slot->asc = (int)(asc * slot->scale);
        int gi = stbtt_FindGlyphIndex(&sf_font, 'H');
        slot->cap = (gi && stbtt_GetGlyphBox(&sf_font, gi, &x0, &y0, &x1, &y1))
                  ? (int)((y1 - y0) * slot->scale) : slot->asc;
    }
    slot->used = ++sf_size_clock;
    sf_cur = slot;
    sf_font_scale = slot->scale;
    sf_font_asc = slot->asc;
    if (fresh)
        for (int c = SF_GLYPH_FIRST; c <= SF_GLYPH_LAST; c++)
            sf_glyph_get((unsigned char)c);
}
/* theme colours (RGB565); defaults ~ white text, accent pill, dark sel-text */
static uint16_t sf_col_text = 0xFFFF, sf_col_sel = 0xC104, sf_col_seltext = 0x0000;

/* Menus render at PANEL resolution (like TreeFrogUI) for crisp text, not at the
 * PS1 game res. Drawing targets sf_tgt/sf_tw/sf_th (default = SCREEN); the menu
 * path points them at a panel-sized buffer and presents it 1:1. */
static uint16_t *sf_tgt = NULL; static int sf_tw = 0, sf_th = 0;
static uint16_t *sf_menu_fb = NULL; static int sf_pw = 0, sf_ph = 0;
enum { SF_MENU_STAGE_COUNT = 3 };
/* The last presented menu frame is still on screen (cleared whenever
 * something else draws: the game, the PCSX pages). */
static int sf_present_valid;
static uint16_t *sf_menu_stage[SF_MENU_STAGE_COUNT];
static int sf_menu_stage_idx;
static void sf_read_panel(void) {
    if (sf_pw) return;
    sf_pw = 640; sf_ph = 480;                 /* R36SX default */
    int fd = open("/tmp/tfdevice.env", O_RDONLY);
    if (fd >= 0) {
        char b[512]; int n = read(fd, b, sizeof b - 1); close(fd);
        if (n > 0) { b[n] = 0; char *p;
            if ((p = strstr(b, "TF_PANEL_W="))) sf_pw = atoi(p + 11);
            if ((p = strstr(b, "TF_PANEL_H="))) sf_ph = atoi(p + 11);
        }
    }
    if (sf_pw < 320) sf_pw = 640; if (sf_ph < 240) sf_ph = 480;
}

static uint16_t sf_888_to_565(unsigned v) {
    int r = (v >> 16) & 0xFF, g = (v >> 8) & 0xFF, b = v & 0xFF;
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}
/* ---- FrogUI settings (frogui/settings.txt, key=value), read once ---------- */
static char fs_text[8192];
static int fs_loaded;
static int fs_get(const char *key, char *out, size_t n) {
    if (!fs_loaded) {
        fs_loaded = 1;
        FILE *f = fopen("/mnt/sdcard/frogui/settings.txt", "r");
        if (f) {
            size_t r = fread(fs_text, 1, sizeof fs_text - 1, f);
            fs_text[r] = '\0';
            fclose(f);
        }
    }
    size_t kl = strlen(key);
    for (const char *p = fs_text; *p; ) {
        const char *eol = strchr(p, '\n');
        size_t len = eol ? (size_t)(eol - p) : strlen(p);
        if (len > kl && !strncmp(p, key, kl) && p[kl] == '=') {
            size_t vl = len - kl - 1;
            if (vl && p[kl + 1 + vl - 1] == '\r') vl--;
            if (vl >= n) vl = n - 1;
            memcpy(out, p + kl + 1, vl);
            out[vl] = '\0';
            return 1;
        }
        p += len + (eol ? 1 : 0);
    }
    return 0;
}
static int fs_is(const char *key, const char *val) {
    char v[64];
    return fs_get(key, v, sizeof v) && !strcmp(v, val);
}

static char sf_skin_font[512];
static void sf_load_skin(void) {
    FILE *f = fopen("/mnt/sdcard/cubegm/skin/skin.txt", "r");
    if (!f) return;
    char line[600];
    while (fgets(line, sizeof line, f)) {
        unsigned v;
        if (sscanf(line, "text_color=0x%x", &v) == 1)      sf_col_text = sf_888_to_565(v);
        else if (sscanf(line, "selection_color=0x%x", &v) == 1) sf_col_sel = sf_888_to_565(v);
        else if (sscanf(line, "sel_text_color=0x%x", &v) == 1)  sf_col_seltext = sf_888_to_565(v);
        else if (!strncmp(line, "font=", 5)) {
            snprintf(sf_skin_font, sizeof sf_skin_font, "%s", line + 5);
            sf_skin_font[strcspn(sf_skin_font, "\r\n")] = '\0';
        }
    }
    fclose(f);
}

static int sf_font_try(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) return 0;
    fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
    if (n <= 0) { fclose(f); return 0; }
    unsigned char *buf = (unsigned char*)malloc(n);
    if (!buf) { fclose(f); return 0; }
    if (fread(buf, 1, n, f) != (size_t)n) { fclose(f); free(buf); return 0; }
    fclose(f);
    if (!stbtt_InitFont(&sf_font, buf, stbtt_GetFontOffsetForIndex(buf, 0))) {
        free(buf); return 0;
    }
    sf_font_buf = buf;
    return 1;
}

/* The same font as picoarch's menu: with an OnionOS theme, the theme's font
 * (FrogUI writes it to skin.txt) or ShallotsOS's Exo 2, drawn as written;
 * without one, FrogUI's GamePocket in capitals. */
static void sf_font_init(void) {
    if (sf_font_ready) return;
    sf_font_ready = -1;   /* tried */
    sf_load_skin();
    int onion = !fs_is("onion_theme", "off");
    const char *onion_faces[] = { sf_skin_font,
                                  "/mnt/sdcard/frogui/onion/fonts/Exo-2-Bold-Italic.ttf", NULL };
    const char *plain_faces[] = { "/mnt/sdcard/frogui/fonts/GamePocket-Regular-ZeroKern.ttf",
                                  "/mnt/sdcard/cubegm/fonts/GamePocket-Regular-ZeroKern.ttf",
                                  "/mnt/sdcard/frogui/fonts/BPreplayBold.otf", NULL };
    if (onion)
        for (int i = 0; onion_faces[i] && !sf_font_buf; i++)
            if (onion_faces[i][0] && sf_font_try(onion_faces[i])) sf_keep_case = 1;
    for (int i = 0; plain_faces[i] && !sf_font_buf; i++)
        sf_font_try(plain_faces[i]);
    if (!sf_font_buf) return;
    sf_font_ready = 1;
    sf_set_px(SF_FONT_PX);
}
static int sf_text_w(const char *s) {
    if (sf_font_ready != 1) return (int)strlen(s) * 8;
    int w = 0, prev = 0;
    for (; *s; s++) {
        sf_glyph_t *g = sf_glyph_get((unsigned char)*s);
        if (!g || !g->glyph_index) { w += 8; prev = 0; continue; }
        if (prev) w += (int)(stbtt_GetGlyphKernAdvance(&sf_font, prev, g->glyph_index) * sf_font_scale);
        w += (int)(g->advance * sf_font_scale); prev = g->glyph_index;
    }
    return w;
}
static int sf_font_h(void) { return (sf_font_ready == 1 && sf_cur) ? sf_cur->px : 8; }

/* Draw UTF-8/ASCII text at (x, y=top) in RGB565 colour, alpha-blended. */
static void sf_text(int x, int y, const char *s, uint16_t color) {
    if (sf_font_ready != 1) { port_printf(x, y, s); return; }
    int pen = x, prev = 0;
    int cr = (color >> 11) & 0x1f, cg = (color >> 5) & 0x3f, cb = color & 0x1f;
    for (; *s; s++) {
        sf_glyph_t *g = sf_glyph_get((unsigned char)*s);
        if (!g || !g->glyph_index) { pen += 8; prev = 0; continue; }
        if (prev) pen += (int)(stbtt_GetGlyphKernAdvance(&sf_font, prev, g->glyph_index) * sf_font_scale);
        if (g->w > 0 && g->h > 0) {
            for (int j = 0; j < g->h; j++) {
                int py = y + sf_font_asc + g->y0 + j;
                if (py < 0 || py >= sf_th) continue;
                uint16_t *row = sf_tgt + py * sf_tw;
                for (int i = 0; i < g->w; i++) {
                    int a = g->bitmap[j*g->w+i]; if (!a) continue;
                    int px = pen + g->x0 + i; if (px < 0 || px >= sf_tw) continue;
                    uint16_t d = row[px];
                    int dr=(d>>11)&0x1f, dg=(d>>5)&0x3f, db=d&0x1f, ia=255-a;
                    int rr=(cr*a+dr*ia)/255, rg=(cg*a+dg*ia)/255, rb=(cb*a+db*ia)/255;
                    row[px] = (uint16_t)((rr<<11)|(rg<<5)|rb);
                }
            }
        }
        pen += (int)(g->advance * sf_font_scale); prev = g->glyph_index;
    }
}
/* Themed-menu API exposed to frontend.c (its ShowMenu uses these). */
int  sf3000_menu_ready(void)  { sf_font_init(); return sf_font_ready == 1; }
int  sf3000_menu_font_h(void) { return sf_font_h(); }
int  sf3000_menu_text_w(const char *s) { return sf_text_w(s); }
void sf3000_menu_text(int x, int y, const char *s, uint16_t color) { sf_text(x, y, s, color); }
void sf3000_menu_pill(int x, int y, int w, int h, uint16_t c) { sf_fill_round(x, y, w, h, h/2, c); }
uint16_t sf3000_col_text(void)    { return sf_col_text; }
uint16_t sf3000_col_sel(void)     { return sf_col_sel; }
uint16_t sf3000_col_seltext(void) { return sf_col_seltext; }
void sf3000_menu_battery(void);

/* Battery for the menu (same sources as FrogUI/picoarch): /dev/check_adc1 =
 * level byte, /dev/check_adc5 = charge (~0 idle, ~140 charging). Persistent
 * O_RDWR fds. "Battery Colour Mode" read from frogui/settings.txt. */
static int sf_adc_read(int slot) {
    static int fd[2] = { -2, -2 };
    static const char *node[2] = { "/dev/check_adc1", "/dev/check_adc5" };
    if (slot < 0 || slot > 1) return -1;
    if (fd[slot] == -2) fd[slot] = open(node[slot], O_RDWR);
    if (fd[slot] < 0) return -1;
    unsigned char b = 0; lseek(fd[slot], 0, SEEK_SET);
    return (read(fd[slot], &b, 1) == 1) ? b : -1;
}
static int sf_batt_pct(int *charging) {
    /* Re-read ~every 5s; the menu redraws continuously. */
    static int cached = -1, cached_chg = 0; static time_t last = 0;
    time_t now = time(NULL);
    if (cached < 0 || now - last >= 5) {
        last = now;
        int a1 = sf_adc_read(0), a5 = sf_adc_read(1);
        cached_chg = (a5 >= 64) ? 1 : 0;
        cached = -1;
        if (a1 >= 0) {
            static const int rx[] = {64,153,224}, py[] = {0,50,100};
            if (a1 <= rx[0]) cached = 0;
            else { cached = 100; for (int i = 1; i < 3; i++) if (a1 <= rx[i]) {
                int sp = rx[i]-rx[i-1]; cached = py[i-1] + (py[i]-py[i-1])*(a1-rx[i-1])/(sp?sp:1); break; } }
        }
    }
    *charging = cached_chg;
    return cached;
}
static int sf_batt_color_mode(void) {
    static int c = -1;
    if (c < 0) { c = 0; FILE *f = fopen("/mnt/sdcard/frogui/settings.txt","r");
        if (f) { char l[128]; while (fgets(l,sizeof l,f)) if (!strncmp(l,"battery_color=on",16)){c=1;break;} fclose(f);} }
    return c;
}
/* Draw battery in the panel buffer, top-right of the menu header. */
static void sf_draw_battery(void) {
    int charging, pct = sf_batt_pct(&charging);
    if (pct < 0) return; if (pct > 100) pct = 100;
    int pad = 14, y = 12;
    int bw = 30, bh = 15, nub = 3, p = 2;
    int x = sf_tw - pad - bw - nub;
    uint16_t out = sf_col_text;
    uint16_t fillc;
    if (sf_batt_color_mode())
        fillc = charging ? 0x2FE6 : (pct>=70)?0x2FE6 : (pct>=30)?0x041F : 0xF800;
    else
        fillc = charging ? 0x2FE6 : (pct<=15)?0xF800 : sf_col_text;
    /* outline (rounded), hollow interior, nub, fill */
    sf_fill_round(x, y, bw, bh, 4, out);
    sf_fill_round(x+1, y+1, bw-2, bh-2, 3, 0x0000);
    for (int j = (bh-bh/2)/2; j < (bh+bh/2)/2; j++)
        for (int i = 0; i < nub; i++) { int py2=y+j; if(py2>=0&&py2<sf_th){ int px=x+bw+i; if(px<sf_tw) sf_tgt[py2*sf_tw+px]=out; } }
    int iw = bw - 2*p, fw = iw*pct/100; if (fw<0) fw=0; if (fw>iw) fw=iw;
    if (fw > 0) sf_fill_round(x+p, y+p, fw, bh-2*p, (bh-2*p)/2, fillc);
}
void sf3000_menu_battery(void) { sf_draw_battery(); }

/* Panel-res menu frame: point drawing at a panel-sized buffer + clear. All sf_
 * draws now land at device resolution (crisp), like TreeFrogUI. */
int sf3000_menu_begin(void) {
    if (!sf3000_menu_ready()) return 0;
    sf_read_panel();
    if (!sf_menu_fb) { sf_menu_fb = (uint16_t*)malloc((size_t)sf_pw * sf_ph * 2); if (!sf_menu_fb) return 0; }
    sf_tgt = sf_menu_fb; sf_tw = sf_pw; sf_th = sf_ph;
    memset(sf_menu_fb, 0, (size_t)sf_pw * sf_ph * 2);
    return 1;
}
/* Present the panel-res menu buffer 1:1 (driver scales panel->panel = crisp). */
void sf3000_menu_present(void) {
    extern void hwdisp_present(const void *, int, int, int);
    extern void hwdisp_set_target_aspect(int, int);
    extern void hwdisp_set_filter(int);
    if (!sf_menu_fb) return;
    hwdisp_set_target_aspect(0, 0);   /* fill panel, no pad */
    hwdisp_set_filter(0);
    /* disp_frame DMA-reads asynchronously. Never hand it sf_menu_fb: the next
     * scroll redraw clears and modifies that buffer while slower SF3500 units
     * may still be scanning it, producing diagonally displaced/flickering text.
     * Rotate stable copies exactly as picoarch does for SF3500 menu frames.
     * This is menu-only; the high-frequency PS1 gameplay path stays untouched. */
    size_t bytes = (size_t)sf_pw * sf_ph * sizeof(uint16_t);
    /* The menus redraw every 16 ms; presenting an unchanged frame tears on
     * SF3500's disp_frame path, so skip it. */
    static uint16_t *last;
    if (!last) last = (uint16_t*)malloc(bytes);
    if (last && sf_present_valid && !memcmp(last, sf_menu_fb, bytes)) {
        sf_tgt = SCREEN; sf_tw = SCREEN_WIDTH; sf_th = SCREEN_HEIGHT;
        return;
    }
    if (last) { memcpy(last, sf_menu_fb, bytes); sf_present_valid = 1; }
    if (!sf_menu_stage[0]) {
        for (int i = 0; i < SF_MENU_STAGE_COUNT; i++)
            sf_menu_stage[i] = (uint16_t*)malloc(bytes);
    }
    uint16_t *present = sf_menu_fb;
    if (sf_menu_stage[0] && sf_menu_stage[1] && sf_menu_stage[2]) {
        present = sf_menu_stage[sf_menu_stage_idx];
        sf_menu_stage_idx = (sf_menu_stage_idx + 1) % SF_MENU_STAGE_COUNT;
        memcpy(present, sf_menu_fb, bytes);
    }
    hwdisp_present(present, sf_pw, sf_ph, sf_pw * 2);
    sf_tgt = SCREEN; sf_tw = SCREEN_WIDTH; sf_th = SCREEN_HEIGHT;  /* restore */
}
int  sf3000_panel_w(void) { sf_read_panel(); return sf_pw; }
int  sf3000_panel_h(void) { sf_read_panel(); return sf_ph; }

/* Filled rounded rect (pill) for the selection highlight. */
static void sf_fill_round(int x, int y, int w, int h, int r, uint16_t c) {
    if (w <= 0 || h <= 0) return;
    for (int j = 0; j < h; j++) {
        int py = y + j; if (py < 0 || py >= sf_th) continue;
        int inset = 0;
        if (j < r)        { int d = r - j; inset = r - (int)(sqrtf((float)(r*r - d*d))); }
        else if (j >= h-r){ int d = j-(h-r)+1; inset = r - (int)(sqrtf((float)(r*r - d*d))); }
        uint16_t *row = sf_tgt + py * sf_tw;
        for (int i = inset; i < w-inset; i++) {
            int px = x + i; if (px < 0 || px >= sf_tw) continue;
            row[px] = c;
        }
    }
}

/* ==========================================================================
 * ShallotsOS: picoarch's in-game menu, keys, GameSwitcher, save slots,
 * power-off save and sleep, so PS1 works like every other system.
 *
 * The menu is drawn exactly as ShallotsOS's picoarch draws its own (same
 * proportions, colours, font and wording): the paused game behind a dark
 * left panel, the selected row on a full-width bar in the theme colour, Save
 * and Load picking their slot with Left/Right over the slot's screenshot,
 * and a hint bar along the bottom. Option pages use picoarch's list layout
 * (status bar with the game's name, values on the right with arrows).
 *
 * Keys follow FrogUI's "In-game Menu Keys": FN tap = menu, FN hold =
 * GameSwitcher (consoles without FN: SELECT+START tap / hold), or hold
 * SELECT = menu, hold START = GameSwitcher. SELECT+R2 / SELECT+L2 save and
 * load the current slot. The GameSwitcher and power-off saves use the
 * Switcher slot and the same files and markers as picoarch, so FrogUI's
 * GameSwitcher, Resume After Power-Off and Auto-Save/Load work unchanged.
 * ========================================================================== */
extern char CdromId[];
extern int cdrIsoMultidiskCount;
extern char sstatesdir[];
int state_load(int slot);
int state_save(int slot);
void config_save(void);
void config_save_game(void);
void config_delete_game(void);
int config_have_game(void);
int GameMenu(void);
void sdl_poll_reset(void);
int sf3000_open_core_settings(void);
int sf3000_open_gpu_settings(void);
int sf3000_open_spu_settings(void);
int sf3000_open_cheats(void);
int sf3000_swap_cd(void);
static void write_recent_screenshot(void);

#define CV_FN 16
#define SH_PCSX4ALL_BIN    "/mnt/sdcard/cubegm/pcsx4all"
#define SH_SWITCHER_FLAG   "/tmp/shallotsos_from_game"
#define SH_POWER_RESUME    "/mnt/sdcard/picoarch/resume_on_boot.txt"
#define SH_SHUTDOWN_SCRIPT "/mnt/sdcard/cubegm/shutdown.sh"
#define SH_HOLD_MS     600   /* FN / SELECT+START held this long = GameSwitcher */
#define SH_PROBE_MS    80    /* FN+volume check while FN is held */
#define SH_UP_POLLS    3     /* a button reads released this many polls in a row */

static int sh_argc;
static char **sh_argv;

static long long sh_ms(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (long long)t.tv_sec * 1000 + t.tv_nsec / 1000000;
}

/* ---- Device and FrogUI settings ------------------------------------------ */
static int sh_is_r36sx(void) {
    static int r = -1;
    if (r >= 0) return r;
    r = 0;
    const char *e = getenv("TF_DEVICE");
    if (e && !strcmp(e, "R36SX")) return r = 1;
    FILE *f = fopen("/tmp/tfdevice.env", "r");
    if (f) {
        char l[128];
        while (fgets(l, sizeof l, f)) if (strstr(l, "TF_DEVICE=R36SX")) { r = 1; break; }
        fclose(f);
    }
    return r;
}
/* Only the R36SX family has FN (overridable like picoarch). */
static int sh_has_fn(void) {
    static int r = -1;
    if (r >= 0) return r;
    r = sh_is_r36sx();
    const char *e = getenv("SF3000_HAS_FN");
    if (e && (e[0] == '1' || e[0] == 'y' || e[0] == 'Y')) r = 1;
    else if (e && (e[0] == '0' || e[0] == 'n' || e[0] == 'N')) r = 0;
    FILE *f = fopen("/mnt/sdcard/fn_enable", "r");
    if (f) { int c = fgetc(f); if (c == '1' || c == 'y' || c == 'Y') r = 1; else if (c == '0' || c == 'n' || c == 'N') r = 0; fclose(f); }
    return r;
}
/* 0 = FN, 1 = hold SELECT / START, 2 = SELECT+START. */
static int sh_game_keys(void) {
    int k = fs_is("game_keys", "hold") ? 1 : fs_is("game_keys", "combo") ? 2 : 0;
    return (k == 0 && !sh_has_fn()) ? 2 : k;
}
static int sh_switcher_on(void) { return !fs_is("game_switcher", "off"); }
static int sh_power_resume(void) { return !fs_is("power_resume", "off"); }
static int sh_autosave(void) { return fs_is("autosave_autoload", "on"); }
static int sh_own_sleep(void) { return fs_is("disable_sleep", "on"); }
static int sh_stock_battery(void) { return !fs_is("stock_battery", "off"); }

/* cubevol's volume and backlight: FN+volume changes one of them (the volume
 * keys never reach joy_key), so a change means FN was a shortcut. */
struct sh_pmem_req { unsigned short flag, id, len, pad; void *buf; };
#define SH_PMEM_GET 0x400c2602u
static int sh_fn_combo_probe(void) {
    unsigned char buf[260];
    int light = 0, sig = -1;
    int fd = open("/dev/persistentmem", O_RDONLY);
    if (fd < 0) return -1;
    struct sh_pmem_req vol = { 3, 0, 260, 0, buf };
    struct sh_pmem_req bl = { 1, 30, 1, 0, &light };
    if (ioctl(fd, SH_PMEM_GET, &vol) == 0 && ioctl(fd, SH_PMEM_GET, &bl) == 0)
        sig = (buf[0] << 8) | (light & 0xFF);
    close(fd);
    return sig;
}
static int sh_backlight_raw(void) {
    int light = 0, ret = -1;
    int fd = open("/dev/persistentmem", O_RDONLY);
    if (fd < 0) return -1;
    struct sh_pmem_req bl = { 1, 30, 1, 0, &light };
    if (ioctl(fd, SH_PMEM_GET, &bl) == 0) ret = light & 0xFF;
    close(fd);
    return ret;
}
static void sh_backlight_write(int raw) {
    int fd = open("/dev/backlight", O_RDWR);
    if (fd < 0) return;
    if (write(fd, &raw, sizeof raw) != sizeof raw) { /* nothing to do */ }
    close(fd);
}

/* ---- Save slots ------------------------------------------------------------
 * Positions as in picoarch: Auto (Auto-Save on exit), Switcher (GameSwitcher
 * and power-off), then user slots 1-30. User slot N is file N, so saves made
 * by the old menu (slot 1) stay where they were. Each state has a screenshot
 * next to it (<state>.bmp) for the menu's preview. */
#define SH_SLOT_AUTO     (-1)
#define SH_SLOT_SWITCHER (-2)
#define SH_USER_SLOTS 30
#define SH_POS_AUTO 0
#define SH_POS_SWITCHER 1
#define SH_POS_FIRST_USER 2
#define SH_POSITIONS (SH_POS_FIRST_USER + SH_USER_SLOTS)

static void sh_state_path(char *out, size_t n, int slot) {
    /* By disc ID like before; a disc without one by its file name. */
    char stem[256];
    const char *id = CdromId;
    if (!id[0]) {
        const char *b = strrchr(g_rom_path, '/');
        snprintf(stem, sizeof stem, "%s", b ? b + 1 : (g_rom_path[0] ? g_rom_path : "game"));
        char *dot = strrchr(stem, '.');
        if (dot && dot != stem) *dot = '\0';
        id = stem;
    }
    if (slot == SH_SLOT_AUTO)          snprintf(out, n, "%s/%s.auto.sav", sstatesdir, id);
    else if (slot == SH_SLOT_SWITCHER) snprintf(out, n, "%s/%s.switcher.sav", sstatesdir, id);
    else                               snprintf(out, n, "%s/%s.%d.sav", sstatesdir, id, slot);
}
static int sh_pos_slot(int pos) {
    return pos == SH_POS_AUTO ? SH_SLOT_AUTO : pos == SH_POS_SWITCHER ? SH_SLOT_SWITCHER
         : pos - SH_POS_FIRST_USER + 1;
}
static int sh_state_exists(int slot) {
    char p[PATH_MAX + 64];
    sh_state_path(p, sizeof p, slot);
    return access(p, R_OK) == 0;
}

/* 24-bit BMP of an RGB565 picture, bottom-up. */
static int sh_write_bmp(const char *path, const uint16_t *px, int w, int h) {
    if (!px || w <= 0 || h <= 0) return 0;
    FILE *f = fopen(path, "wb");
    if (!f) return 0;
    int row = (w * 3 + 3) & ~3, imgsz = row * h, total = 54 + imgsz;
    unsigned char hd[54] = {0};
    hd[0]='B'; hd[1]='M';
    hd[2]=total; hd[3]=total>>8; hd[4]=total>>16; hd[5]=total>>24;
    hd[10]=54; hd[14]=40;
    hd[18]=w; hd[19]=w>>8; hd[20]=w>>16; hd[21]=w>>24;
    hd[22]=h; hd[23]=h>>8; hd[24]=h>>16; hd[25]=h>>24;
    hd[26]=1; hd[28]=24;
    hd[34]=imgsz; hd[35]=imgsz>>8; hd[36]=imgsz>>16; hd[37]=imgsz>>24;
    fwrite(hd, 1, 54, f);
    unsigned char *line = (unsigned char*)malloc(row);
    if (!line) { fclose(f); return 0; }
    for (int y = h - 1; y >= 0; y--) {
        const uint16_t *s = px + (size_t)y * w;
        int o = 0;
        for (int x = 0; x < w; x++) {
            uint16_t c = s[x];
            line[o++] = (c & 0x1F) << 3;
            line[o++] = ((c >> 5) & 0x3F) << 2;
            line[o++] = ((c >> 11) & 0x1F) << 3;
        }
        while (o < row) line[o++] = 0;
        fwrite(line, 1, row, f);
    }
    free(line);
    fclose(f);
    return 1;
}
/* Reads a 24-bit BMP (as written above) into a new RGB565 buffer. */
static uint16_t *sh_read_bmp(const char *path, int *w, int *h) {
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    unsigned char hd[54];
    uint16_t *out = NULL;
    if (fread(hd, 1, 54, f) == 54 && hd[0] == 'B' && hd[1] == 'M' && hd[28] == 24) {
        int off = hd[10] | hd[11] << 8 | hd[12] << 16 | hd[13] << 24;
        int bw = hd[18] | hd[19] << 8 | hd[20] << 16 | hd[21] << 24;
        int bh = hd[22] | hd[23] << 8 | hd[24] << 16 | hd[25] << 24;
        int top_down = bh < 0;
        if (top_down) bh = -bh;
        int row = (bw * 3 + 3) & ~3;
        unsigned char *line = malloc(row > 0 ? row : 1);
        if (bw > 0 && bh > 0 && bw <= 1024 && bh <= 1024 && line &&
            (out = malloc((size_t)bw * bh * 2)) && fseek(f, off, SEEK_SET) == 0) {
            for (int y = 0; y < bh; y++) {
                if (fread(line, 1, row, f) != (size_t)row) { free(out); out = NULL; break; }
                uint16_t *d = out + (size_t)(top_down ? y : bh - 1 - y) * bw;
                for (int x = 0; x < bw; x++)
                    d[x] = (uint16_t)(((line[x*3+2] >> 3) << 11) | ((line[x*3+1] >> 2) << 5) | (line[x*3] >> 3));
            }
            if (out) { *w = bw; *h = bh; }
        }
        free(line);
    }
    fclose(f);
    return out;
}

/* The paused game: SCREEN copied when the menu opens (the PCSX settings pages
 * clear SCREEN), used as the menu background and for state screenshots. */
static uint16_t *sh_bg;
static int sh_bg_w, sh_bg_h, sh_bg_valid;
static int sh_in_menu;   /* SCREEN may have been cleared by a PCSX page */
static void sh_bg_capture(void) {
    if (!SCREEN || SCREEN_WIDTH <= 0 || SCREEN_HEIGHT <= 0) return;
    size_t n = (size_t)SCREEN_WIDTH * SCREEN_HEIGHT;
    if (!sh_bg || sh_bg_w * sh_bg_h < (int)n) {
        free(sh_bg);
        sh_bg = malloc(n * 2);
        if (!sh_bg) return;
    }
    memcpy(sh_bg, SCREEN, n * 2);
    sh_bg_w = SCREEN_WIDTH; sh_bg_h = SCREEN_HEIGHT; sh_bg_valid = 1;
}

/* Save or load a slot; a save also writes its screenshot. 0 = done. */
static int sh_state_save(int slot, const uint16_t *shot, int sw, int sh) {
    mkdir(sstatesdir, 0755);
    char p[PATH_MAX + 64];
    sh_state_path(p, sizeof p, slot);
    if (SaveState(p) != 0) return -1;
    strncat(p, ".bmp", sizeof p - strlen(p) - 1);
    sh_write_bmp(p, shot, sw, sh);
    return 0;
}
static int sh_state_load(int slot) {
    char p[PATH_MAX + 64];
    sh_state_path(p, sizeof p, slot);
    if (access(p, R_OK) != 0) return -1;
    return LoadState(p) == 0 ? 0 : -1;
}

/* The ".resume" marker next to the Switcher state: resume once on launch. */
static void sh_marker_path(char *out, size_t n) {
    sh_state_path(out, n, SH_SLOT_SWITCHER);
    strncat(out, ".resume", n - strlen(out) - 1);
}
static void sh_mark_resume(int on) {
    char p[PATH_MAX + 80];
    sh_marker_path(p, sizeof p);
    if (on) { FILE *f = fopen(p, "w"); if (f) fclose(f); }
    else unlink(p);
}

/* ---- In-game toast (picoarch's HUD card, top left of the game frame) ------ */
static char sh_toast_msg[64];
static long long sh_toast_until;
static void sh_toast(const char *msg) {
    snprintf(sh_toast_msg, sizeof sh_toast_msg, "%s", msg);
    sh_toast_until = sh_ms() + 2000;
}
static void sh_toast_draw(void) {
    if (!sh_toast_msg[0] || sh_ms() > sh_toast_until || !SCREEN) { sh_toast_msg[0] = 0; return; }
    if (sf_font_ready != 1) return;
    uint16_t *save_t = sf_tgt; int save_w = sf_tw, save_h = sf_th;
    sf_tgt = SCREEN; sf_tw = SCREEN_WIDTH; sf_th = SCREEN_HEIGHT;
    int H = SCREEN_HEIGHT;
    float px = 20.0f * H / 480; if (px < 9) px = 9;
    sf_set_px(px);
    int tw = sf_text_w(sh_toast_msg), pad = (int)(px * 0.7f) + 1;
    int w = tw + pad * 2, h = (int)(px * 1.8f) + 1, m = H / 60 + 2, r = h / 3;
    for (int y = 0; y < h && m + y < H; y++) {
        uint16_t *d = SCREEN + (size_t)(m + y) * SCREEN_WIDTH + m;
        for (int x = 0; x < w && m + x < SCREEN_WIDTH; x++) {
            int dx = x < r ? r - x : (x >= w - r ? x - (w - r - 1) : 0);
            int dy = y < r ? r - y : (y >= h - r ? y - (h - r - 1) : 0);
            if (dx * dx + dy * dy > r * r) continue;
            unsigned c = d[x], k = 45;   /* 210/255 dark */
            d[x] = (uint16_t)(((((c >> 11) & 31) * k / 255) << 11) |
                              ((((c >> 5) & 63) * k / 255) << 5) | ((c & 31) * k / 255));
        }
    }
    sf_text(m + pad, m + h / 2 - (sf_cur->asc - sf_cur->cap / 2), sh_toast_msg, 0xFFFF);
    sf_tgt = save_t; sf_tw = save_w; sf_th = save_h;
}

/* ---- Drawing, as picoarch's menu.c --------------------------------------- */
#define SH_K (sf_th >= 480 ? 2 : 1)
static void sh_fill(int x, int y, int w, int h, uint16_t c) {
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > sf_tw) w = sf_tw - x;
    if (y + h > sf_th) h = sf_th - y;
    for (int j = 0; j < h; j++) {
        uint16_t *r = sf_tgt + (size_t)(y + j) * sf_tw + x;
        for (int i = 0; i < w; i++) r[i] = c;
    }
}
static void sh_arrow(int cx, int cy, int size, int dir, uint16_t c) {
    for (int i = 0; i < size; i++) {
        int half = i * 2 / 3;
        int x = dir < 0 ? cx - size / 2 + i : cx + size / 2 - i;
        sh_fill(x, cy - half, 1, half * 2 + 1, c);
    }
}
static void sh_text_mid(int x, int cy, const char *t, uint16_t c) {
    if (sf_font_ready != 1 || !sf_cur) return;
    sf_text(x, cy - (sf_cur->asc - sf_cur->cap / 2), t, c);
}
static void sh_round(int x, int y, int w, int h, int r, uint16_t c) {
    for (int j = 0; j < h; j++) {
        int dy = j < r ? r - j : (j >= h - r ? j - (h - r - 1) : 0);
        int inset = 0;
        if (dy) { int k = 0; while (k < r && (r - k) * (r - k) + dy * dy > r * r) k++; inset = k; }
        sh_fill(x + inset, y + j, w - inset * 2, 1, c);
    }
}
static int sh_hint(int x, int cy, int size, const char *btn, const char *label, uint16_t accent) {
    sh_round(x, cy - size / 2, size, size, size / 4, accent);
    int bw = sf_text_w(btn);
    sh_text_mid(x + (size - bw) / 2, cy, btn, 0xFFFF);
    x += size + size / 4;
    sh_text_mid(x, cy, label, 0xFFFF);
    return x + sf_text_w(label) + size;
}
static void sh_thumb(const uint16_t *src, int sw, int sh, int dx, int dy, int dw, int dh) {
    if (!src || sw <= 0 || sh <= 0) { sh_fill(dx, dy, dw, dh, 0x0000); return; }
    for (int y = 0; y < dh; y++) {
        int py = dy + y; if (py < 0 || py >= sf_th) continue;
        const uint16_t *srow = src + (size_t)(y * sh / dh) * sw;
        uint16_t *drow = sf_tgt + (size_t)py * sf_tw;
        for (int x = 0; x < dw; x++) {
            int px = dx + x; if (px < 0 || px >= sf_tw) continue;
            drow[px] = srow[x * sw / dw];
        }
    }
}
static void sh_cut(char *label, int room) {
    while (label[0] && sf_text_w(label) > room) {
        size_t len = strlen(label);
        if (len <= 4) { label[0] = '\0'; break; }
        label[len - 4] = '\0';
        strcat(label, "...");
    }
}
static void sh_game_title(char *out, size_t n) {
    const char *base = strrchr(g_rom_path, '/');
    base = base ? base + 1 : g_rom_path;
    snprintf(out, n, "%s", base);
    char *dot = strrchr(out, '.');
    if (dot && dot != out) *dot = '\0';
    if (!out[0]) snprintf(out, n, "PCSX4ALL");
}
/* picoarch's battery glyph, top right (the stock OSD shows it otherwise). */
static void sh_battery(void) {
    if (sh_stock_battery()) return;
    int charging, pct = sf_batt_pct(&charging);
    if (pct < 0) return;
    if (pct > 100) pct = 100;
    int k = SH_K, bw = 22 * k, bh = 11 * k, pad = 2 * k, nub = 2 * k;
    int x = sf_tw - 8 * k - bw - nub, y = 6 * k;
    uint16_t out = sf_col_text, bg = sf_col_seltext, fillc;
    if (sf_batt_color_mode())
        fillc = charging ? 0x2FE6 : (pct >= 70) ? 0x2FE6 : (pct >= 30) ? 0x041F : 0xF800;
    else
        fillc = charging ? 0x2FE6 : (pct <= 15 ? 0xF800 : out);
    sh_fill(x, y, bw, bh, out);
    sh_fill(x + 1, y + 1, bw - 2, bh - 2, bg);
    sh_fill(x + bw, y + bh / 2 - 2, nub, 4, out);
    int inner = bw - 2 * pad, fw = inner * pct / 100;
    sh_fill(x + pad, y + pad, fw, bh - 2 * pad, fillc);
    if (charging) for (int j = pad; j < bh - pad; j++) sh_fill(x + bw / 2 + (j < bh / 2 ? -1 : 1), y + j, 1, 1, 0xFFFF);
}
static int sh_status_bar(const char *title, uint16_t bg, uint16_t fg) {
    int W = sf_tw, H = sf_th, h = H * 40 / 480;
    if (h < 22 * SH_K) h = 22 * SH_K;
    sh_fill(0, 0, W, h, bg);
    float px = h * 0.55f;
    sf_set_px(px < 14 * SH_K ? 14 * SH_K : px);
    char label[128];
    snprintf(label, sizeof label, "%s", title);
    sh_cut(label, W - W / 32 - 48);
    sh_text_mid(W / 32, h / 2, label, fg);
    return h;
}

/* Menu input: edges, with key repeat on the d-pad. */
typedef struct { uint32_t prev; long long rep_at; } sh_input_t;
enum { SH_UP = 1, SH_DOWN = 2, SH_LEFT = 4, SH_RIGHT = 8, SH_A = 16, SH_B = 32, SH_X = 64, SH_Y = 128 };
static void sh_input_begin(sh_input_t *in) {
    in->prev = cv_keys ? *cv_keys : 0;
    in->rep_at = 0;
}
static int sh_input_poll(sh_input_t *in) {
    if (!cv_keys) return 0;
    uint32_t k = *cv_keys, edge = k & ~in->prev;
    const uint32_t dpad = (1u << CV_UP) | (1u << CV_DOWN) | (1u << CV_LEFT) | (1u << CV_RIGHT);
    long long now = sh_ms();
    if (edge & dpad) in->rep_at = now + 400;
    else if ((k & dpad) && in->rep_at && now >= in->rep_at) { edge |= k & dpad; in->rep_at = now + 80; }
    in->prev = k;
    int r = 0;
    if (edge & (1u << CV_UP)) r |= SH_UP;
    if (edge & (1u << CV_DOWN)) r |= SH_DOWN;
    if (edge & (1u << CV_LEFT)) r |= SH_LEFT;
    if (edge & (1u << CV_RIGHT)) r |= SH_RIGHT;
    if (edge & (1u << CV_A)) r |= SH_A;
    if (edge & (1u << CV_B)) r |= SH_B;
    if (edge & (1u << CV_X)) r |= SH_X;
    if (edge & (1u << CV_Y)) r |= SH_Y;
    return r;
}
static void sh_wait_release(void) {
    for (int i = 0; cv_keys && (*cv_keys & 0x1FFFFu) && i < 300; i++) usleep(10000);
}

/* ---- picoarch's list page ---------------------------------------------- */
typedef struct { char name[64]; char value[64]; int can_change; } sh_row_t;
static char sh_note[96];
static long long sh_note_until;
static void sh_set_note(const char *msg) {
    snprintf(sh_note, sizeof sh_note, "%s", msg);
    sh_note_until = sh_ms() + 2048;
}
static void sh_draw_list(const sh_row_t *rows, int n, int sel, const char *ok_label) {
    if (!sf3000_menu_begin()) return;   /* loads the theme colours first */
    const uint16_t ACCENT = sf_col_sel, SEL_TEXT = sf_col_seltext;
    const uint16_t PANEL = 0x18E3, FOOTER = 0x10A2, DIM = 0xA534;
    int W = sf_tw, H = sf_th, foot_h = H * 56 / 480, pad = W / 32, K = SH_K;
    sh_fill(0, 0, W, H, PANEL);
    char game[128];
    sh_game_title(game, sizeof game);
    int top = sh_status_bar(game, FOOTER, DIM);
    int area_h = H - foot_h - top, rh = area_h / 6;
    int per_page = area_h / rh;
    int first = sel - per_page / 2;
    if (first > n - per_page) first = n - per_page;
    if (first < 0) first = 0;
    float px = rh * 0.42f;
    if (px < 18 * K) px = 18 * K;
    if (px > 28 * K) px = 28 * K;
    sf_set_px(px);
    for (int r = 0; r < per_page && first + r < n; r++) {
        const sh_row_t *row = &rows[first + r];
        int on = first + r == sel, y = top + r * rh;
        uint16_t tc = on ? SEL_TEXT : 0xFFFF;
        if (on) sh_fill(0, y, W, rh, ACCENT);
        int right = W - pad, vw = sf_text_w(row->value), a = rh / 4;
        if (row->value[0]) {
            if (row->can_change) { sh_arrow(right - a / 2, y + rh / 2, a, 1, tc); right -= a * 2; }
            sh_text_mid(right - vw, y + rh / 2, row->value, on ? tc : DIM);
            if (row->can_change) sh_arrow(right - vw - a, y + rh / 2, a, -1, tc);
        }
        char label[128];
        snprintf(label, sizeof label, "%s", row->name);
        sh_cut(label, (row->value[0] ? right - vw - a * 3 : W - pad) - pad);
        sh_text_mid(pad, y + rh / 2, label, tc);
    }
    sh_fill(0, H - foot_h, W, foot_h, FOOTER);
    int cy = H - foot_h / 2, bsz = foot_h * 6 / 10;
    if (sh_note[0] && sh_ms() > sh_note_until) sh_note[0] = 0;
    if (sh_note[0]) {
        sh_text_mid(pad, cy, sh_note, 0xFFFF);
    } else {
        int hx = sh_hint(pad, cy, bsz, "A", ok_label ? ok_label : "OK", ACCENT);
        hx = sh_hint(hx, cy, bsz, "B", "BACK", ACCENT);
        int a = bsz / 3;
        sh_arrow(hx + a / 2, cy, a, -1, 0xFFFF);
        sh_arrow(hx + a * 2, cy, a, 1, 0xFFFF);
        sh_text_mid(hx + a * 3 + a / 2, cy, "CHANGE", 0xFFFF);
        if (n > per_page) {
            char pos[16];
            snprintf(pos, sizeof pos, "%d/%d", sel + 1, n);
            sh_text_mid(W - pad - sf_text_w(pos), cy, pos, DIM);
        }
    }
    sh_battery();
    sf3000_menu_present();
}

/* frontend.c's PCSX settings pages (Core / GPU / SPU / Cheats) draw through
 * this, so they look like picoarch's Options pages too. */
void sf3000_list_draw(int n, const char *const *names, const char *const *values,
                      const int *can_change, int sel) {
    static sh_row_t rows[64];
    if (n > 64) n = 64;
    for (int i = 0; i < n; i++) {
        snprintf(rows[i].name, sizeof rows[i].name, "%s", names[i] ? names[i] : "");
        snprintf(rows[i].value, sizeof rows[i].value, "%s", values[i] ? values[i] : "");
        rows[i].can_change = can_change[i];
    }
    sh_draw_list(rows, n, sel, NULL);
}

static void sh_onoff(char *out, size_t n, int v) { snprintf(out, n, "%s", v ? "On" : "Off"); }

/* ---- Options ---------------------------------------------------------------
 * As picoarch: Audio and video, Emulator options, Player controls, Save
 * config. The PCSX pages live under Audio and video and Emulator options. */
static void sh_apply_scale(void) {
    last_fb_y_off = -1; last_fb_y_len = -1; last_h_blend = -1;
    if (scale_mode == 1) { hwdisp_set_target_aspect(0, 0); hwdisp_set_filter(0); }
    else { hwdisp_set_target_aspect(16, 9); hwdisp_set_filter(1); }
}
static void sh_run_frontend(int (*fn)(void)) {
    sh_wait_release();
    sdl_poll_reset();
    fn();
    sf_present_valid = 0;
    sh_wait_release();
    last_fb_y_off = -1; last_fb_y_len = -1;
}

static void sh_menu_video(void) {
    int sel = 0;
    sh_input_t in; sh_input_begin(&in);
    for (;;) {
        sh_row_t r[8]; memset(r, 0, sizeof r);
        int n = 0;
        snprintf(r[n].name, 64, "Aspect ratio");
        snprintf(r[n].value, 64, "%s", scale_mode == 1 ? "Fullscreen" : "Aspect-Correct"); r[n++].can_change = 1;
        snprintf(r[n].name, 64, "Gamma");
        if (gamma_percent <= 100) snprintf(r[n].value, 64, "Off");
        else snprintf(r[n].value, 64, "%d.%02d", gamma_percent / 100, gamma_percent % 100);
        r[n++].can_change = 1;
        snprintf(r[n].name, 64, "Pixel skip (speed)"); sh_onoff(r[n].value, 64, gpu_unai_config_ext.pixel_skip); r[n++].can_change = 1;
        snprintf(r[n].name, 64, "Interlace (speed)"); sh_onoff(r[n].value, 64, gpu_unai_config_ext.ilace_force); r[n++].can_change = 1;
        snprintf(r[n].name, 64, "Hi-Res fix"); sh_onoff(r[n].value, 64, hires_fix); r[n++].can_change = 1;
        snprintf(r[n++].name, 64, "GPU settings");
        snprintf(r[n++].name, 64, "SPU settings");
        sh_draw_list(r, n, sel, NULL);
        usleep(16000);
        int k = sh_input_poll(&in);
        if (k & SH_B) return;
        if (k & SH_UP) sel = (sel + n - 1) % n;
        if (k & SH_DOWN) sel = (sel + 1) % n;
        int step = (k & SH_RIGHT) ? 1 : (k & SH_LEFT) ? -1 : (k & SH_A) ? 1 : 0;
        if (!step) continue;
        switch (sel) {
        case 0: scale_mode = !scale_mode; sh_apply_scale(); break;
        case 1:
            if (k & SH_A) gamma_percent = gamma_percent > 100 ? 100 : 130;
            else gamma_percent += step * 10;
            if (gamma_percent < 100) gamma_percent = 100;
            if (gamma_percent > 250) gamma_percent = 250;
            break;
        case 2: gpu_unai_config_ext.pixel_skip = !gpu_unai_config_ext.pixel_skip; break;
        case 3: gpu_unai_config_ext.ilace_force = !gpu_unai_config_ext.ilace_force; break;
        case 4: hires_fix = !hires_fix; break;
        case 5: if (k & SH_A) { sh_run_frontend(sf3000_open_gpu_settings); sh_input_begin(&in); } break;
        case 6: if (k & SH_A) { sh_run_frontend(sf3000_open_spu_settings); sh_input_begin(&in); } break;
        }
    }
}

static void sh_menu_controls(void) {
    int sel = 0;
    sh_input_t in; sh_input_begin(&in);
    for (;;) {
        sh_row_t r[PB_COUNT]; memset(r, 0, sizeof r);
        for (int i = 0; i < PB_COUNT; i++) {
            snprintf(r[i].name, 64, "%s", pb_name[i]);
            snprintf(r[i].value, 64, "%s", cv_bit_name(pb_bind[i]));
        }
        sh_draw_list(r, PB_COUNT, sel, "BIND");
        usleep(16000);
        int k = sh_input_poll(&in);
        if (k & SH_B) return;
        if (k & SH_UP) sel = (sel + PB_COUNT - 1) % PB_COUNT;
        if (k & SH_DOWN) sel = (sel + 1) % PB_COUNT;
        if (k & SH_A) {
            char msg[64];
            snprintf(msg, sizeof msg, "Press a button for %s", pb_name[sel]);
            snprintf(sh_note, sizeof sh_note, "%s", msg);
            sh_note_until = sh_ms() + 60000;
            sh_draw_list(r, PB_COUNT, sel, "BIND");
            sh_wait_release();
            for (int done = 0; !done && cv_keys; usleep(10000)) {
                uint32_t kk = *cv_keys;
                for (int j = 0; j < CV_BTNS_N; j++)
                    if (btn(kk, cv_btns[j].bit)) { pb_bind[sel] = cv_btns[j].bit; done = 1; break; }
            }
            sh_wait_release();
            sh_note[0] = 0;
            sh_input_begin(&in);
        }
    }
}

static void sh_menu_config(void) {
    int sel = 0;
    sh_input_t in; sh_input_begin(&in);
    for (;;) {
        sh_row_t r[3]; memset(r, 0, sizeof r);
        snprintf(r[0].name, 64, "Save global config");
        snprintf(r[1].name, 64, "Save game config");
        if (config_have_game()) snprintf(r[1].value, 64, "*");
        snprintf(r[2].name, 64, "Delete game config");
        sh_draw_list(r, 3, sel, NULL);
        usleep(16000);
        int k = sh_input_poll(&in);
        if (k & SH_B) return;
        if (k & SH_UP) sel = (sel + 2) % 3;
        if (k & SH_DOWN) sel = (sel + 1) % 3;
        if (k & SH_A) {
            if (sel == 0) { config_save(); sh_set_note("Saved global config"); }
            if (sel == 1) { config_save_game(); sh_set_note("Saved game config"); }
            if (sel == 2) { config_delete_game(); sh_set_note("Deleted game config"); }
        }
    }
}

static void sh_menu_options(void) {
    static const char *names[] = { "Audio and video", "Emulator options", "Player controls", "Save config" };
    int sel = 0, n = 4;
    sh_input_t in; sh_input_begin(&in);
    for (;;) {
        sh_row_t r[4]; memset(r, 0, sizeof r);
        for (int i = 0; i < n; i++) snprintf(r[i].name, 64, "%s", names[i]);
        sh_draw_list(r, n, sel, NULL);
        usleep(16000);
        int k = sh_input_poll(&in);
        if (k & SH_B) return;
        if (k & SH_UP) sel = (sel + n - 1) % n;
        if (k & SH_DOWN) sel = (sel + 1) % n;
        if (k & SH_A) {
            sh_wait_release();
            if (sel == 0) sh_menu_video();
            if (sel == 1) sh_run_frontend(sf3000_open_core_settings);
            if (sel == 2) sh_menu_controls();
            if (sel == 3) sh_menu_config();
            sh_wait_release();
            sh_input_begin(&in);
        }
    }
}

/* ---- Leaving the game -------------------------------------------------------
 * Exit Game: Auto-Save when that setting is on, drop the resume marker, and
 * return to FrogUI. Reset: start the game again (a fresh process). */
static void sh_exit_game(void) {
    if (sh_autosave() && sh_bg_valid)
        sh_state_save(SH_SLOT_AUTO, sh_bg, sh_bg_w, sh_bg_h);
    sh_mark_resume(0);
    sync();
    exit(0);
}
static void sh_reset_game(void) {
    sh_mark_resume(0);
    sync();
    if (sh_argc >= 2) {
        fflush(NULL);
        hwdisp_deinit();
        execl(sh_argv[0], sh_argv[0], sh_argv[1], (char *)NULL);
    }
}

/* ---- Main menu (picoarch's minui_draw_main) ------------------------------- */
enum { SH_M_CONTINUE, SH_M_SAVE, SH_M_LOAD, SH_M_DISC, SH_M_CHEATS, SH_M_OPTIONS, SH_M_RESET, SH_M_EXIT };
static int sh_save_pos = SH_POS_FIRST_USER, sh_load_pos = SH_POS_FIRST_USER;
static int sh_slot_used[SH_POSITIONS];
static uint16_t *sh_shot; static int sh_shot_w, sh_shot_h, sh_shot_for = -2;

static void sh_slot_label(int pos, char *out, size_t n) {
    if (pos == SH_POS_AUTO) snprintf(out, n, "Auto");
    else if (pos == SH_POS_SWITCHER) snprintf(out, n, "Switcher");
    else snprintf(out, n, "%d", pos - SH_POS_FIRST_USER + 1);
}
static void sh_slots_scan(void) {
    for (int p = 0; p < SH_POSITIONS; p++) sh_slot_used[p] = sh_state_exists(sh_pos_slot(p));
    sh_shot_for = -2;
}
static int sh_shot_load(int pos) {
    if (sh_shot_for == pos) return sh_shot != NULL;
    sh_shot_for = pos;
    free(sh_shot); sh_shot = NULL;
    if (!sh_slot_used[pos]) return 0;
    char p[PATH_MAX + 64];
    sh_state_path(p, sizeof p, sh_pos_slot(pos));
    strncat(p, ".bmp", sizeof p - strlen(p) - 1);
    sh_shot = sh_read_bmp(p, &sh_shot_w, &sh_shot_h);
    return sh_shot != NULL;
}

static void sh_draw_main(const int *ids, const char *const *names, int n, int sel) {
    if (!sf3000_menu_begin()) return;   /* loads the theme colours first */
    const uint16_t ACCENT = sf_col_sel, SEL_TEXT = sf_col_seltext;
    const uint16_t PANEL = 0x18E3, FOOTER = 0x10A2, DIM = 0x8C51;
    int W = sf_tw, H = sf_th, K = SH_K;
    int foot_h = H * 56 / 480, area_h = H - foot_h, panel_w = W / 2;
    int sel_id = ids[sel], slot_row = sel_id == SH_M_SAVE || sel_id == SH_M_LOAD;
    sh_fill(0, 0, W, H, 0x0000);
    int bh = area_h, bw = bh * 4 / 3;
    if (bw > W) { bw = W; bh = bw * 3 / 4; }
    int bx = (W - bw) / 2, by = (area_h - bh) / 2;
    int shown = sel_id == SH_M_SAVE ? sh_save_pos : sh_load_pos;
    if (slot_row) {
        if (sh_shot_load(shown)) sh_thumb(sh_shot, sh_shot_w, sh_shot_h, bx, by, bw, bh);
    } else if (sh_bg_valid) {
        sh_thumb(sh_bg, sh_bg_w, sh_bg_h, bx, by, bw, bh);
    }
    int rh = area_h / (n > 6 ? n : 6);
    float px = rh * 0.42f;
    if (px < 18 * K) px = 18 * K;
    if (px > 30 * K) px = 30 * K;
    sf_set_px(px);
    sh_fill(0, 0, panel_w, area_h, PANEL);
    int pad = W / 32;
    for (int i = 0; i < n; i++) {
        int on = i == sel, y = i * rh;
        uint16_t tc = on ? SEL_TEXT : 0xFFFF;
        if (on) sh_fill(0, y, panel_w, rh, ACCENT);
        sh_text_mid(pad, y + rh / 2, names[i], tc);
        if (ids[i] == SH_M_SAVE || ids[i] == SH_M_LOAD) {
            char num[24];
            sh_slot_label(ids[i] == SH_M_SAVE ? sh_save_pos : sh_load_pos, num, sizeof num);
            int nw = sf_text_w(num), a = rh / 4;
            int right = panel_w - pad, cx = right - a - nw / 2 - a;
            sh_arrow(right - a / 2, y + rh / 2, a, 1, tc);
            sh_text_mid(cx - nw / 2, y + rh / 2, num, tc);
            sh_arrow(cx - nw / 2 - a - a / 2, y + rh / 2, a, -1, tc);
        }
    }
    if (slot_row && !sh_slot_used[shown]) {
        const char *empty = "Empty";
        sh_text_mid(panel_w + (W - panel_w - sf_text_w(empty)) / 2, area_h / 2, empty, DIM);
    }
    sh_fill(0, area_h, W, foot_h, FOOTER);
    const char *ok = sel_id == SH_M_SAVE ? "SAVE" : sel_id == SH_M_LOAD ? "LOAD" : "OPEN";
    int hx = pad, cy = area_h + foot_h / 2, bsz = foot_h * 6 / 10;
    hx = sh_hint(hx, cy, bsz, "A", ok, ACCENT);
    hx = sh_hint(hx, cy, bsz, "B", "BACK", ACCENT);
    if (slot_row) {
        int a = bsz / 3;
        sh_arrow(hx + a / 2, cy, a, -1, 0xFFFF);
        sh_arrow(hx + a * 2, cy, a, 1, 0xFFFF);
        sh_text_mid(hx + a * 3 + a / 2, cy, "SLOT", 0xFFFF);
    }
    sh_battery();
    sf3000_menu_present();
}

static void sh_menu(void) {
    int ids[8], n = 0;
    const char *names[8];
    ids[n] = SH_M_CONTINUE; names[n++] = "Continue";
    ids[n] = SH_M_SAVE;     names[n++] = "Save";
    ids[n] = SH_M_LOAD;     names[n++] = "Load";
    if (cdrIsoMultidiskCount > 1) { ids[n] = SH_M_DISC; names[n++] = "Disc control"; }
    if (cheat_get() && cheat_get()->num_entries > 0) { ids[n] = SH_M_CHEATS; names[n++] = "Cheats"; }
    ids[n] = SH_M_OPTIONS;  names[n++] = "Options";
    ids[n] = SH_M_RESET;    names[n++] = "Reset";
    ids[n] = SH_M_EXIT;     names[n++] = "Exit Game";
    static int sel = 0;
    if (sel >= n) sel = 0;
    sf_present_valid = 0;   /* the game is on screen */
    sh_in_menu = 1;
    sh_bg_capture();
    sh_slots_scan();
    sh_wait_release();
    sh_input_t in; sh_input_begin(&in);
    for (;;) {
        sh_draw_main(ids, names, n, sel);
        usleep(16000);
        int k = sh_input_poll(&in);
        if (k & SH_B) break;
        if (k & SH_UP) sel = (sel + n - 1) % n;
        if (k & SH_DOWN) sel = (sel + 1) % n;
        int id = ids[sel];
        if ((k & (SH_LEFT | SH_RIGHT)) && (id == SH_M_SAVE || id == SH_M_LOAD)) {
            int step = (k & SH_RIGHT) ? 1 : -1;
            if (id == SH_M_SAVE) {
                sh_save_pos += step;
                if (sh_save_pos < SH_POS_FIRST_USER) sh_save_pos = SH_POSITIONS - 1;
                if (sh_save_pos >= SH_POSITIONS) sh_save_pos = SH_POS_FIRST_USER;
            } else {
                sh_load_pos = (sh_load_pos + step + SH_POSITIONS) % SH_POSITIONS;
            }
        }
        if (!(k & SH_A)) continue;
        if (id == SH_M_CONTINUE) break;
        if (id == SH_M_SAVE || id == SH_M_LOAD) {
            int loading = id == SH_M_LOAD, pos = loading ? sh_load_pos : sh_save_pos;
            if (loading && !sh_slot_used[pos]) continue;
            char label[24], msg[64];
            sh_slot_label(pos, label, sizeof label);
            int failed = loading ? sh_state_load(sh_pos_slot(pos))
                                 : sh_state_save(sh_pos_slot(pos), sh_bg, sh_bg_w, sh_bg_h);
            if (failed) snprintf(msg, sizeof msg, loading ? "Load failed" : "Save failed");
            else if (!loading) { snprintf(msg, sizeof msg, "Saved to slot %s", label); sh_load_pos = pos; }
            else if (pos < SH_POS_FIRST_USER) snprintf(msg, sizeof msg, "Loaded %s", label);
            else snprintf(msg, sizeof msg, "Loaded slot %s", label);
            if (!failed && pos >= SH_POS_FIRST_USER) sh_save_pos = sh_load_pos = pos;
            sh_toast(msg);
            if (failed) { sh_slots_scan(); continue; }
            break;
        }
        sh_wait_release();
        if (id == SH_M_DISC) { sh_run_frontend(sf3000_swap_cd); }
        if (id == SH_M_CHEATS) { sh_run_frontend(sf3000_open_cheats); }
        if (id == SH_M_OPTIONS) sh_menu_options();
        if (id == SH_M_RESET) sh_reset_game();
        if (id == SH_M_EXIT) sh_exit_game();
        sh_wait_release();
        sh_input_begin(&in);
    }
    sh_wait_release();
    sh_in_menu = 0;
    fb1_blank(1);
    last_fb_y_off = -1; last_fb_y_len = -1;
    set_panel_aspect();
}

/* ---- GameSwitcher and power-off -------------------------------------------
 * The GameSwitcher saves the Switcher slot, marks it to resume, refreshes the
 * GameSwitcher picture and tells FrogUI to open its GameSwitcher. Power-off
 * (R36SX: power button held a second; any console: the shutdown's SIGTERM)
 * saves the same way and leaves resume_on_boot.txt for the next boot. */
static void sh_switcher_suspend(void) {
    if (sh_state_save(SH_SLOT_SWITCHER, SCREEN, SCREEN_WIDTH, SCREEN_HEIGHT) == 0)
        sh_mark_resume(1);
    write_recent_screenshot();
    FILE *f = fopen(SH_SWITCHER_FLAG, "w");
    if (f) fclose(f);
    sync();
    exit(0);
}

#define SH_GPIO_BASE 0x18800000u
#define SH_GPIO_IN   (0x50 / 4)
#define SH_GPIO_DIR  (0x58 / 4)
#define SH_GPIO_BIT  (1u << 14)
#define SH_POWER_SAVE_MS   1000
#define SH_POWER_CANCEL_MS 10000
static volatile uint32_t *sh_power_gpio;
static volatile sig_atomic_t sh_power_term;
static int sh_power_saved;

static void sh_term_handler(int sig) { (void)sig; sh_power_term = 1; }

static void sh_power_init(void) {
    signal(SIGTERM, sh_term_handler);
    if (!sh_is_r36sx()) return;   /* pin 14 is verified only on the R36SX family */
    int fd = open("/dev/mem", O_RDONLY | O_SYNC);
    if (fd < 0) return;
    void *p = mmap(NULL, 4096, PROT_READ, MAP_SHARED, fd, SH_GPIO_BASE);
    close(fd);
    if (p == MAP_FAILED) return;
    volatile uint32_t *r = p;
    if ((r[SH_GPIO_DIR] & SH_GPIO_BIT) || !(r[SH_GPIO_IN] & SH_GPIO_BIT)) { munmap(p, 4096); return; }
    sh_power_gpio = r;
}

static void sh_power_suspend(void) {
    if (!sh_power_resume()) return;
    if (sh_state_save(SH_SLOT_SWITCHER, SCREEN, SCREEN_WIDTH, SCREEN_HEIGHT) != 0) return;
    sh_mark_resume(1);
    write_recent_screenshot();
    FILE *f = fopen(SH_POWER_RESUME, "w");
    if (f) {
        fprintf(f, "%s\n%s\n", SH_PCSX4ALL_BIN, g_rom_path);
        fflush(f); fsync(fileno(f)); fclose(f);
    }
    sync();
}

/* ShallotsOS sleep on a short press when FrogUI's Disable Sleep is on (the
 * firmware's own sleep is then off): backlight off and wait, as picoarch. */
static void sh_sleep(void) {
    int light = sh_backlight_raw();
    char v[16];
    long long limit = (fs_get("sleep_poweroff", v, sizeof v) ? atoi(v) : 300);
    if (limit < 5) limit = 5;
    if (limit > 3600) limit = 3600;
    limit *= 1000;
    sh_backlight_write(0);
    long long start = sh_ms(), down_at = -1;
    int powering_off = 0;
    for (;;) {
        usleep(50000);
        long long now = sh_ms();
        int down = !(sh_power_gpio[SH_GPIO_IN] & SH_GPIO_BIT);
        if (powering_off || sh_power_term) continue;
        if (down) {
            if (down_at < 0) down_at = now;
            if (now - down_at >= SH_POWER_SAVE_MS) {
                if (!sh_power_saved) sh_power_suspend();
                sh_power_saved = 1;
                powering_off = 1;
            }
        } else if (down_at >= 0) {
            break;
        }
        if (now - start >= limit) {
            if (!sh_power_saved) sh_power_suspend();
            sh_power_saved = 1;
            sync();
            if (fork() == 0) { execl("/bin/sh", "sh", SH_SHUTDOWN_SCRIPT, (char *)NULL); _exit(1); }
            powering_off = 1;
        }
    }
    if (light >= 0) sh_backlight_write(light);
}

static void sh_power_watch(void) {
    static long long down_since = -1, released_at = -1;
    if (sh_power_term) {
        if (!sh_power_saved) sh_power_suspend();
        sh_power_saved = 1;
        sync();
        exit(0);
    }
    if (!sh_power_gpio) return;
    long long now = sh_ms();
    if (!(sh_power_gpio[SH_GPIO_IN] & SH_GPIO_BIT)) {
        released_at = -1;
        if (down_since < 0) down_since = now;
        else if (!sh_power_saved && now - down_since >= SH_POWER_SAVE_MS) {
            sh_power_suspend();
            sh_power_saved = 1;
        }
        return;
    }
    if (down_since >= 0) {
        long long held = now - down_since;
        down_since = -1;
        if (held < SH_POWER_SAVE_MS && !sh_power_saved && sh_own_sleep()) { sh_sleep(); return; }
    }
    if (!sh_power_saved) return;
    if (released_at < 0) released_at = now;
    else if (now - released_at >= SH_POWER_CANCEL_MS) {
        /* Still running long after the button was let go: not a power-off. */
        sh_mark_resume(0);
        unlink(SH_POWER_RESUME);
        sync();
        sh_power_saved = 0;
        released_at = -1;
    }
}

/* First frame: resume the GameSwitcher / power-off state (once), else the
 * Auto slot when Auto-Save/Load is on. */
static void sh_first_frame(void) {
    char marker[PATH_MAX + 80];
    sh_marker_path(marker, sizeof marker);
    if (access(marker, F_OK) == 0) {
        unlink(marker);
        if (sh_state_load(SH_SLOT_SWITCHER) == 0) return;
    }
    if (sh_autosave() && sh_state_exists(SH_SLOT_AUTO) && sh_state_load(SH_SLOT_AUTO) == 0)
        sh_toast("Loaded Auto");
}

/* ---- In-game keys ---------------------------------------------------------- */
void pad_update(void) {
    static int first = 1;
    if (first) { first = 0; sh_first_frame(); }
    sh_power_watch();
    if (!cv_keys) return;
    uint32_t k = *cv_keys;
    const uint32_t SEL = 1u << CV_SEL, START = 1u << CV_START, FN = 1u << CV_FN;
    const int keys = sh_game_keys();
    const int both = (k & (SEL | START)) == (SEL | START);
    long long now = sh_ms();
    int open_menu = 0, open_switcher = 0;
    uint32_t hide = 0;   /* buttons kept from the game this frame */

    if (keys == 0) {
        /* FN: tap = menu, hold = GameSwitcher; with another button or a
         * volume/brightness change it is a shortcut and does neither. */
        static long long fn_down = -1, fn_probe = 0;
        static int fn_combo, fn_fired, fn_up, fn_sig;
        if (k & FN) {
            fn_up = 0;
            if (fn_down < 0) { fn_down = fn_probe = now; fn_combo = fn_fired = 0; fn_sig = sh_fn_combo_probe(); }
            if (k & 0xFFFFu) fn_combo = 1;
            if (!fn_combo && now - fn_probe >= SH_PROBE_MS) {
                fn_probe = now;
                if (sh_fn_combo_probe() != fn_sig) fn_combo = 1;
            }
            if (!fn_fired && !fn_combo && sh_switcher_on() && now - fn_down >= SH_HOLD_MS) {
                fn_fired = 1; open_switcher = 1;
            }
        } else if (fn_down >= 0 && ++fn_up >= SH_UP_POLLS) {
            if (!fn_combo && sh_fn_combo_probe() != fn_sig) fn_combo = 1;
            if (!fn_fired && !fn_combo) open_menu = 1;
            fn_down = -1;
        }
    } else if (keys == 2) {
        /* SELECT+START: tap = menu, hold = GameSwitcher. */
        static long long pair_down = -1;
        static int pair_fired, pair_up;
        if (both) {
            pair_up = 0;
            hide |= SEL | START;
            if (pair_down < 0) { pair_down = now; pair_fired = 0; }
            else if (!pair_fired && sh_switcher_on() && now - pair_down >= SH_HOLD_MS) {
                pair_fired = 1; open_switcher = 1;
            }
        } else if (pair_down >= 0) {
            hide |= SEL | START;
            if (++pair_up >= SH_UP_POLLS) { open_menu = !pair_fired; pair_down = -1; }
        }
    } else {
        /* Hold SELECT = menu, hold START = GameSwitcher (alone, no combo). */
        static uint32_t hold_bit;
        static long long hold_down;
        static int hold_used;
        uint32_t one = k & (SEL | START);
        if (one && one != (SEL | START) && !(k & 0xFFF6u & ~one)) {
            if (hold_bit != one) { hold_bit = one; hold_down = now; hold_used = 0; }
            else if (!hold_used && now - hold_down >= SH_HOLD_MS) {
                hold_used = 1;
                if (one == SEL) open_menu = 1;
                else if (sh_switcher_on()) open_switcher = 1;
            }
        } else {
            hold_bit = 0;
        }
    }

    /* SELECT+R2 save / SELECT+L2 load the current slot, once per press. */
    static int ss_armed = 1;
    int save_combo = (k & SEL) && (k & (1u << CV_R2)), load_combo = (k & SEL) && (k & (1u << CV_L2));
    if (ss_armed && (save_combo || load_combo)) {
        ss_armed = 0;
        int pos = sh_save_pos;
        char label[24], msg[64];
        sh_slot_label(pos, label, sizeof label);
        if (save_combo) {
            int ok = sh_state_save(sh_pos_slot(pos), SCREEN, SCREEN_WIDTH, SCREEN_HEIGHT) == 0;
            snprintf(msg, sizeof msg, ok ? "Saved to slot %s" : "Save failed", label);
        } else {
            int ok = sh_state_load(sh_pos_slot(pos)) == 0;
            snprintf(msg, sizeof msg, ok ? "Loaded slot %s" : "Load failed", label);
        }
        sh_toast(msg);
    } else if (!(k & SEL)) {
        ss_armed = 1;
    }
    if (!ss_armed) hide |= SEL | (1u << CV_R2) | (1u << CV_L2);

    if (open_switcher) sh_switcher_suspend();
    if (open_menu) { sh_menu(); k = cv_keys ? *cv_keys : 0; hide = k; }

    k &= ~hide;
    /* Build PSX pad word (0=pressed, 1=released) */
    uint16_t p = 0xFFFF;
    if (btn(k, CV_UP))    p &= ~PSX_UP;
    if (btn(k, CV_DOWN))  p &= ~PSX_DOWN;
    if (btn(k, CV_LEFT))  p &= ~PSX_LEFT;
    if (btn(k, CV_RIGHT)) p &= ~PSX_RIGHT;
    /* Face + shoulder buttons via the remappable bind table (default layout:
     * B=Cross, A=Circle, Y=Square, X=Triangle, L/R/L2/R2 1:1). */
    for (int i = 0; i < PB_COUNT; i++)
        if (btn(k, pb_bind[i])) p &= ~pb_mask[i];
    if (btn(k, CV_SEL))   p &= ~PSX_SELECT;
    if (btn(k, CV_START)) p &= ~PSX_START;
    pad1 = p;
}

/* --------------------------------------------------------------------------
 * Timing
 * -------------------------------------------------------------------------- */
unsigned get_ticks(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (unsigned)(tv.tv_sec * 1000000ULL + tv.tv_usec);
}

void wait_ticks(unsigned us) {
    usleep(us);
}

/* --------------------------------------------------------------------------
 * Config paths
 * -------------------------------------------------------------------------- */
static char homedir[PATH_MAX]    = "/mnt/sdcard/cubegm/cores/.pcsx4all";
char        sstatesdir[PATH_MAX] = "/mnt/sdcard/cubegm/cores/.pcsx4all/savestates";
char        cheatsdir[PATH_MAX]  = "/mnt/sdcard/cubegm/cores/.pcsx4all/cheats";

#define SHARED_BIOS_DIR          "/mnt/sdcard/cubegm/bios"
#define LEGACY_CORES_BIOS_DIR    "/mnt/sdcard/cubegm/cores/.pcsx4all/bios"
#define LEGACY_PCSX4ALL_DIR      "/mnt/sdcard/cubegm/.pcsx4all"
#define LEGACY_PCSX4ALL_BIOS_DIR "/mnt/sdcard/cubegm/.pcsx4all/bios"

struct ps1_controller player_controller[2] = {{0},{0}};

void Set_Controller_Mode(void) {
    switch (Config.Analog_Mode) {
        default:
            player_controller[0].id = 0x41;
            player_controller[0].pad_mode = 0;
            player_controller[0].pad_controllertype = 0;
            break;
        case 1:
            player_controller[0].id = 0x53;
            player_controller[0].pad_mode = 1;
            player_controller[0].pad_controllertype = 1;
            break;
        case 2:
            player_controller[0].id = 0x73;
            player_controller[0].pad_mode = 1;
            player_controller[0].pad_controllertype = 1;
            break;
    }
}

/* --------------------------------------------------------------------------
 * Config load/save (matches icube-written cfg format)
 * -------------------------------------------------------------------------- */
/* Per-game config path: homedir/config/<rom basename, no ext>.cfg */
static void config_game_path(char *out, size_t n) {
    const char *base = g_rom_path[0] ? strrchr(g_rom_path, '/') : NULL;
    base = base ? base + 1 : (g_rom_path[0] ? g_rom_path : "game");
    char name[300]; strncpy(name, base, sizeof name - 1); name[sizeof name - 1] = 0;
    char *dot = strrchr(name, '.'); if (dot) *dot = 0;
    snprintf(out, n, "%s/config/%s.cfg", homedir, name);
}
int config_have_game(void) {   /* does a per-game override exist? */
    char p[PATH_MAX]; config_game_path(p, sizeof p);
    FILE *f = fopen(p, "r"); if (f) { fclose(f); return 1; } return 0;
}

static int config_read(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) return -1;

#ifdef DSPR2_PROFILE
    fprintf(stderr, "PCSX4ALL_PROFILE_CONFIG_FILE: %s\n", path);
#endif

    char line[512];
    int linenum = 0;
    int version = -1;
    while (fgets(line, sizeof(line), f)) {
        char *arg = strchr(line, ' ');
        if (!arg) continue;
        *arg++ = '\0';
        /* strip trailing newline */
        char *nl = strchr(arg, '\n'); if (nl) *nl = '\0';
        linenum++;
        int v; sscanf(arg, "%d", &v);

        if (!strcmp(line,"CONFIG_VERSION")) {
            version = v;
            if (v < 0 || v > CONFIG_VERSION) break;
        }
        else if (!strncmp(line,"PsxBind",7)) { int idx = atoi(line+7); if (idx >= 0 && idx < PB_COUNT) pb_bind[idx] = v; }
        else if (!strcmp(line,"Xa"))              Config.Xa = v;
        else if (!strcmp(line,"Mdec"))            Config.Mdec = v;
        else if (!strcmp(line,"PsxAuto"))         Config.PsxAuto = v;
        else if (!strcmp(line,"Cdda"))            Config.Cdda = v;
        else if (!strcmp(line,"HLE"))             Config.HLE = v;
        else if (!strcmp(line,"RCntFix"))         Config.RCntFix = v;
        else if (!strcmp(line,"VSyncWA"))         Config.VSyncWA = v;
        else if (!strcmp(line,"Cpu"))             Config.Cpu = v;
        else if (!strcmp(line,"PsxType"))         Config.PsxType = v;
        else if (!strcmp(line,"SpuIrq"))          Config.SpuIrq = v;
        else if (!strcmp(line,"SyncAudio"))       Config.SyncAudio = v;
        else if (!strcmp(line,"SpuUpdateFreq"))   Config.SpuUpdateFreq = v;
        else if (!strcmp(line,"ForcedXAUpdates")) Config.ForcedXAUpdates = v;
        else if (!strcmp(line,"ShowFps"))         Config.ShowFps = v;
        else if (!strcmp(line,"FrameLimit"))      Config.FrameLimit = v;
        else if (!strcmp(line,"FrameSkip"))       Config.FrameSkip = v;
        else if (!strcmp(line,"AnalogArrow"))     Config.AnalogArrow = v;
        else if (!strcmp(line,"Analog_Mode"))     Config.Analog_Mode = v;
        else if (!strcmp(line,"LastDir"))         strncpy(Config.LastDir, arg, sizeof(Config.LastDir)-1);
        else if (!strcmp(line,"BiosDir"))         strncpy(Config.BiosDir, arg, sizeof(Config.BiosDir)-1);
        else if (!strcmp(line,"Bios"))            strncpy(Config.Bios,    arg, sizeof(Config.Bios)-1);
#ifdef PSXREC
        else if (!strcmp(line,"CycleMultiplier")) sscanf(arg, "%x", &cycle_multiplier);
#endif
#ifdef GPU_UNAI
        else if (!strcmp(line,"lighting"))      gpu_unai_config_ext.lighting = v;
        else if (!strcmp(line,"fast_lighting")) gpu_unai_config_ext.fast_lighting = v;
        else if (!strcmp(line,"blending"))      gpu_unai_config_ext.blending = v;
        else if (!strcmp(line,"dithering"))     gpu_unai_config_ext.dithering = v;
        else if (!strcmp(line,"clip_368"))      gpu_unai_config_ext.clip_368 = v;
        else if (!strcmp(line,"pixel_skip"))    gpu_unai_config_ext.pixel_skip = v;
        else if (!strcmp(line,"ilace_force"))   gpu_unai_config_ext.ilace_force = v;
#endif
#ifdef SPU_PCSXREARMED
        else if (!strcmp(line,"SpuUseInterpolation")) spu_config.iUseInterpolation = v;
        else if (!strcmp(line,"SpuUseReverb"))        spu_config.iUseReverb = v;
        else if (!strcmp(line,"SpuVolume"))           spu_config.iVolume = v;
#endif
        else if (!strcmp(line,"ScaleMode"))   scale_mode   = v;
        else if (!strcmp(line,"HiResFix"))    hires_fix    = v ? 1 : 0;
        else if (!strcmp(line,"ScaleFilter")) scale_filter = v;
        else if (!strcmp(line,"Gamma"))       gamma_percent = (v < 100 ? 100 : (v > 250 ? 250 : v));
    }
    fclose(f);
    return version;
}

static void config_write(const char *path);

/* v0 was shipped with 16 SPU events per emulated frame and the FPS overlay
 * enabled. On R36SX that produces dozens of tiny audio submissions for every
 * displayed frame. Upgrade only those old defaults; all user controls, BIOS,
 * video quality, and per-game settings remain intact. Once rewritten as v1,
 * an explicit user choice of either setting is respected. */
static int config_migrate_v0(void) {
    int changed = 0;
    if (Config.SpuUpdateFreq == SPU_UPDATE_FREQ_16) {
        Config.SpuUpdateFreq = SPU_UPDATE_FREQ_1;
        changed = 1;
    }
    if (Config.ShowFps == 1) {
        Config.ShowFps = 0;
        changed = 1;
    }
    return changed;
}

/* Load global config, then layer the per-game override on top (if any). */
void config_load(void) {
    char p[PATH_MAX];
    snprintf(p, sizeof p, "%s/pcsx4all.cfg", homedir);
    int version = config_read(p);
    if (version == 0) {
        config_migrate_v0();
        config_write(p);
    }
    config_game_path(p, sizeof p);
    version = config_read(p);   /* no-op if absent; overrides global */
    if (version == 0 && config_migrate_v0())
        config_write(p);
}

static void config_write(const char *path) {
    FILE *f = fopen(path, "w");
    if (!f) return;
    fprintf(f, "CONFIG_VERSION %d\n"
               "Xa %d\nMdec %d\nPsxAuto %d\nCdda %d\nHLE %d\n"
               "RCntFix %d\nVSyncWA %d\nCpu %d\nPsxType %d\n"
               "SpuIrq %d\nSyncAudio %d\nSpuUpdateFreq %d\n"
               "ForcedXAUpdates %d\nShowFps %d\nFrameLimit %d\n"
               "FrameSkip %d\nAnalogArrow %d\nAnalog_Mode %d\n",
            CONFIG_VERSION,
            Config.Xa, Config.Mdec, Config.PsxAuto, Config.Cdda, Config.HLE,
            Config.RCntFix, Config.VSyncWA, Config.Cpu, Config.PsxType,
            Config.SpuIrq, Config.SyncAudio, Config.SpuUpdateFreq,
            Config.ForcedXAUpdates, Config.ShowFps, Config.FrameLimit,
            Config.FrameSkip, Config.AnalogArrow, Config.Analog_Mode);
#ifdef PSXREC
    fprintf(f, "CycleMultiplier %03x\n", cycle_multiplier);
#endif
#ifdef GPU_UNAI
    fprintf(f, "lighting %d\nfast_lighting %d\nblending %d\ndithering %d\nclip_368 %d\n"
               "pixel_skip %d\nilace_force %d\n",
            gpu_unai_config_ext.lighting, gpu_unai_config_ext.fast_lighting,
            gpu_unai_config_ext.blending, gpu_unai_config_ext.dithering,
            gpu_unai_config_ext.clip_368,
            gpu_unai_config_ext.pixel_skip, gpu_unai_config_ext.ilace_force);
#endif
#ifdef SPU_PCSXREARMED
    fprintf(f, "SpuUseInterpolation %d\nSpuUseReverb %d\nSpuVolume %d\n",
            spu_config.iUseInterpolation, spu_config.iUseReverb, spu_config.iVolume);
#endif
    fprintf(f, "ScaleMode %d\nScaleFilter %d\nGamma %d\n", scale_mode, scale_filter, gamma_percent);
    fprintf(f, "HiResFix %d\n", hires_fix);
    for (int i = 0; i < PB_COUNT; i++) fprintf(f, "PsxBind%d %d\n", i, pb_bind[i]);
    if (Config.LastDir[0]) fprintf(f, "LastDir %s\n", Config.LastDir);
    if (Config.BiosDir[0]) fprintf(f, "BiosDir %s\n", Config.BiosDir);
    if (Config.Bios[0])    fprintf(f, "Bios %s\n",    Config.Bios);
    fclose(f);
}

/* Explicit saves (no autosave-on-exit). Global = pcsx4all.cfg; per-game =
 * config/<rom>.cfg (loaded over the global at launch). */
void config_save(void) {   /* global */
    char p[PATH_MAX];
    snprintf(p, sizeof p, "%s/pcsx4all.cfg", homedir);
    config_write(p);
}
void config_save_game(void) {
    char dir[PATH_MAX], p[PATH_MAX];
    snprintf(dir, sizeof dir, "%s/config", homedir);
    mkdir(dir, 0755);
    config_game_path(p, sizeof p);
    config_write(p);
}
void config_delete_game(void) {
    char p[PATH_MAX]; config_game_path(p, sizeof p);
    unlink(p);
}

/* --------------------------------------------------------------------------
 * State save/load
 * -------------------------------------------------------------------------- */
int state_load(int slot) {
    char path[PATH_MAX+64];
    sh_state_path(path, sizeof path, slot);
    if (access(path, R_OK) == 0) return LoadState(path);
    return -1;
}

int state_save(int slot) {
    mkdir(sstatesdir, 0755);
    char path[PATH_MAX+64];
    sh_state_path(path, sizeof path, slot);
    return SaveState(path);
}


/* --------------------------------------------------------------------------
 * main()
 * -------------------------------------------------------------------------- */
void plog_pub(const char *msg) { (void)msg; }
static void plog(const char *msg) {
    /* Gated diagnostic: writes only if /mnt/sdcard/pcsx_log.txt exists. Zero
     * cost on a shipping card (no file). Used to capture the PSX resolution
     * transitions before a driver freeze. */
    static int en = -1;
    if (en < 0) { FILE *t = fopen("/mnt/sdcard/pcsx_log.txt", "r"); en = t ? 1 : 0; if (t) fclose(t); }
    if (!en) return;
    FILE *f = fopen("/mnt/sdcard/pcsx_log.txt", "a");
    if (f) { fprintf(f, "%s\n", msg); fflush(f); fsync(fileno(f)); fclose(f); }
}

/* FrogUI's recents switcher shows /mnt/sdcard/picoarch/<tag>/<base>.scr.bmp,
 * written by picoarch cores on exit. Standalone pcsx4all isn't a picoarch core,
 * so PS1 had no recents screenshot — dump the last frame here on exit. */
static char g_rom_path[1024];

static void write_recent_screenshot(void) {
    if (!g_rom_path[0]) return;
    char dir[1024]; strncpy(dir, g_rom_path, sizeof dir - 1); dir[sizeof dir - 1] = 0;
    char *sl = strrchr(dir, '/'); if (!sl) return;
    char base[256]; strncpy(base, sl + 1, sizeof base - 1); base[sizeof base - 1] = 0;
    *sl = 0;
    char *tagsl = strrchr(dir, '/'); const char *tag = tagsl ? tagsl + 1 : dir;
    char *dot = strrchr(base, '.'); if (dot) *dot = 0;

    char d[1100]; snprintf(d, sizeof d, "/mnt/sdcard/picoarch/%s", tag);
    mkdir("/mnt/sdcard/picoarch", 0777); mkdir(d, 0777);
    char out[1200]; snprintf(out, sizeof out, "%s/%s.scr.bmp", d, base);
    /* The PCSX settings pages clear SCREEN: leaving from the menu uses the
     * frame copied when it opened. */
    if (sh_in_menu && sh_bg_valid) sh_write_bmp(out, sh_bg, sh_bg_w, sh_bg_h);
    else if (SCREEN) sh_write_bmp(out, SCREEN, SCREEN_WIDTH, SCREEN_HEIGHT);
}

/* Auto-detect a PS1 BIOS so users don't have to name it exactly scph1001.bin.
 * Probe the shared cubegm/bios dir first, then every layout used by older
 * PCSX4ALL releases. Accept any 512 KB (0x80000) file, preferring an scph*
 * name; point Config at the first match. psxMemInit then loads it (HLE only if
 * nothing valid is found). */
static int probe_bios_dir(const char *dir) {
    DIR *d = opendir(dir);
    if (!d) return 0;
    char found[256] = ""; int found_scph = 0;
    struct dirent *e;
    while ((e = readdir(d))) {
        if (e->d_name[0] == '.') continue;
        char p[PATH_MAX]; struct stat st;
        if (snprintf(p, sizeof p, "%s/%s", dir, e->d_name) >= (int)sizeof p) continue;
        if (stat(p, &st) != 0 || !S_ISREG(st.st_mode) || st.st_size != 0x80000) continue;
        if (strncasecmp(e->d_name, "scph", 4) == 0) {      /* real PS1 bios name */
            strncpy(found, e->d_name, sizeof found - 1); found[sizeof found - 1] = 0;
            found_scph = 1; break;
        }
        if (!found[0]) { strncpy(found, e->d_name, sizeof found - 1); found[sizeof found - 1] = 0; }
    }
    closedir(d);
    if (!found[0]) return 0;
    strncpy(Config.BiosDir, dir,   sizeof(Config.BiosDir) - 1);
    strncpy(Config.Bios,    found, sizeof(Config.Bios) - 1);
    Config.HLE = 0;   /* real BIOS present -> use it, override any stale cfg */
    plog(found_scph ? "bios: auto-detected scph*" : "bios: auto-detected 512KB file");
    plog(Config.BiosDir); plog(Config.Bios);
    return 1;
}
/* Does `dir` contain a file named `name` (case-insensitive, like psxMemInit)? */
static int bios_present(const char *dir, const char *name) {
    if (!dir[0] || !name[0]) return 0;
    DIR *d = opendir(dir);
    if (!d) return 0;
    struct dirent *e; int hit = 0;
    while ((e = readdir(d))) if (!strcasecmp(e->d_name, name)) { hit = 1; break; }
    closedir(d);
    return hit;
}
static void probe_bios(void) {
    /* Respect a manual/existing selection: if the configured BIOS actually
     * resolves, leave BiosDir/Bios/HLE exactly as the user set them - no guess,
     * no HLE override. Only auto-detect when it's unset or the file is missing;
     * on a guess we switch HLE off so the found BIOS is actually used. */
    if (bios_present(Config.BiosDir, Config.Bios)) return;
    if (probe_bios_dir(SHARED_BIOS_DIR)) return;
    if (probe_bios_dir(homedir)) return;
    if (probe_bios_dir(LEGACY_CORES_BIOS_DIR)) return;
    if (probe_bios_dir(LEGACY_PCSX4ALL_DIR)) return;
    if (probe_bios_dir(LEGACY_PCSX4ALL_BIOS_DIR)) return;
    /* nothing found anywhere: leave config_load()'s values; psxMemInit HLEs. */
}

int main(int argc, char *argv[]) {
    plog("main: start");
    if (argc < 2) { plog("main: no ROM arg, exit"); return 1; }
    plog(argv[1]);
#ifdef DSPR2_PROFILE
    profile_dump_cpu_policy();
#endif
    /* Defaults */
    Config.Xa = 0; Config.Mdec = 0; Config.PsxAuto = 1;
    Config.Cdda = 0; Config.HLE = 0;  /* 0=BIOS (preferred); psxMemInit falls back to HLE if missing */
    Config.SpuUpdateFreq = SPU_UPDATE_FREQ_1; Config.ForcedXAUpdates = 1;
    Config.ShowFps = 0; Config.FrameLimit = 1; Config.FrameSkip = -1;
#ifdef PSXREC
    /* Stock 0x200 (2.0).  LOWERING it makes each instruction cost fewer cycles,
     * so more instructions are emulated per frame = MORE host work = slower.
     * RAISING it (e.g. 0x240/0x280) is the speed-hack (fewer emulated
     * instructions) but starves the game's CPU → in-game slowdown/glitches.
     * Keep accurate default; tune per-game via pcsx4all.cfg CycleMultiplier. */
    cycle_multiplier = 0x200;
#endif
    strncpy(Config.BiosDir, homedir, sizeof(Config.BiosDir) - 1);
    strncpy(Config.Bios, "scph1001.bin", sizeof(Config.Bios) - 1);
    /* BIOS auto-detect runs AFTER config_load() below, so it overrides a stale
     * cfg pointing at an empty path. */

#ifdef GPU_UNAI
    /* Hardware profiling in Tekken 3 showed full lighting and blending cost
     * no measurable frame time versus the degraded fast settings. Keep the
     * correct image and take the proven performance win from sane SPU pacing. */
    gpu_unai_config_ext.lighting     = 1;
    gpu_unai_config_ext.fast_lighting= 1;
    gpu_unai_config_ext.blending     = 1;
    gpu_unai_config_ext.dithering    = 0;
    gpu_unai_config_ext.clip_368     = 0;
    /* SF3000: keep GPU output full-quality by default — NO line skipping
     * (ilace_force=0) and NO pixel skipping (pixel_skip=0; risky with the HW
     * downscaler anyway).  Speed comes from the lossless levers (cycle_multiplier
     * + CPU governor).  Both remain per-game opt-in via pcsx4all.cfg. */
    gpu_unai_config_ext.pixel_skip   = 0;
    gpu_unai_config_ext.ilace_force  = 0;
#endif
#ifdef SPU_PCSXREARMED
    spu_config.iHaveConfiguration = 1;
    spu_config.iUseReverb         = 0;
    spu_config.iUseInterpolation  = 0;
    spu_config.iXAPitch           = 0;
    spu_config.iVolume            = 1024;
    spu_config.iUseThread         = 0;
    spu_config.iUseFixedUpdates   = 0;
    spu_config.iDisabled          = 0;
#endif

    /* ROM path first, so config_load() can find the per-game override. */
    if (argc >= 2) strncpy(g_rom_path, argv[1], sizeof(g_rom_path)-1);
    mkdir(homedir, 0755);
    config_load();   /* global config, then per-game override on top */

#ifdef DSPR2_PROFILE
    fprintf(stderr,
            "PCSX4ALL_PROFILE_CONFIG: cpu=%d frameskip=%d limit=%d cycle=%03x "
            "lighting=%d fast_lighting=%d blending=%d dithering=%d "
            "pixel_skip=%d ilace_force=%d spu_interp=%d spu_reverb=%d "
            "scale=%d hires_fix=%d gamma=%d\n",
            Config.Cpu, Config.FrameSkip, Config.FrameLimit,
#ifdef PSXREC
            cycle_multiplier,
#else
            0,
#endif
#ifdef GPU_UNAI
            gpu_unai_config_ext.lighting, gpu_unai_config_ext.fast_lighting,
            gpu_unai_config_ext.blending, gpu_unai_config_ext.dithering,
            gpu_unai_config_ext.pixel_skip, gpu_unai_config_ext.ilace_force,
#else
            0, 0, 0, 0, 0, 0,
#endif
#ifdef SPU_PCSXREARMED
            spu_config.iUseInterpolation, spu_config.iUseReverb,
#else
            0, 0,
#endif
            scale_mode, hires_fix, gamma_percent);
#endif

    /* Memory cards: the sf3000 port never set these, so Config.Mcd1/2 were
     * empty → BIOS reported "no card" → games couldn't save. Point them at
     * persistent files under homedir/memcards (LoadMcd auto-creates the .mcr
     * if the parent dir exists). */
    {
        static char mcddir[PATH_MAX];
        snprintf(mcddir, sizeof(mcddir), "%s/memcards", homedir);
        mkdir(mcddir, 0755);
        snprintf(Config.Mcd1, sizeof(Config.Mcd1), "%s/mcd001.mcr", mcddir);
        snprintf(Config.Mcd2, sizeof(Config.Mcd2), "%s/mcd002.mcr", mcddir);
    }

    /* BIOS auto-detect: if a real BIOS is present in cubegm/bios or any legacy
     * .pcsx4all location, point Config at it and force HLE off — so just
     * dropping the file in place works, even if a stale pcsx4all.cfg still
     * points HLE on or at an empty path. */
    probe_bios();

    /* ROM from command line arg 1 */
    if (argc >= 2) {
        SetIsoFile(argv[1]);
        strncpy(g_rom_path, argv[1], sizeof(g_rom_path)-1);
        strncpy(Config.LastDir, argv[1], sizeof(Config.LastDir)-1);
        char *sl = strrchr(Config.LastDir, '/');
        if (sl) *sl = '\0';
    }

    sh_argc = argc; sh_argv = argv;
    if (display_init() < 0) { plog("display_init FAILED"); return 1; }
    atexit(shutdown_display);
    atexit(write_recent_screenshot);   /* runs before shutdown_display (LIFO) → SCREEN valid */
    input_init();
    sh_power_init();

    mkdir(sstatesdir, 0755);
    mkdir(cheatsdir, 0755);

    if (psxInit() == -1) { plog("psxInit FAILED"); return 1; }
    reopen_speaker_amp();
    {
        extern R3000Acpu psxRec;
        plog(psxCpu == &psxRec ? "dynarec ACTIVE" : "INTERPRETER (slow!)");
#ifdef DSPR2_PROFILE
        fprintf(stderr, "PCSX4ALL_PROFILE_CPU: %s\n",
                psxCpu == &psxRec ? "dynarec" : "INTERPRETER");
#endif
    }

    /* This port hand-rolls plugin init and never calls LoadPlugins(), so do the
     * memcard load here (creates the .mcr files if absent). Without this the
     * cards are never inserted → games can't save. */
    LoadMcd(MCD1, Config.Mcd1);
    LoadMcd(MCD2, Config.Mcd2);

    if (CDR_init() < 0) { plog("CDR_init FAILED"); return 1; }
    if (GPU_init() < 0) { plog("GPU_init FAILED"); return 1; }
    if (SPU_init() < 0) { plog("SPU_init FAILED"); return 1; }
    if (CDR_open() < 0) { plog("CDR_open FAILED"); return 1; }

    pl_init();
    psxReset();

    if (argc >= 2) {
        if (CheckCdrom() == -1) {
            SetIsoFile(NULL);
        } else if (LoadCdrom() == -1) {
            SetIsoFile(NULL);
        } else {
            cheat_load();
        }
    }

    CheckforCDROMid_applyhacks();

    psxCpu->Execute();

    /* No autosave on exit - config is saved only via the menu's Save Config. */
    return 0;
}
