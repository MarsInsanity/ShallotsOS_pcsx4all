/*
 * pcsx4all SF3000 port
 * Display: /dev/dis ioctl + /dev/fb0 direct mmap (same as picoarch)
 * Input:   cubevol shared memory at /tmp/joy_key
 * Exit:    SELECT+START → exit(0) → icube relaunches FrogUI
 * Menu:    SELECT+L1    → GameMenu()
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
#include <linux/fb.h>
#include <time.h>
#include <math.h>
#include <dirent.h>

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
static char g_rom_path[1024];   /* current ROM path (defined below) */

/* Reusable themed menu list: header + scrolling items + pill selection + hint,
 * matching picoarch/FrogUI. Handles lists longer than the screen (scrolls to keep
 * the selection visible). Falls back to the bitmap font if the TTF didn't load. */
int  sf3000_menu_begin(void);
void sf3000_menu_present(void);
/* Brief centered confirmation, e.g. after saving config. */
static void sf_menu_toast(const char *msg) {
    if (!sf3000_menu_begin()) { video_clear(); port_printf(2, 20, msg); video_flip(); usleep(600000); return; }
    int w = sf_text_w(msg), x = (sf_tw - w) / 2, y = sf_th / 2 - 14;
    sf_fill_round(x - 16, y - 8, w + 32, 40, 20, sf_col_sel);
    sf_text(x, y, msg, sf_col_seltext);
    sf3000_menu_present();
    usleep(650000);
}
static void sf_menu_draw(const char *title, const char *const *items, int n,
                         int sel, const char *hint) {
    if (!sf3000_menu_begin()) {   /* not ready: bitmap fallback on SCREEN */
        video_clear();
        port_printf(2, 0, title);
        for (int i = 0; i < n; i++) {
            char buf[80]; snprintf(buf, sizeof buf, "%s%s", (i==sel)?"> ":"  ", items[i]);
            port_printf(2, (i + 2) * 10, buf);
        }
        if (hint) port_printf(2, (n + 3) * 10, hint);
        video_flip();
        return;
    }
    /* themed, panel resolution (sf_tw x sf_th) */
    int fh = sf_font_h(), rowh = fh + 8, pad = 14;
    int header_h = rowh + 6;
    int hint_h = hint ? (fh + 8) : 0;
    int avail = sf_th - header_h - hint_h - pad;
    int vis = avail / rowh; if (vis < 1) vis = 1; if (vis > n) vis = n;
    int off = 0;
    if (sel >= vis) off = sel - vis + 1;
    if (off > n - vis) off = n - vis; if (off < 0) off = 0;

    { char h[96]; snprintf(h, sizeof h, ">>> %s", title); sf_text(pad, pad, h, sf_col_sel); }
    sf_draw_battery();
    int y0 = pad + header_h;
    for (int i = 0; i < vis; i++) {
        int idx = off + i, ry = y0 + i * rowh;
        if (idx == sel) {
            int w = sf_text_w(items[idx]) + 24;
            if (w > sf_tw - (pad - 6) * 2) w = sf_tw - (pad - 6) * 2;
            sf_fill_round(pad - 6, ry - 4, w, fh + 6, (fh + 6) / 2, sf_col_sel);
            sf_text(pad + 6, ry, items[idx], sf_col_seltext);
        } else {
            sf_text(pad + 6, ry, items[idx], sf_col_text);
        }
    }
    if (off > 0)        sf_text(sf_tw - pad - 12, y0, "^", sf_col_text);
    if (off + vis < n)  sf_text(sf_tw - pad - 12, y0 + (vis-1)*rowh, "v", sf_col_text);
    if (hint) sf_text(pad, y0 + vis * rowh + 2, hint, sf_col_text);
    sf3000_menu_present();
}

/* Controls remap screen: pick a PSX button, press a physical button to bind it.
 * Saved with the rest of the config on "Exit to FrogUI". */
static void sf3000_controls_menu(void) {
    int sel = 0;
    uint32_t prev = cv_keys ? *cv_keys : 0;
    while (1) {
        char rows[PB_COUNT][64]; const char *items[PB_COUNT];
        for (int i = 0; i < PB_COUNT; i++) {
            snprintf(rows[i], sizeof rows[i], "%-11s = %s button",
                     pb_name[i], cv_bit_name(pb_bind[i]));
            items[i] = rows[i];
        }
        sf_menu_draw("CONTROLS (remap)", items, PB_COUNT, sel, "A=rebind  B=back");
        usleep(16000);
        if (!cv_keys) continue;
        uint32_t k = *cv_keys;
        int up = btn(k, CV_UP)   && !btn(prev, CV_UP);
        int dn = btn(k, CV_DOWN) && !btn(prev, CV_DOWN);
        int a  = btn(k, CV_A)    && !btn(prev, CV_A);
        int b  = btn(k, CV_B)    && !btn(prev, CV_B);
        prev = k;
        if (up && sel > 0) sel--;
        if (dn && sel < PB_COUNT - 1) sel++;
        if (b) return;
        if (a) {
            char msg[64]; snprintf(msg, sizeof msg, "Press a button for %s", pb_name[sel]);
            if (sf3000_menu_begin()) {
                sf_text(14, 14, ">>> CONTROLS (remap)", sf_col_sel);
                sf_text(14, 60, msg, sf_col_text);
                sf3000_menu_present();
            } else {
                video_clear();
                port_printf(2, 0, "=== CONTROLS (remap) ==="); port_printf(2, 30, msg);
                video_flip();
            }
            while (cv_keys && *cv_keys) usleep(10000);      /* wait all released */
            int done = 0;
            while (!done && cv_keys) {
                uint32_t kk = *cv_keys;
                for (int j = 0; j < CV_BTNS_N; j++)
                    if (btn(kk, cv_btns[j].bit)) { pb_bind[sel] = cv_btns[j].bit; done = 1; break; }
                usleep(10000);
            }
            while (cv_keys && *cv_keys) usleep(10000);       /* wait release */
            prev = cv_keys ? *cv_keys : 0;
        }
    }
}

static void sf3000_menu(void) {
    int sel = 0;
    uint32_t prev = cv_keys ? *cv_keys : 0;
    fb1_blank(0);   /* show battery while in pause menu */

    while (1) {
        char scale_label[48];
        snprintf(scale_label,  sizeof(scale_label),  "Scale:  %s",
                 scale_mode == 1 ? "Fullscreen" : "Aspect-Correct");
        char gamma_label[48];
        if (gamma_percent <= 100)
            snprintf(gamma_label, sizeof(gamma_label), "Gamma:  Off");
        else
            snprintf(gamma_label, sizeof(gamma_label), "Gamma:  %d.%02d",
                     gamma_percent / 100, gamma_percent % 100);

        char pixskip_label[48], ilace_label[48], hires_label[48];
        snprintf(pixskip_label, sizeof(pixskip_label), "Pixel Skip (speed): %s",
                 gpu_unai_config_ext.pixel_skip ? "On" : "Off");
        snprintf(ilace_label, sizeof(ilace_label), "Interlace (speed):  %s",
                 gpu_unai_config_ext.ilace_force ? "On" : "Off");
        snprintf(hires_label, sizeof(hires_label), "Hi-Res Fix:         %s",
                 hires_fix ? "On" : "Off");

        extern int config_have_game(void);
        char gamecfg_label[48];
        snprintf(gamecfg_label, sizeof gamecfg_label, "Save Config: This Game%s",
                 config_have_game() ? " *" : "");
        const char *items[] = {
            "Resume",
            "Save State (slot 1)",
            "Load State (slot 1)",
            scale_label,
            gamma_label,
            pixskip_label,
            ilace_label,
            hires_label,
            "PCSX Settings",
            "Controls (remap)",
            "Save Config: Global",   /* 10 */
            gamecfg_label,           /* 11 */
            "Exit to FrogUI",        /* 12 */
            NULL
        };
        int n = 0; while (items[n]) n++;

        sf_menu_draw("PCSX4ALL MENU", items, n, sel, "UP/DOWN  A=select  B=back");

        usleep(16000);
        if (!cv_keys) continue;
        uint32_t k = *cv_keys;

        int up  = btn(k, CV_UP)    && !btn(prev, CV_UP);
        int dn  = btn(k, CV_DOWN)  && !btn(prev, CV_DOWN);
        int lt  = btn(k, CV_LEFT)  && !btn(prev, CV_LEFT);
        int rt  = btn(k, CV_RIGHT) && !btn(prev, CV_RIGHT);
        int a   = btn(k, CV_A)     && !btn(prev, CV_A);
        int b   = btn(k, CV_B)     && !btn(prev, CV_B);
        prev = k;

        if (up && sel > 0)   sel--;
        if (dn && sel < n-1) sel++;
        if (b) { fb1_blank(1); return; }
        /* LEFT/RIGHT adjusts scale (sel 3) and gamma (sel 4) */
        if (lt || rt) {
            if (sel == 3) {
                scale_mode = (scale_mode + 1) % 2;
                last_fb_y_off = -1; last_fb_y_len = -1; last_h_blend = -1;
                if (scale_mode == 1) {
                    hwdisp_set_target_aspect(0, 0); hwdisp_set_filter(0);
                } else {
                    hwdisp_set_target_aspect(16, 9); hwdisp_set_filter(1);
                }
            } else if (sel == 4) {
                gamma_percent += rt ? 10 : -10;
                if (gamma_percent < 100) gamma_percent = 100;
                if (gamma_percent > 250) gamma_percent = 250;
            } else if (sel == 5) {
                gpu_unai_config_ext.pixel_skip = !gpu_unai_config_ext.pixel_skip;
            } else if (sel == 6) {
                gpu_unai_config_ext.ilace_force = !gpu_unai_config_ext.ilace_force;
            } else if (sel == 7) {
                hires_fix = !hires_fix;
            }
        }
        if (a) {
            switch (sel) {
                case 0: fb1_blank(1); return;
                case 1: state_save(1); usleep(300000); fb1_blank(1); return;
                case 2: state_load(1); usleep(300000); fb1_blank(1); return;
                case 3:
                    scale_mode = (scale_mode + 1) % 2;
                    last_fb_y_off = -1; last_fb_y_len = -1; last_h_blend = -1;
                    hwdisp_set_target_aspect(scale_mode == 1 ? 0 : 16, scale_mode == 1 ? 0 : 9);
                    break;
                case 4: /* gamma: A toggles off/default */
                    gamma_percent = (gamma_percent > 100) ? 100 : 130;
                    break;
                case 5: /* pixel skip: A toggles */
                    gpu_unai_config_ext.pixel_skip = !gpu_unai_config_ext.pixel_skip;
                    break;
                case 6: /* interlace: A toggles */
                    gpu_unai_config_ext.ilace_force = !gpu_unai_config_ext.ilace_force;
                    break;
                case 7: /* hi-res fix: A toggles */
                    hires_fix = !hires_fix;
                    break;
                case 8: {
                    extern int GameMenu(void);
                    extern void sdl_poll_reset(void);
                    while (cv_keys && btn(*cv_keys, CV_A)) usleep(10000);
                    sdl_poll_reset();
                    GameMenu();
                    prev = cv_keys ? *cv_keys : 0;
                    last_fb_y_off = -1; last_fb_y_len = -1;
                    break;
                }
                case 9: {  /* Controls (remap) */
                    while (cv_keys && btn(*cv_keys, CV_A)) usleep(10000);
                    sf3000_controls_menu();
                    prev = cv_keys ? *cv_keys : 0;   /* NOT auto-saved: use Save Config */
                    break;
                }
                case 10: {  /* Save Config (global) */
                    void config_save(void); config_save();
                    sf_menu_toast("Saved global config");
                    prev = cv_keys ? *cv_keys : 0;
                    break;
                }
                case 11: {  /* Save Config (this game) */
                    void config_save_game(void); config_save_game();
                    sf_menu_toast("Saved game config");
                    prev = cv_keys ? *cv_keys : 0;
                    break;
                }
                case 12: exit(0);   /* Exit: no autosave */
            }
        }
    }
}

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

void pad_update(void) {
    if (!cv_keys) return;
    uint32_t k = *cv_keys;

    /* SELECT+START → exit to FrogUI (no autosave; use Save Config in the menu) */
    if (btn(k, CV_SEL) && btn(k, CV_START)) {
        exit(0);
    }

    /* SELECT+L → in-game menu (wait for release so SELECT doesn't trigger _KEY_BACK) */
    static int menu_was = 0;
    int menu_now = btn(k, CV_SEL) && btn(k, CV_L);
    if (menu_now && !menu_was) {
        while (cv_keys && (btn(*cv_keys, CV_SEL) || btn(*cv_keys, CV_L)))
            usleep(10000);
        sf3000_menu();
    }
    menu_was = menu_now;

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

uint16_t pad_read(int num) { return (num == 0 ? pad1 : 0xFFFF); }

/* --------------------------------------------------------------------------
 * Video API
 * -------------------------------------------------------------------------- */
void video_flip(void) {
    if (Config.ShowFps) port_printf(2, 2, pl_data.stats_msg);
    gamma_apply(SCREEN, SCREEN_WIDTH * SCREEN_HEIGHT);
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

#define SF_FONT_PX 26.0f
static stbtt_fontinfo sf_font;
static unsigned char *sf_font_buf = NULL;
static float sf_font_scale = 0;
static int   sf_font_asc = 0;           /* scaled ascent (baseline offset) */
static int   sf_font_ready = 0;
/* theme colours (RGB565); defaults ~ white text, accent pill, dark sel-text */
static uint16_t sf_col_text = 0xFFFF, sf_col_sel = 0x2FE6, sf_col_seltext = 0x0000;

/* Menus render at PANEL resolution (like TreeFrogUI) for crisp text, not at the
 * PS1 game res. Drawing targets sf_tgt/sf_tw/sf_th (default = SCREEN); the menu
 * path points them at a panel-sized buffer and presents it 1:1. */
static uint16_t *sf_tgt = NULL; static int sf_tw = 0, sf_th = 0;
static uint16_t *sf_menu_fb = NULL; static int sf_pw = 0, sf_ph = 0;
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
static void sf_load_skin(void) {
    FILE *f = fopen("/mnt/sdcard/cubegm/skin/skin.txt", "r");
    if (!f) return;
    char line[128];
    while (fgets(line, sizeof line, f)) {
        unsigned v;
        if (sscanf(line, "text_color=0x%x", &v) == 1)      sf_col_text = sf_888_to_565(v);
        else if (sscanf(line, "selection_color=0x%x", &v) == 1) sf_col_sel = sf_888_to_565(v);
        else if (sscanf(line, "sel_text_color=0x%x", &v) == 1)  sf_col_seltext = sf_888_to_565(v);
    }
    fclose(f);
}
static void sf_font_init(void) {
    if (sf_font_ready) return;
    sf_font_ready = -1;   /* tried */
    const char *paths[] = { "/mnt/sdcard/frogui/fonts/BPreplayBold.otf",
                            "/mnt/sdcard/frogui/fonts/GamePocket-Regular-ZeroKern.ttf", NULL };
    for (int i = 0; paths[i]; i++) {
        FILE *f = fopen(paths[i], "rb");
        if (!f) continue;
        fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
        if (n <= 0) { fclose(f); continue; }
        sf_font_buf = (unsigned char*)malloc(n);
        if (!sf_font_buf) { fclose(f); return; }
        if (fread(sf_font_buf, 1, n, f) != (size_t)n) { fclose(f); free(sf_font_buf); sf_font_buf = NULL; continue; }
        fclose(f);
        if (!stbtt_InitFont(&sf_font, sf_font_buf, stbtt_GetFontOffsetForIndex(sf_font_buf, 0))) {
            free(sf_font_buf); sf_font_buf = NULL; continue;
        }
        sf_font_scale = stbtt_ScaleForPixelHeight(&sf_font, SF_FONT_PX);
        int asc, desc, gap; stbtt_GetFontVMetrics(&sf_font, &asc, &desc, &gap);
        sf_font_asc = (int)(asc * sf_font_scale + 0.5f);
        sf_font_ready = 1;
        sf_load_skin();
        return;
    }
}
static int sf_text_w(const char *s) {
    if (sf_font_ready != 1) return (int)strlen(s) * 8;
    int w = 0, prev = 0;
    for (; *s; s++) {
        int gi = stbtt_FindGlyphIndex(&sf_font, (unsigned char)*s);
        int adv, lsb; stbtt_GetGlyphHMetrics(&sf_font, gi, &adv, &lsb);
        if (prev) w += (int)(stbtt_GetGlyphKernAdvance(&sf_font, prev, gi) * sf_font_scale);
        w += (int)(adv * sf_font_scale); prev = gi;
    }
    return w;
}
static int sf_font_h(void) { return sf_font_ready == 1 ? (int)(SF_FONT_PX) : 8; }

/* Draw UTF-8/ASCII text at (x, y=top) in RGB565 colour, alpha-blended. */
static void sf_text(int x, int y, const char *s, uint16_t color) {
    if (sf_font_ready != 1) { port_printf(x, y, s); return; }
    int pen = x, prev = 0;
    int cr = (color >> 11) & 0x1f, cg = (color >> 5) & 0x3f, cb = color & 0x1f;
    for (; *s; s++) {
        int gi = stbtt_FindGlyphIndex(&sf_font, (unsigned char)*s);
        if (prev) pen += (int)(stbtt_GetGlyphKernAdvance(&sf_font, prev, gi) * sf_font_scale);
        int x0,y0,x1,y1;
        stbtt_GetGlyphBitmapBox(&sf_font, gi, sf_font_scale, sf_font_scale, &x0,&y0,&x1,&y1);
        int gw = x1-x0, gh = y1-y0;
        if (gw > 0 && gh > 0 && gw < 64 && gh < 64) {
            unsigned char bmp[64*64];
            stbtt_MakeGlyphBitmap(&sf_font, bmp, gw, gh, gw, sf_font_scale, sf_font_scale, gi);
            for (int j = 0; j < gh; j++) {
                int py = y + sf_font_asc + y0 + j;
                if (py < 0 || py >= sf_th) continue;
                uint16_t *row = sf_tgt + py * sf_tw;
                for (int i = 0; i < gw; i++) {
                    int a = bmp[j*gw+i]; if (!a) continue;
                    int px = pen + x0 + i; if (px < 0 || px >= sf_tw) continue;
                    uint16_t d = row[px];
                    int dr=(d>>11)&0x1f, dg=(d>>5)&0x3f, db=d&0x1f, ia=255-a;
                    int rr=(cr*a+dr*ia)/255, rg=(cg*a+dg*ia)/255, rb=(cb*a+db*ia)/255;
                    row[px] = (uint16_t)((rr<<11)|(rg<<5)|rb);
                }
            }
        }
        int adv, lsb; stbtt_GetGlyphHMetrics(&sf_font, gi, &adv, &lsb);
        pen += (int)(adv * sf_font_scale); prev = gi;
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
            static const int rx[] = {64,153,224,255}, py[] = {0,50,80,100};
            if (a1 <= rx[0]) cached = 0;
            else { cached = 100; for (int i = 1; i < 4; i++) if (a1 <= rx[i]) {
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
    hwdisp_present(sf_menu_fb, sf_pw, sf_ph, sf_pw * 2);
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
            int px = x + i; if (px < 0 || px >= SCREEN_WIDTH) continue;
            row[px] = c;
        }
    }
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
    snprintf(path, sizeof(path), "%s/%s.%d.sav", sstatesdir, CdromId, slot);
    if (access(path, R_OK) == 0) return LoadState(path);
    return -1;
}

int state_save(int slot) {
    mkdir(sstatesdir, 0755);
    char path[PATH_MAX+64];
    snprintf(path, sizeof(path), "%s/%s.%d.sav", sstatesdir, CdromId, slot);
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
    if (!SCREEN || !g_rom_path[0]) return;
    char dir[1024]; strncpy(dir, g_rom_path, sizeof dir - 1); dir[sizeof dir - 1] = 0;
    char *sl = strrchr(dir, '/'); if (!sl) return;
    char base[256]; strncpy(base, sl + 1, sizeof base - 1); base[sizeof base - 1] = 0;
    *sl = 0;
    char *tagsl = strrchr(dir, '/'); const char *tag = tagsl ? tagsl + 1 : dir;
    char *dot = strrchr(base, '.'); if (dot) *dot = 0;

    char d[1100]; snprintf(d, sizeof d, "/mnt/sdcard/picoarch/%s", tag);
    mkdir("/mnt/sdcard/picoarch", 0777); mkdir(d, 0777);
    char out[1200]; snprintf(out, sizeof out, "%s/%s.scr.bmp", d, base);
    FILE *f = fopen(out, "wb"); if (!f) return;

    int w = SCREEN_WIDTH, h = SCREEN_HEIGHT;
    int row = (w * 3 + 3) & ~3;                  /* rows 4-byte aligned */
    int imgsz = row * h, total = 54 + imgsz;
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
    if (!line) { fclose(f); return; }
    for (int y = h - 1; y >= 0; y--) {           /* BMP is bottom-up */
        const unsigned short *s = SCREEN + (size_t)y * w;
        int o = 0;
        for (int x = 0; x < w; x++) {
            unsigned short px = s[x];
            line[o++] = (px & 0x1F) << 3;            /* B */
            line[o++] = ((px >> 5)  & 0x3F) << 2;    /* G */
            line[o++] = ((px >> 11) & 0x1F) << 3;    /* R */
        }
        while (o < row) line[o++] = 0;
        fwrite(line, 1, row, f);
    }
    free(line);
    fclose(f);
}

/* Auto-detect a PS1 BIOS so users don't have to name it exactly scph1001.bin.
 * Probe the shared cubegm/bios dir first (what the docs recommend), then the
 * legacy .pcsx4all dir. Accept any 512 KB (0x80000) file, preferring an scph*
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
    if (probe_bios_dir("/mnt/sdcard/cubegm/bios")) return;
    if (probe_bios_dir(homedir)) return;
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

    /* BIOS auto-detect: if a real BIOS is present in cubegm/bios (or the legacy
     * .pcsx4all dir), point Config at it and force HLE off — so just dropping the
     * file in place "just works", no menu dance, even if a stale pcsx4all.cfg
     * (loaded above) still points HLE on or at an empty path. If none is found we
     * leave config_load()'s values and psxMemInit() HLE-falls-back. */
    probe_bios();

    /* ROM from command line arg 1 */
    if (argc >= 2) {
        SetIsoFile(argv[1]);
        strncpy(g_rom_path, argv[1], sizeof(g_rom_path)-1);
        strncpy(Config.LastDir, argv[1], sizeof(Config.LastDir)-1);
        char *sl = strrchr(Config.LastDir, '/');
        if (sl) *sl = '\0';
    }

    if (display_init() < 0) { plog("display_init FAILED"); return 1; }
    atexit(shutdown_display);
    atexit(write_recent_screenshot);   /* runs before shutdown_display (LIFO) → SCREEN valid */
    input_init();

    mkdir(sstatesdir, 0755);
    mkdir(cheatsdir, 0755);

    if (psxInit() == -1) { plog("psxInit FAILED"); return 1; }
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
