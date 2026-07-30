/* SPU output via HiChip driver.so (SF3000).
 * Same pattern picoarch uses: dlopen + sound_driver_init/playframe/deinit.
 * Driver runs at 48000 Hz; PSX SPU outputs 44100 Hz; we resample linearly. */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <dlfcn.h>
#include "out.h"
#include "tf_driver.h"
#include "dspr2_prof.h"

#define IN_RATE     44100
#define OUT_RATE    48000

static void *g_handle = NULL;
static int  (*p_init)(void *dev, int rate, int ch) = NULL;
static int  (*p_playframe)(const void *buf, int frames) = NULL;
static void (*p_deinit)(void) = NULL;
static int  g_active = 0;

/* Resampler state — fractional phase carried across feed() calls.
 * 16.16 fixed point. step = (IN_RATE << 16) / OUT_RATE. */
static uint32_t g_phase = 0;
static int16_t  g_last_l = 0;
static int16_t  g_last_r = 0;

/* Output buffer (resampled). 4096 stereo frames = 16KB. */
#define OUT_BUF_FRAMES 4096
static int16_t g_out_buf[OUT_BUF_FRAMES * 2];

/* Global output gain (8.8 fixed, 256 = 1.0), shared with picoarch/FrogUI via
 * cubegm/sndgain.txt — there's no system mixer on this hardware. */
static int g_gain_q8 = 256;
static void load_snd_gain(void) {
    g_gain_q8 = 256;
    FILE *f = fopen("/mnt/sdcard/cubegm/sndgain.txt", "r");
    if (!f) return;
    int pct = 100;
    if (fscanf(f, "%d", &pct) == 1) {
        if (pct < 0) pct = 0; if (pct > 100) pct = 100;
        g_gain_q8 = pct * 256 / 100;
    }
    fclose(f);
}

static int sf3000_init(void) {
    if (g_active) return 0;
    load_snd_gain();
    if (!g_handle) {
        g_handle = dlopen(tf_driver_path(), RTLD_LAZY);
        if (!g_handle) {
            fprintf(stderr, "out_sf3000: dlopen failed: %s\n", dlerror());
            return -1;
        }
        p_init      = dlsym(g_handle, "sound_driver_init");
        p_playframe = dlsym(g_handle, "sound_driver_playframe");
        p_deinit    = dlsym(g_handle, "sound_driver_deinit");
        if (!p_init || !p_playframe) {
            fprintf(stderr, "out_sf3000: missing audio symbols\n");
            dlclose(g_handle); g_handle = NULL;
            return -1;
        }
    }
    if (p_init(NULL, OUT_RATE, 2) < 0) {
        fprintf(stderr, "out_sf3000: sound_driver_init failed\n");
        return -1;
    }
    g_phase  = 0;
    g_last_l = 0;
    g_last_r = 0;
    g_active = 1;
    fprintf(stderr, "out_sf3000: audio active at %d Hz stereo (resample from %d)\n",
            OUT_RATE, IN_RATE);
    return 0;
}

static void sf3000_finish(void) {
    if (!g_active) return;
    if (p_deinit) p_deinit();
    g_active = 0;
}

static int sf3000_busy(void) {
    return 0;
}

/* Linear-interp resample 44100→48000 stereo S16.
 * Output frames per input frame ≈ 48/44.1 = 1.088. Phase is 16.16 fixed.
 * step = (IN_RATE << 16) / OUT_RATE ≈ 60211 (0xEB33). */
static void sf3000_feed(void *buf, int bytes) {
    if (!g_active || !p_playframe || !buf || bytes <= 0) return;
    PROF_AUDIO_BEGIN();

    const int16_t *src = (const int16_t *)buf;
    int in_frames = bytes / 4;
    if (in_frames <= 0) {
        PROF_AUDIO_END();
        return;
    }

    const uint32_t step = (uint32_t)(((uint64_t)IN_RATE << 16) / OUT_RATE);
    uint32_t phase = g_phase;
    int16_t last_l = g_last_l, last_r = g_last_r;
    int in_idx = 0;
    int out_idx = 0;

    /* Produce output frames while we have enough input. phase tracks the
     * sub-sample position in the input stream. */
    while (in_idx < in_frames && out_idx < OUT_BUF_FRAMES) {
        /* phase < 0x10000 means we interpolate between last and src[0]. */
        if (phase < 0x10000) {
            uint32_t frac = phase;
            int32_t l = last_l + (((int32_t)(src[in_idx*2  ] - last_l) * (int32_t)frac) >> 16);
            int32_t r = last_r + (((int32_t)(src[in_idx*2+1] - last_r) * (int32_t)frac) >> 16);
            g_out_buf[out_idx*2  ] = (int16_t)l;
            g_out_buf[out_idx*2+1] = (int16_t)r;
            out_idx++;
            phase += step;
        } else {
            /* Advance source by one sample, decrement phase by 1.0 */
            last_l = src[in_idx*2  ];
            last_r = src[in_idx*2+1];
            in_idx++;
            phase -= 0x10000;
        }
    }

    /* Drain remaining input into last/phase even if output buf full
     * (caller will call us again next chunk). */
    while (in_idx < in_frames && phase >= 0x10000) {
        last_l = src[in_idx*2  ];
        last_r = src[in_idx*2+1];
        in_idx++;
        phase -= 0x10000;
    }

    g_phase  = phase;
    g_last_l = last_l;
    g_last_r = last_r;

    if (g_gain_q8 < 256) {
        for (int i = 0; i < out_idx * 2; i++)
            g_out_buf[i] = (int16_t)(((int)g_out_buf[i] * g_gain_q8) >> 8);
    }
    if (out_idx > 0) p_playframe(g_out_buf, out_idx);
    PROF_AUDIO_END();
}

void out_register_sf3000(struct out_driver *drv) {
    drv->name   = "sf3000";
    drv->init   = sf3000_init;
    drv->finish = sf3000_finish;
    drv->busy   = sf3000_busy;
    drv->feed   = sf3000_feed;
}
