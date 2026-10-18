/* SF3000 PCM output. The worker owns driver init/write/deinit on one thread. */
#include <dlfcn.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

#include "out.h"
#include "tf_audio_queue.h"
#include "tf_driver.h"
#include "dspr2_prof.h"

static void *g_handle;
static int (*p_init)(void *dev, int rate, int ch);
static int (*p_playframe)(const void *buf, int frames);
static void (*p_deinit)(void);
static tf_audio_queue g_queue;
static pthread_t g_thread;
static int g_active, g_running, g_ready;
static unsigned g_output_chunks;
static int g_gain_q8 = 256;

static void sf3000_finish(void);

static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + ts.tv_nsec / 1000u;
}

static void load_snd_gain(void)
{
    g_gain_q8 = 256;
    FILE *f = fopen("/mnt/sdcard/cubegm/sndgain.txt", "r");
    if (!f) return;
    int pct = 100;
    if (fscanf(f, "%d", &pct) == 1) {
        if (pct < 0) pct = 0;
        if (pct > 100) pct = 100;
        g_gain_q8 = pct * 256 / 100;
    }
    fclose(f);
}

static void *audio_thread(void *unused)
{
    (void)unused;
    tf_audio_frame chunk[TF_AUDIO_CHUNK];
    uint64_t next_us = 0;
    struct sched_param priority = { .sched_priority = 1 };
    if (pthread_setschedparam(pthread_self(), SCHED_RR, &priority) != 0)
        setpriority(PRIO_PROCESS, (pid_t)syscall(SYS_gettid), -5);
    for (int pass = 0; pass < 2; pass++) {
        int rc = -1;
        for (int attempt = 0; attempt < 20; attempt++) {
            rc = p_init(NULL, 48000, 2);
            if (rc >= 0) break;
            usleep(100000);
        }
        if (rc < 0) {
            __atomic_store_n(&g_ready, -1, __ATOMIC_RELEASE);
            return NULL;
        }
        if (pass == 0) {
            if (p_deinit) p_deinit();
            usleep(100000);
        }
    }
    fprintf(stderr, "out_sf3000: audio driver ready after clean restart; queue target=20ms\n");
    __atomic_store_n(&g_ready, 1, __ATOMIC_RELEASE);
    while (__atomic_load_n(&g_running, __ATOMIC_ACQUIRE)) {
        if (!tf_audio_queue_pull(&g_queue, chunk)) {
            usleep(2000);
            continue;
        }
        if (g_gain_q8 < 256) {
            for (int i = 0; i < TF_AUDIO_CHUNK; i++) {
                chunk[i].left = (int16_t)((int)chunk[i].left * g_gain_q8 >> 8);
                chunk[i].right = (int16_t)((int)chunk[i].right * g_gain_q8 >> 8);
            }
        }
        if (p_playframe(chunk, TF_AUDIO_CHUNK) < 0) {
            fprintf(stderr, "out_sf3000: sound_driver_playframe failed\n");
            __atomic_store_n(&g_running, 0, __ATOMIC_RELEASE);
            break;
        }
        g_output_chunks++;
        uint64_t now = now_us();
        if (!next_us) next_us = now;
        next_us += 10000u;
        if (next_us > now) {
            uint64_t delay = next_us - now;
            if (delay > 20000u) delay = 20000u;
            usleep((useconds_t)delay);
        } else if (now - next_us > 20000u) {
            next_us = now;
        }
    }
    if (p_deinit) p_deinit();
    return NULL;
}

static int sf3000_init(void)
{
    if (g_active) return 0;
    load_snd_gain();
    if (!g_handle) {
        g_handle = dlopen(tf_driver_path(), RTLD_LAZY);
        if (!g_handle) {
            fprintf(stderr, "out_sf3000: dlopen failed: %s\n", dlerror());
            return -1;
        }
        p_init = dlsym(g_handle, "sound_driver_init");
        p_playframe = dlsym(g_handle, "sound_driver_playframe");
        p_deinit = dlsym(g_handle, "sound_driver_deinit");
        if (!p_init || !p_playframe) {
            fprintf(stderr, "out_sf3000: missing audio symbols\n");
            dlclose(g_handle);
            g_handle = NULL;
            return -1;
        }
    }
    if (tf_audio_queue_init(&g_queue, 44100, 20) != 0) return -1;
    g_output_chunks = 0;
    __atomic_store_n(&g_running, 1, __ATOMIC_RELEASE);
    __atomic_store_n(&g_ready, 0, __ATOMIC_RELEASE);
    if (pthread_create(&g_thread, NULL, audio_thread, NULL) != 0) {
        tf_audio_queue_destroy(&g_queue);
        return -1;
    }
    for (int i = 0; i < 2000 && __atomic_load_n(&g_ready, __ATOMIC_ACQUIRE) == 0; i++)
        usleep(1000);
    if (__atomic_load_n(&g_ready, __ATOMIC_ACQUIRE) != 1) {
        __atomic_store_n(&g_running, 0, __ATOMIC_RELEASE);
        pthread_join(g_thread, NULL);
        tf_audio_queue_destroy(&g_queue);
        fprintf(stderr, "out_sf3000: sound_driver_init failed\n");
        return -1;
    }
    g_active = 1;
    atexit(sf3000_finish);
    return 0;
}

static void sf3000_finish(void)
{
    if (!g_active) return;
    g_active = 0;
    __atomic_store_n(&g_running, 0, __ATOMIC_RELEASE);
    pthread_join(g_thread, NULL);
    unsigned underruns, overruns;
    tf_audio_queue_counts(&g_queue, &underruns, &overruns);
    fprintf(stderr, "out_sf3000: session output_chunks=%u underruns=%u overruns=%u\n",
            g_output_chunks, underruns, overruns);
    tf_audio_queue_destroy(&g_queue);
}

static int sf3000_busy(void) { return 0; }

static void sf3000_feed(void *buf, int bytes)
{
    if (!g_active || !__atomic_load_n(&g_running, __ATOMIC_ACQUIRE) || !buf || bytes <= 0) return;
    PROF_AUDIO_BEGIN();
    tf_audio_queue_push(&g_queue, buf, (unsigned)bytes / 4u);
    PROF_AUDIO_END();
}

void out_register_sf3000(struct out_driver *drv)
{
    drv->name = "sf3000";
    drv->init = sf3000_init;
    drv->finish = sf3000_finish;
    drv->busy = sf3000_busy;
    drv->feed = sf3000_feed;
}
