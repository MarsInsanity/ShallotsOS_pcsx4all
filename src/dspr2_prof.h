/*
 * dspr2_prof.h - Phase 1 go/no-go profiling for the DSPr2 plan.
 *
 * Measures how much of each displayed frame is spent in the gpu_unai software
 * rasterizer (do_cmd_list) versus total frame wall time. DSPr2 only speeds the
 * rasterizer ALU stage, so if the GPU share is small or the rasterizer is
 * memory-bound this tells us not to bother.
 *
 * Enable by adding -DDSPR2_PROFILE to CFLAGS. When undefined every macro is a
 * no-op, so the shipping build pays nothing.
 *
 * Output: every PROF_DUMP_FRAMES frames, one line to stderr (captured into
 * /mnt/sdcard/log.txt by the launch redirect).
 */
#ifndef DSPR2_PROF_H
#define DSPR2_PROF_H

#ifdef DSPR2_PROFILE

#include <stdio.h>
#include <stdint.h>
#include <sys/time.h>

#define PROF_DUMP_FRAMES 120

static inline uint64_t prof_us(void)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
}

/* MIPS cycle counter (CP0 Count via rdhwr $2). Negligible overhead, so safe to
 * wrap functions called tens of thousands of times per frame (GTE ops) where
 * gettimeofday would distort. Counts free-running CPU cycles.
 * NOTE: if rdhwr $2 is not user-accessible on this kernel, reads garbage/0 and
 * gte_share will look absurd (0 or >100%) — that's the tell. */
static inline uint32_t prof_cc(void)
{
  uint32_t c;
  __asm__ __volatile__("rdhwr %0, $2" : "=r"(c));
  return c;
}

/* Accumulators. Defined once (see DSPR2_PROF_DEFINE in gpulib_if.cpp). */
extern uint64_t g_prof_gpu_us;      /* summed do_cmd_list time this window  */
extern uint64_t g_prof_present_us;  /* summed display-driver time           */
extern uint64_t g_prof_audio_us;    /* summed resample/audio-driver time    */
extern uint64_t g_prof_audio_calls; /* audio feed calls this window         */
extern uint64_t g_prof_calls;       /* do_cmd_list invocations this window  */
extern uint64_t g_prof_window_us;   /* wall time accumulated this window    */
extern uint64_t g_prof_last_flip;   /* timestamp of previous video_flip     */
extern uint32_t g_prof_frames;      /* frames counted this window           */
extern uint64_t g_prof_gpu_tmp;     /* scratch: start time of current call  */
extern uint64_t g_prof_present_tmp; /* scratch: start time of presentation  */
extern uint64_t g_prof_audio_tmp;   /* scratch: start time of audio feed    */
extern uint64_t g_prof_gte_cc;      /* summed GTE func cycles this window    */
extern uint64_t g_prof_gte_calls;   /* GTE func invocations this window      */
extern uint64_t g_prof_frame_cc;    /* CPU cycles accumulated this window    */
extern uint32_t g_prof_cc_last;     /* CC at previous video_flip            */

#define DSPR2_PROF_DEFINE \
  uint64_t g_prof_gpu_us = 0; \
  uint64_t g_prof_present_us = 0; \
  uint64_t g_prof_audio_us = 0; \
  uint64_t g_prof_audio_calls = 0; \
  uint64_t g_prof_calls = 0; \
  uint64_t g_prof_window_us = 0; \
  uint64_t g_prof_last_flip = 0; \
  uint32_t g_prof_frames = 0; \
  uint64_t g_prof_gpu_tmp = 0; \
  uint64_t g_prof_present_tmp = 0; \
  uint64_t g_prof_audio_tmp = 0; \
  uint64_t g_prof_gte_cc = 0; \
  uint64_t g_prof_gte_calls = 0; \
  uint64_t g_prof_frame_cc = 0; \
  uint32_t g_prof_cc_last = 0;

/* Wrap the rasterizer (do_cmd_list body). */
#define PROF_GPU_BEGIN()  do { g_prof_gpu_tmp = prof_us(); } while (0)
#define PROF_GPU_END()    do { \
    g_prof_gpu_us += prof_us() - g_prof_gpu_tmp; \
    g_prof_calls++; \
  } while (0)

/* Wrap the complete presentation path, including any format/aspect staging
 * and the proprietary driver's disp_frame call. */
#define PROF_PRESENT_BEGIN() do { g_prof_present_tmp = prof_us(); } while (0)
#define PROF_PRESENT_END()   do { \
    g_prof_present_us += prof_us() - g_prof_present_tmp; \
  } while (0)

#define PROF_AUDIO_BEGIN() do { g_prof_audio_tmp = prof_us(); } while (0)
#define PROF_AUDIO_END()   do { \
    g_prof_audio_us += prof_us() - g_prof_audio_tmp; \
    g_prof_audio_calls++; \
  } while (0)

/* Wrap a GTE compute func. BEGIN declares a local; END must be in same scope,
 * before the closing brace. cc wraps every 2^32 cycles (~few sec at ~1GHz);
 * delta of two u32 reads handles one wrap correctly via unsigned subtraction. */
#define PROF_GTE_BEGIN()  uint32_t _gte_cc0 = prof_cc()
#define PROF_GTE_END()    do { \
    g_prof_gte_cc += (uint32_t)(prof_cc() - _gte_cc0); \
    g_prof_gte_calls++; \
  } while (0)

/* Call once per displayed frame, at video_flip. */
#define PROF_FRAME_TICK() do { \
    uint64_t _now = prof_us(); \
    uint32_t _cc = prof_cc(); \
    if (g_prof_last_flip != 0) { \
      g_prof_window_us += _now - g_prof_last_flip; \
      g_prof_frame_cc += (uint32_t)(_cc - g_prof_cc_last); \
    } \
    g_prof_last_flip = _now; \
    g_prof_cc_last = _cc; \
    if (++g_prof_frames >= PROF_DUMP_FRAMES) { \
      double _fr_ms  = (double)g_prof_window_us / g_prof_frames / 1000.0; \
      double _gpu_ms = (double)g_prof_gpu_us    / g_prof_frames / 1000.0; \
      double _present_ms = (double)g_prof_present_us / g_prof_frames / 1000.0; \
      double _audio_ms = (double)g_prof_audio_us / g_prof_frames / 1000.0; \
      double _pct    = g_prof_window_us ? \
                       100.0 * (double)g_prof_gpu_us / (double)g_prof_window_us : 0.0; \
      double _gte_pct = g_prof_frame_cc ? \
                       100.0 * (double)g_prof_gte_cc / (double)g_prof_frame_cc : 0.0; \
      fprintf(stderr, "DSPR2_PROF: frame=%.2fms gpu=%.2fms present=%.2fms audio=%.2fms " \
                      "gpu_share=%.1f%% gte_share=%.1f%% gte_calls/frame=%.0f " \
                      "fps=%.1f gpu_calls/frame=%.1f audio_calls/frame=%.1f\n", \
              _fr_ms, _gpu_ms, _present_ms, _audio_ms, _pct, _gte_pct, \
              (double)g_prof_gte_calls / g_prof_frames, \
              _fr_ms > 0 ? 1000.0 / _fr_ms : 0.0, \
              (double)g_prof_calls / g_prof_frames, \
              (double)g_prof_audio_calls / g_prof_frames); \
      g_prof_gpu_us = g_prof_present_us = g_prof_window_us = g_prof_calls = 0; \
      g_prof_audio_us = g_prof_audio_calls = 0; \
      g_prof_gte_cc = g_prof_gte_calls = g_prof_frame_cc = 0; \
      g_prof_frames = 0; \
    } \
  } while (0)

#else /* !DSPR2_PROFILE */

#define DSPR2_PROF_DEFINE
#define PROF_GPU_BEGIN()  do {} while (0)
#define PROF_GPU_END()    do {} while (0)
#define PROF_PRESENT_BEGIN() do {} while (0)
#define PROF_PRESENT_END()   do {} while (0)
#define PROF_AUDIO_BEGIN() do {} while (0)
#define PROF_AUDIO_END()   do {} while (0)
#define PROF_GTE_BEGIN()  do {} while (0)
#define PROF_GTE_END()    do {} while (0)
#define PROF_FRAME_TICK() do {} while (0)

#endif /* DSPR2_PROFILE */

#endif /* DSPR2_PROF_H */
