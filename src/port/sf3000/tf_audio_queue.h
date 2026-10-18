#ifndef TF_AUDIO_QUEUE_H
#define TF_AUDIO_QUEUE_H

#include <pthread.h>
#include <stdint.h>

#define TF_AUDIO_CHUNK 480
#define TF_AUDIO_RING 16384

typedef struct { int16_t left, right; } tf_audio_frame;

/* Stereo S16 PCM in, fixed 48 kHz stereo S16 PCM out. The caller owns the DAC. */
typedef struct {
    pthread_mutex_t lock;
    tf_audio_frame ring[TF_AUDIO_RING];
    uint32_t read, write, phase;
    int input_rate, target, slow_q16, primed, recovering;
    tf_audio_frame previous, last;
    unsigned underruns, overruns;
} tf_audio_queue;

int tf_audio_queue_init(tf_audio_queue *q, int input_rate, int target_ms);
void tf_audio_queue_destroy(tf_audio_queue *q);
void tf_audio_queue_push(tf_audio_queue *q, const int16_t *stereo, unsigned frames);
/* Returns zero until initial prefill is reached; otherwise fills 480 frames. */
int tf_audio_queue_pull(tf_audio_queue *q, tf_audio_frame out[TF_AUDIO_CHUNK]);
void tf_audio_queue_counts(tf_audio_queue *q, unsigned *underruns, unsigned *overruns);

#endif
