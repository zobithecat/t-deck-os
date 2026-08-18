#pragma once

#include <stddef.h>
#include <stdint.h>

#include "snt_piperlite_q8.h"

#ifdef __cplusplus
extern "C" {
#endif

#define KOREAN_TTS_DECODER_HOP 256
#define KOREAN_TTS_DECODER_CHUNK_FRAMES 8
#define KOREAN_TTS_DECODER_HALO_FRAMES 12

typedef int (*korean_tts_pcm_sink_t)(const float *pcm, int samples, void *user);

typedef struct {
    snt_piperlite_q8_model model;
    const float *sample_latent;
    int sample_frames;
} korean_tts_decoder_t;

/* Bind the embedded W8A12 decoder and fixed bring-up latent. */
int korean_tts_decoder_init(korean_tts_decoder_t *decoder);

/* Buffer form used by the host golden gate and custom storage layouts. */
int korean_tts_decoder_init_from_buffers(korean_tts_decoder_t *decoder,
                                         const void *meta, size_t meta_bytes,
                                         const int8_t *weights, size_t weight_bytes,
                                         const float *sample_latent, size_t sample_latent_bytes);

/* Workspace for chunk+halo inference. PSRAM is supported. */
size_t korean_tts_decoder_workspace_bytes(const korean_tts_decoder_t *decoder,
                                          int chunk_frames, int halo_frames);

/* Decode channel-major latent [model.in_ch, frames] without whole-utterance
 * activation storage. Each callback contains a contiguous core chunk. */
int korean_tts_decoder_synthesize(const korean_tts_decoder_t *decoder,
                                  const float *latent, int frames,
                                  int chunk_frames, int halo_frames,
                                  void *workspace, size_t workspace_bytes,
                                  korean_tts_pcm_sink_t sink, void *user);

#ifdef __cplusplus
}
#endif
