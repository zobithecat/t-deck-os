/* Model blobs for the sanoTTS decoder benchmark, linked straight into flash.
 *
 * .incbin rather than a generated C array: 1.7 MB as source text is minutes of
 * compile per build for bytes the compiler has nothing to say about. Only built
 * when TTS_BENCH is defined — see src/tts_bench.cpp for why this exists at all.
 *
 * Absolute paths because this is a local experiment against a sibling checkout
 * (zobithecat/korean-sanotts-esp32s3 @ 07bf690), not a vendored dependency.
 */
#ifdef TTS_BENCH

    .section .rodata.tts_bench, "a"

    .global  snt_meta_start
    .balign  16
snt_meta_start:
    .incbin  "/Users/zobithecat/Documents/projects/etc_projects/korean-sanotts-esp32s3/firmware/components/korean_tts_decoder/models/decoder.meta.q8.bin"
    .global  snt_meta_end
snt_meta_end:

    .global  snt_weights_start
    .balign  16
snt_weights_start:
    .incbin  "/Users/zobithecat/Documents/projects/etc_projects/korean-sanotts-esp32s3/firmware/components/korean_tts_decoder/models/decoder.weights.q8.bin"
    .global  snt_weights_end
snt_weights_end:

    .global  snt_latent_start
    .balign  16
snt_latent_start:
    .incbin  "/Users/zobithecat/Documents/projects/etc_projects/korean-sanotts-esp32s3/firmware/components/korean_tts_decoder/models/sample.latent.f32.bin"
    .global  snt_latent_end
snt_latent_end:

#endif /* TTS_BENCH */
