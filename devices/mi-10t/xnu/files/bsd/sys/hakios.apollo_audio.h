/*
 * The Xiaomi Mi 10T's (apollo) sound for iOS, kept apart from redwood's (sys/hakios.audio.h):
 *   /dev/apollo-audio   what iOS's audio plug-in writes (userland/src/audio_plugin.c opens it when it is there):
 *                       48 kHz stereo, 16-bit samples, written as they come (bsd/dev/hakios.apollo_audio.c)
 *   the speakers' guard what the samples go through on their way to the amplifiers, for want of Cirrus's speaker
 *                       protection firmware (bsd/dev/hakios.apollo_guard.c)
 * The device speaks the plug-in's protocol: the same frames, and the volume ioctl with the same number.
 */
#ifndef _SYS_APOLLO_AUDIO_H_
#define _SYS_APOLLO_AUDIO_H_

#include <sys/ioccom.h>

#define APOLLO_AUDIO_DEVICE     "/dev/apollo-audio"
#define APOLLO_AUDIO_RATE       48000
#define APOLLO_AUDIO_FRAME      4       /* bytes */

/* the speakers' volume as iOS sets it: eighths of a dB under the loudest, and muted (iOS's volume at 0) */
struct apollo_audio_volume {
	uint32_t below;
	uint32_t muted;
};
#define APOLLO_AUDIO_SET_VOLUME _IOW('r', 1, struct apollo_audio_volume)       /* the plug-in's number */

#ifdef KERNEL
#include <stdbool.h>

/* a sink is offered frames as they are written and says how many it took; called from the writer, must not wait */
typedef uint32_t (*apollo_audio_sink_t)(const int16_t *frames, uint32_t count);

void apollo_audio_init(void);
void apollo_audio_set_sink(const char *name, apollo_audio_sink_t sink);
bool apollo_audio_volume(uint32_t *below, bool *muted, uint32_t *generation);  /* false if iOS has set none */
bool apollo_audio_recording_fed(void);          /* (no recording on apollo yet: false) */
void apollo_audio_record_fed(const int16_t *frames, uint32_t count);

/* the guard: levels in eighths of a dB */
#define APOLLO_GUARD_SILENT     0xffffffffu     /* the amplifiers are off */
struct apollo_guard_report {
	uint32_t blocks, rms_blocks, fast_blocks, slow_blocks;
	uint32_t limited_frames;
	uint32_t peak_in, peak_out;
	uint32_t least_peak_gain;               /* 1/65536 */
	uint64_t least_gain, gain;              /* Q30 */
	uint64_t rms, fast, slow;               /* Q30 of a full-scale square wave at Android's level */
};
#define APOLLO_GUARD_MAX_GAIN   (12 * 8)
void apollo_guard_configure(bool hpf, uint32_t gain);
void apollo_guard_attenuation(uint32_t below);
void apollo_guard_monitor(uint32_t below);
void apollo_guard_process(int16_t *frames, uint32_t count);
void apollo_guard_take_report(struct apollo_guard_report *report);
int apollo_guard_decibels(uint64_t ratio);
#endif /* KERNEL */

#endif /* _SYS_APOLLO_AUDIO_H_ */
