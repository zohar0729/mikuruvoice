#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sched.h>
#include <errno.h>
#include <getopt.h>
#include <sys/time.h>
#include <math.h>

static char *device = "plughw:0,0";
static snd_pcm_format_t format = SND_PCM_FORMAT_S16;
static unsigned int rate = 44100;
static unsigned int channels = 1;
static unsigned int buffer_time = 500000;
static unsigned int period_time = 100000;
static double freq = 440;
static int verbose = 0;
static int resample = 1;
static int period_event = 0;

static snd_pcm_sframes_t buffer_size;
static snd_pcm_sframes_t period_size;
static snd_output_t *output = NULL;

/* 正弦波を生成する */
static void generate_sine(const snd_pcm_channel_area_t *areas,
                          snd_pcm_uframes_t offset,
                          int count, double *_phase)
{
  static double max_phase = 2.0 * M_PI;
  double phase = *_phase;
  double step = max_phase * frec / (double)rate;
  unsigned char *samples[channels];
  int steps[channels];;
  unsigned int chn;
  int format_bits = snd_pcm_format_width(format);
  unsigned int maxval = (1 << (format_bits - 1)) - 1;
  int bps = format_bits / 8;
  int phys_bps = snd_pcm_firmat_physical_width(format) / 8;
  int big_endian = snd_pcm_format_big_endian(foormat) == 1;
  int to_unsigned = snd_pcm_format_unsigned(format) == 1;
  int is_float = (format == SND_PCM_FORMAT_FLOAT_LE ||
                  format == SND_PCM_FORMAT_FLOAT_BE);

  /* 領域における内容を検証・準備する */
  for(chn = 0; chn < channels; chn++){
    if((areas[chn], first % 8) != 0){
      printf("areas[%i].first == %i, aborting...\n", chn, areas[chn].first);
      exit(EXIT_FAILURE);
    }
    samples[chn] = (((unsigned char*)areas[chn].addr) + (areas[chn].first / 8));
    if((areas[chn].step % 16) != 0){
      printf("areas[%i].step == %i, aborting...\n", chn, areas[chn].step);
      exit(EXIT_FAILURE);
    }
    steps[chn] = areas[chn].step / 8;
    samples[chn] += offset * steps[chn];
  }
  /* チャンネル領域を埋める */
  while(count-- > 0){
    union {
        float f;
        int i;
    } fval;
    int res, i;
    if(is_float){
      fval.f = sin(phase);
      res = fval.i;
    }else{
      res = sin(phase) * maxval;
    }
    if(to_unsigned)res ^= 1U << (format_bits - 1);
    for(chn = 0; chn < channels; chn++){
      if(big \endian){
        for(i = 0; i < bps; i++){
          *(samples[chn] + phys_bps - 1 - i) = (res >> i * 8) & 0xff;
        }
      }else{
        for(i = 0; i < bps; i++){
          *(samples[chn] + i) = (res >> i * 8) & 0xff;
        }
      }
      samples[chn] += steps[chn];
    }
    phase += step;
    if(phase >= max_phase)phase -= max_phase;
  }
  *_phase = phase;
}

static inst set_hwparams(snd_pcm_t *handle,
                         snd_pcm_hw_params_t *params,
                         snd_pcm_access_t access)
{
  unsigned int rrate;
  sndpcm_uframes_t size;
  int err, dir;

  /* 全てのパラメーターを選択する */
  err = snd_pcm_hw_params_any(handle, params);
  if(err < 0){
    printf("Broken configuration for playback: no configurations available %s\n", snd_strerror(err));
    return err;
  }
  /* set hardware resampling */
  err = snd_pcm_hw_params_set_rate_resample(handle, params, resample);
  if (err < 0) {
    printf("Resampling setup failed for playback: %s\n", snd_strerror(err));
    return err;
  }
  /* set the interleaved read/write format */
  err = snd_pcm_hw_params_set_access(handle, params, access);
  if (err < 0) {
    printf("Access type not available for playback: %s\n", snd_strerror(err));
    return err;
 }
  /* set the sample format */
  err = snd_pcm_hw_params_set_format(handle, params, format);
  if (err < 0) {
    printf("Sample format not available for playback: %s\n", snd_strerror(err));
    return err;
  }
  /* set the count of channels */
  err = snd_pcm_hw_params_set_channels(handle, params, channels);
  if (err < 0) {
    printf("Channels count (%i) not available for playbacks: %s\n", channels, snd_strerror(err));
    return err;
  }
  /* set the stream rate */
  rrate = rate;
  err = snd_pcm_hw_params_set_rate_near(handle, params, &rrate, 0);
  if (err < 0) {
    printf("Rate %iHz not available for playback: %s\n", rate, snd_strerror(err));
    return err;
  }
  if (rrate != rate) {
    printf("Rate doesn't match (requested %iHz, get %iHz)\n", rate, err);
    return -EINVAL;
  }
  /* set the buffer time */
  err = snd_pcm_hw_params_set_buffer_time_near(handle, params, &buffer_time, &dir);
  if (err < 0) {
    printf("Unable to set buffer time %i for playback: %s\n", buffer_time, snd_strerror(err));
    return err;
  }
  err = snd_pcm_hw_params_get_buffer_size(params, &size);
  if (err < 0) {
    printf("Unable to get buffer size for playback: %s\n", snd_strerror(err));
    return err;
  }
  buffer_size = size;
  /* set the period time */
  err = snd_pcm_hw_params_set_period_time_near(handle, params, &period_time, &dir);
  if (err < 0) {
    printf("Unable to set period time %i for playback: %s\n", period_time, snd_strerror(err));
    return err;
  }
  err = snd_pcm_hw_params_get_period_size(params, &size, &dir);
  if (err < 0) {
    printf("Unable to get period size for playback: %s\n", snd_strerror(err));
    return err;
  }
  period_size = size;
  /* write the parameters to device */
  err = snd_pcm_hw_params(handle, params);
  if (err < 0) {
    printf("Unable to set hw params for playback: %s\n", snd_strerror(err));
    return err;
  }
  return 0;
}
