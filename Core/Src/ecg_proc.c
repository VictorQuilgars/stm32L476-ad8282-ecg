#include <math.h>
#include <string.h>
#include "ecg_proc.h"

/* ----------------- Petites briques DSP ----------------- */
typedef struct { float b0,b1,b2,a1,a2, z1,z2; } biquad_t;
static inline void biquad_init(biquad_t* f, float b0,float b1,float b2,float a1,float a2){
  f->b0=b0; f->b1=b1; f->b2=b2; f->a1=a1; f->a2=a2; f->z1=0.0f; f->z2=0.0f;
}
static inline float biquad_process(biquad_t* f, float x){
  float y = f->b0*x + f->z1;
  f->z1 = f->b1*x - f->a1*y + f->z2;
  f->z2 = f->b2*x - f->a2*y;
  return y;
}

/* High-pass baseline (EMA ~0.5 Hz) avec init anti-pic */
typedef struct { float alpha, baseline; uint8_t init; } hp_ema_t;
static inline void hp_ema_init(hp_ema_t* h, float fs, float fc){
  h->alpha = 2.0f*(float)M_PI*fc/fs;
  if (h->alpha < 1e-4f) h->alpha = 1e-4f;
  if (h->alpha > 0.2f)  h->alpha = 0.2f;
  h->baseline = 0.0f;
  h->init = 0;
}
static inline float hp_ema_process(hp_ema_t* h, float x){
  if (!h->init) { h->baseline = x; h->init = 1; }
  h->baseline += h->alpha * (x - h->baseline);
  return x - h->baseline;
}

/* Moving Window Integration (~150 ms @ fs) */
#define MWI_MAX 128
typedef struct { float buf[MWI_MAX]; uint16_t idx, len; float acc; } mwi_t;
static inline void mwi_init(mwi_t* m, uint16_t len){
  if (len > MWI_MAX) len = MWI_MAX;
  memset(m, 0, sizeof(*m)); m->len = len;
}
static inline float mwi_push(mwi_t* m, float x){
  m->acc -= m->buf[m->idx];
  m->buf[m->idx] = x;
  m->acc += x;
  if (++m->idx >= m->len) m->idx = 0;
  return m->acc / (float)m->len;
}

/* Détecteur R (seuil adaptatif + peak-hold + réfractaire) */
typedef struct {
  float thr, noise, signal;
  uint32_t ref_smp, ref_cnt;
  uint8_t was_high;
  uint32_t last_r_idx;
  float bpm;
  float peak_feat;
  uint32_t peak_idx;
} rdet_t;

static inline void rdet_init(rdet_t* d, uint32_t fs){
  d->thr=0.02f; d->noise=0.02f; d->signal=0.05f;
  d->ref_smp = (uint32_t)(0.300f * fs); // 300 ms
  d->ref_cnt=0; d->was_high=0; d->last_r_idx=0; d->bpm=0.0f;
  d->peak_feat = 0.0f; d->peak_idx = 0;
}
static inline int rdet_process(rdet_t* d, float feat, uint32_t idx, uint32_t fs){
  // MAJ niveaux
  const float w_sig=0.125f, w_noi=0.125f;
  if (feat > d->thr) d->signal = (1.0f-w_sig)*d->signal + w_sig*feat;
  else               d->noise  = (1.0f-w_noi)*d->noise  + w_noi*feat;
  d->thr = 0.5f*d->noise + 0.5f*d->signal*0.6f;

  if (d->ref_cnt) d->ref_cnt--;

  int r = 0;
  if (feat > d->thr) {
    if (!d->was_high) { d->peak_feat = feat; d->peak_idx = idx; }
    else if (feat > d->peak_feat) { d->peak_feat = feat; d->peak_idx = idx; }
    d->was_high = 1;
  } else {
    if (d->was_high && !d->ref_cnt) {
      uint32_t rr = d->peak_idx - d->last_r_idx;
      if (d->last_r_idx && rr > (uint32_t)(0.3f*fs) && rr < (uint32_t)(2.0f*fs)) {
        d->bpm = 60.0f * ((float)fs / (float)rr);
      }
      d->last_r_idx = d->peak_idx;
      d->ref_cnt = d->ref_smp;
      r = 1;
    }
    d->was_high = 0;
    d->peak_feat = 0.0f;
  }
  return r;
}

/* ----------------- Contexte global ----------------- */
static float    g_fs   = 250.0f;
static uint32_t g_fs_u = 250;
static uint32_t g_index = 0;

static hp_ema_t g_hp;
static biquad_t g_notch, g_lpf;
static mwi_t    g_mwi;
static rdet_t   g_rdet;
static float    g_prev_y = 0.0f;

static int      g_enabled = 1; // lead-off → 0

/* Coeffs biquads pour fs=250 Hz */
static void setup_notch_50(void){
  // Notch 50 Hz, Q≈30
  biquad_init(&g_notch, 0.983916f, -1.616422f, 0.983916f, -1.616422f, 0.967832f);
}
static void setup_notch_60(void){
  // Notch 60 Hz, Q≈30
  biquad_init(&g_notch, 0.979389f, -1.438254f, 0.979389f, -1.438254f, 0.958778f);
}
static void setup_lpf_15(void){
  // Passe-bas ~15 Hz (Butterworth 2e ordre)
  biquad_init(&g_lpf, 0.067455f, 0.134911f, 0.067455f, -1.142980f, 0.412802f);
}

void ecg_proc_init(float fs_hz, ecg_notch_t notch){
  g_fs = fs_hz;
  g_fs_u = (uint32_t)(fs_hz + 0.5f);
  hp_ema_init(&g_hp, g_fs, 0.5f);
  (notch==ECG_NOTCH_60HZ) ? setup_notch_60() : setup_notch_50();
  setup_lpf_15();
  mwi_init(&g_mwi, (uint16_t)(0.150f * g_fs)); // ~150 ms
  if (g_mwi.len < 8) g_mwi.len = 8;
  rdet_init(&g_rdet, g_fs_u);
  g_prev_y = 0.0f;
  g_index = 0;
  g_enabled = 1;
}

void ecg_proc_reset(void){
  hp_ema_init(&g_hp, g_fs, 0.5f);
  setup_lpf_15();
  mwi_init(&g_mwi, (uint16_t)(0.150f * g_fs));
  if (g_mwi.len < 8) g_mwi.len = 8;
  rdet_init(&g_rdet, g_fs_u);
  g_prev_y = 0.0f;
  g_index = 0;
}

void ecg_proc_set_enabled(int enabled){ g_enabled = enabled; }

void ecg_proc_process_sample(float x_mv, ecg_sample_out_t* out){
  // si désactivé (lead-off), sortir à 0
  if (!g_enabled) { out->filt_mv=0.0f; out->feat=0.0f; out->rflag=0; out->bpm=g_rdet.bpm; return; }

  // 1) baseline removal
  float y = hp_ema_process(&g_hp, x_mv);
  // 2) notch
  y = biquad_process(&g_notch, y);
  // 3) low-pass
  y = biquad_process(&g_lpf, y);
  // clamp anti-artefact
  if (y > 2000.0f) y = 2000.0f;
  if (y < -2000.0f) y = -2000.0f;

  // 4) Pan-Tompkins light : diff → square → MWI
  float diff = y - g_prev_y; g_prev_y = y;
  float sq = diff * diff;
  float feat = mwi_push(&g_mwi, sq);

  // 5) Détection R
  int r = rdet_process(&g_rdet, feat, g_index++, g_fs_u);

  out->filt_mv = y;
  out->feat    = feat;
  out->rflag   = (uint8_t)r;
  out->bpm     = g_rdet.bpm;
}

void ecg_proc_process_block(const uint16_t* adc, uint32_t n,
                            float vref_mv, uint8_t adc_bits,
                            ecg_proc_callback_t cb, void* user){
  if (!g_enabled) return;
  const float scale = vref_mv / (float)((1u<<adc_bits)-1u);
  ecg_sample_out_t s;
  for (uint32_t i=0; i<n; ++i){
    float mv = adc[i] * scale; // mV si vref_mv est en mV
    ecg_proc_process_sample(mv, &s);
    if (cb) cb(&s, user);
  }
}
