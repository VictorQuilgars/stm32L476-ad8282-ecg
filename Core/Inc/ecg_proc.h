#ifndef ECG_PROC_H
#define ECG_PROC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  ECG_NOTCH_50HZ = 50,
  ECG_NOTCH_60HZ = 60
} ecg_notch_t;

typedef struct {
  float   filt_mv;  // échantillon filtré (mV)
  float   feat;     // feature (MWI)
  uint8_t rflag;    // 1 si un pic R a été validé sur cet échantillon
  float   bpm;      // BPM courant (0 si inconnu)
} ecg_sample_out_t;

/** Initialise le pipeline ECG (à appeler une fois au démarrage). */
void ecg_proc_init(float fs_hz, ecg_notch_t notch);

/** Reset propre du pipeline (baseline, filtres, détecteur). */
void ecg_proc_reset(void);

/** Active/désactive le traitement (ex. en cas de lead-off). */
void ecg_proc_set_enabled(int enabled); // 1 = on traite, 0 = on ignore

/** Traite un échantillon (mV) → remplit 'out'. */
void ecg_proc_process_sample(float x_mv, ecg_sample_out_t* out);

/** Traite un bloc ADC (counts → mV → DSP) et appelle 'cb' par sample. */
typedef void (*ecg_proc_callback_t)(const ecg_sample_out_t* s, void* user);
void ecg_proc_process_block(const uint16_t* adc, uint32_t n,
                            float vref_mv, uint8_t adc_bits,
                            ecg_proc_callback_t cb, void* user);

#ifdef __cplusplus
}
#endif
#endif /* ECG_PROC_H */
