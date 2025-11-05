#include "main.h"
#include <string.h>
#include <stdio.h>
#include "ecg_proc.h"

/* ---------- Handles ---------- */
ADC_HandleTypeDef  hadc1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef  htim6;
DMA_HandleTypeDef  hdma_adc1;

/* ---------- Buffers & counters ---------- */
#define ADC_BUF_LEN 250
uint16_t adc_buf[ADC_BUF_LEN];

volatile uint32_t g_samples_total = 0;
static uint32_t   g_fs_real = 0;     // cadence réelle fixée par TIM6_SetRate()
static uint32_t   g_leads_on_ms = 0; // timestamp de la dernière transition LEADS ON

/* ---------- Protos ---------- */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);

static void uart_cb(const ecg_sample_out_t* s, void* user);

/* ---------- Helper: set TIM6 to target rate (auto PSC/ARR) ---------- */
/* Retourne la cadence réellement atteinte (en Hz) */
static uint32_t TIM6_SetRate(uint32_t fs_hz)
{
    // Fréquence horloge du timer (TIM6 sur APB1)
    uint32_t timclk = HAL_RCC_GetPCLK1Freq();
    RCC_ClkInitTypeDef clk; uint32_t flashLatency;
    HAL_RCC_GetClockConfig(&clk, &flashLatency);
    if (clk.APB1CLKDivider != RCC_HCLK_DIV1) {
        timclk *= 2U; // timers à 2*PCLK1 si APB1 prescaler != 1
    }

    // Recherche du meilleur couple (PSC, ARR) 16 bits (arrondi au plus proche)
    uint32_t best_psc = 0, best_arr = 0, best_err = 0xFFFFFFFF;
    for (uint32_t psc = 0; psc <= 0xFFFF; ++psc) {
        uint64_t denom = (uint64_t)(psc + 1U) * (uint64_t)fs_hz;
        uint64_t arrp1 = ((uint64_t)timclk + denom/2) / denom; // round
        if (arrp1 == 0 || arrp1 > 65536ULL) continue;

        uint32_t arr = (uint32_t)(arrp1 - 1U);
        uint32_t fs_real = (uint32_t)((uint64_t)timclk / ((uint64_t)(psc+1U) * (uint64_t)(arr+1U)));
        uint32_t err = (fs_real > fs_hz) ? (fs_real - fs_hz) : (fs_hz - fs_real);

        if (err < best_err) { best_err = err; best_psc = psc; best_arr = arr; }
        if (best_err == 0) break;
    }

    htim6.Instance = TIM6;
    htim6.Init.Prescaler         = (uint16_t)best_psc;
    htim6.Init.Period            = (uint16_t)best_arr;
    htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim6);

    TIM_MasterConfigTypeDef s = {0};
    s.MasterOutputTrigger = TIM_TRGO_UPDATE;
    s.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim6, &s);

    uint32_t fs_real = (uint32_t)((uint64_t)timclk / ((uint64_t)(best_psc+1U) * (uint64_t)(best_arr+1U)));

    char buf[96];
    int n = snprintf(buf, sizeof(buf),
                     "TIM6 set to %lu Hz (PSC=%lu ARR=%lu, err=%lu)\r\n",
                     (unsigned long)fs_real,
                     (unsigned long)best_psc,
                     (unsigned long)best_arr,
                     (unsigned long)((fs_real>fs_hz)?(fs_real-fs_hz):(fs_hz-fs_real)));
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, n, 100);

    return fs_real;
}

/* ======================================================= */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();

  // Petite attente + synchro pour éviter un caractère parasite à l'ouverture du terminal
  HAL_Delay(50);
  HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 20);

  // Init DSP @250 Hz, notch 50 Hz
  ecg_proc_init(250.0f, ECG_NOTCH_50HZ);

  // Active le module (SDN̅ = HIGH sur PA1)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

  const char *msg = "Start ECG acquisition @250Hz\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

  // Cadence exacte 250 Hz selon clocks réelles
  g_fs_real = TIM6_SetRate(250);
  HAL_TIM_Base_Start(&htim6);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);

  /* --- Télémétrie / Lead-off debounce --- */
  static uint32_t last_ms = 0;
  static uint32_t lo_up_ms = 0, lo_dn_ms = 0;
  static int lo_state = 0;       // 0=contacts OK, 1=lead-off
  static int last_lo_print = -1;

  // Sanity-check périodique (toutes les ~10 s) sur fenêtre 2 s
  static uint32_t check_t0_ms = 0;
  if (check_t0_ms == 0) check_t0_ms = HAL_GetTick();

  while (1) {
    uint32_t now = HAL_GetTick();

    // Lire LO+ (PA2) & LO− (PA3)
    int lo_plus_now  = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
    int lo_minus_now = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
    int lo_any = (lo_plus_now || lo_minus_now);

    // Debounce ~20 ms + ON/OFF DSP + fenêtre de stabilisation
    if (lo_any) {
      if (!lo_state) { lo_up_ms = now; }
      if (!lo_state && (now - lo_up_ms >= 20)) {
        lo_state = 1;
        ecg_proc_set_enabled(0);
        ecg_proc_reset();
      }
    } else {
      if (lo_state) { lo_dn_ms = now; }
      if (lo_state && (now - lo_dn_ms >= 20)) {
        lo_state = 0;
        ecg_proc_reset();
        ecg_proc_set_enabled(1);
        g_leads_on_ms = HAL_GetTick(); // start settle window (3 s)
      }
    }

    // Message clair à la transition
    if (lo_state != last_lo_print) {
      last_lo_print = lo_state;
      const char* txt = lo_state ? "LEADS OFF\r\n" : "LEADS ON\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t*)txt, strlen(txt), 50);
    }

    // Télémétrie 1 Hz (rate basé sur g_fs_real, stable)
    if (now - last_ms >= 1000) {
      char line[64];
      int n = snprintf(line, sizeof(line), "rate=%lu sps  LO+=%d LO-=%d\r\n",
                       (unsigned long)g_fs_real, lo_plus_now, lo_minus_now);
      HAL_UART_Transmit(&huart2, (uint8_t*)line, n, 50);
      last_ms = now;
    }

    // Sanity-check ~toutes les 10 s (fenêtre 2 s)
    if (now - check_t0_ms >= 10000) {
      uint32_t t1 = HAL_GetTick(); uint32_t s1 = g_samples_total;
      HAL_Delay(2000); // fenêtre 2 s
      uint32_t t2 = HAL_GetTick(); uint32_t s2 = g_samples_total;
      uint32_t dt = t2 - t1;
      if (dt) {
        uint32_t sps = ((s2 - s1) * 1000u) / dt;
        if (sps > g_fs_real*101/100 || sps < g_fs_real*99/100) {
          char m[64];
          int n=snprintf(m,sizeof(m),"WARN: sps=%lu vs set=%lu\r\n",
                         (unsigned long)sps,(unsigned long)g_fs_real);
          HAL_UART_Transmit(&huart2,(uint8_t*)m,n,50);
        }
      }
      check_t0_ms = HAL_GetTick();
    }
  }
}

/* ---------- Callbacks DMA ADC ---------- */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  g_samples_total += ADC_BUF_LEN/2;
  ecg_proc_process_block(&adc_buf[0], ADC_BUF_LEN/2, 3300.0f, 12, uart_cb, NULL);
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  g_samples_total += ADC_BUF_LEN/2;
  ecg_proc_process_block(&adc_buf[ADC_BUF_LEN/2], ADC_BUF_LEN/2, 3300.0f, 12, uart_cb, NULL);
}

/* ---------- UART sink ---------- */
static void uart_cb(const ecg_sample_out_t* s, void* user)
{
  static uint32_t k=0, last_bpm_ms=0, last_r_ms=0;
  static float bpm_smooth = 0.0f;
  char line[64];

  // Masquer toute publication R/BPM pendant 3 s après LEADS ON
  uint32_t tnow = HAL_GetTick();
  int settling = (g_leads_on_ms != 0) && (tnow - g_leads_on_ms < 3000); // 3 s

  // Réduire le débit UART : 1/16 échantillons -> on loggue le signal filtré
  if ((k++ & 15u)==0) {
    int n = snprintf(line, sizeof(line), "FILT=%.1f\r\n", s->filt_mv);
    HAL_UART_Transmit(&huart2, (uint8_t*)line, n, 50);
  }

  if (settling) return;

  // R-peak : Refractory 250 ms + plage plausible
  if (s->rflag) {
    if (tnow - last_r_ms < 250) {
      return; // ignore: trop rapproché (double détection)
    }
    last_r_ms = tnow;

    if (s->bpm >= 40.0f && s->bpm <= 180.0f) {
      // lissage BPM (IIR ~3 s)
      const float alpha = 0.28f;
      bpm_smooth = (bpm_smooth == 0.0f) ? s->bpm : (1.0f - alpha)*bpm_smooth + alpha*s->bpm;

      int n1 = snprintf(line, sizeof(line), "R, BPM=%.1f\r\n", s->bpm);
      HAL_UART_Transmit(&huart2, (uint8_t*)line, n1, 50);

      int n2 = snprintf(line, sizeof(line), "BPM=%.1f\r\n", bpm_smooth);
      HAL_UART_Transmit(&huart2, (uint8_t*)line, n2, 50);

      last_bpm_ms = tnow;
    }
  } else {
    // BPM continu (toutes ~2 s), lissé, dans la même plage plausible
    if (s->bpm >= 40.0f && s->bpm <= 180.0f && (tnow - last_bpm_ms) > 2000) {
      const float alpha = 0.28f;
      bpm_smooth = (bpm_smooth == 0.0f) ? s->bpm : (1.0f - alpha)*bpm_smooth + alpha*s->bpm;

      int n = snprintf(line, sizeof(line), "BPM=%.1f\r\n", bpm_smooth);
      HAL_UART_Transmit(&huart2, (uint8_t*)line, n, 50);
      last_bpm_ms = tnow;
    }
  }
}

/* =================== Peripherals =================== */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_SYSCLK);
  __HAL_RCC_ADC_CLK_ENABLE();

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  hadc1.Init.ContinuousConvMode    = DISABLE;
  hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode      = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

  // Calibration recommandée sur L4
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) Error_Handler();

  // PA0 = ADC1_IN5
  sConfig.Channel      = ADC_CHANNEL_5;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  hdma_adc1.Instance                 = DMA1_Channel1;
  hdma_adc1.Init.Request             = DMA_REQUEST_0;
  hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1.Init.Mode                = DMA_CIRCULAR;
  hdma_adc1.Init.Priority            = DMA_PRIORITY_HIGH;
  if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) Error_Handler();

  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

static void MX_TIM6_Init(void)
{
  __HAL_RCC_TIM6_CLK_ENABLE();
  // Les paramètres finaux sont posés par TIM6_SetRate(250) juste après.
  htim6.Instance = TIM6;
  htim6.Init.Prescaler         = 7999;
  htim6.Init.Period            = 39;
  htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim6);

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);
}

static void MX_USART2_UART_Init(void)
{
  __HAL_RCC_USART2_CLK_ENABLE();

  huart2.Instance          = USART2;
  huart2.Init.BaudRate     = 115200;
  huart2.Init.WordLength   = UART_WORDLENGTH_8B;
  huart2.Init.StopBits     = UART_STOPBITS_1;
  huart2.Init.Parity       = UART_PARITY_NONE;
  huart2.Init.Mode         = UART_MODE_TX;
  huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};

  // PA0: ADC input
  g.Pin  = GPIO_PIN_0;
  g.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &g);

  // PA1: SDN̅ (module enable)
  g.Pin   = GPIO_PIN_1;
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &g);

  // PA5, PA3: LO+, LO− (inputs) avec pull-up pour éviter le flottement
  g.Pin  = GPIO_PIN_5 | GPIO_PIN_3;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &g);

  // PA2: USART2 TX(AF7)
  g.Pin       = GPIO_PIN_2;
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_NOPULL;
  g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &g);
}

/* ---------- System Clock ---------- */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState       = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange  = RCC_MSIRANGE_6; // 4 MHz
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM       = 1;
  RCC_OscInitStruct.PLL.PLLN       = 40;
  RCC_OscInitStruct.PLL.PLLR       = RCC_PLLR_DIV2; // 80 MHz
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                     RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; // si tu changes, TIM6_SetRate s'adapte
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

/* ---------- IRQ ---------- */
void DMA1_Channel1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc1);
}

/* ---------- Error ---------- */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
