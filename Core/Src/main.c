#include "main.h"
#include <string.h>
#include <stdio.h>

ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim6;
DMA_HandleTypeDef hdma_adc1;

volatile uint32_t g_samp_cnt = 0;
uint32_t g_t0 = 0;

#define ADC_BUF_LEN 250
uint16_t adc_buf[ADC_BUF_LEN];

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_DMA_Init(void);

void process_adc_block(uint16_t *buf, uint32_t len);

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();

  // Active le module AD8232 (SDN̅ = HIGH)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

  const char *msg = "Start ECG acquisition @250Hz\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

  // Démarre timer + ADC DMA
  HAL_TIM_Base_Start(&htim6);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);

  while (1)
  {
	  // Heartbeat LED (si tes defines LD_G_* existent), sinon commente.
	  // HAL_GPIO_TogglePin(LD_G_GPIO_Port, LD_G_Pin);

	  // Affiche toutes les 1000 ms le débit et l'état LO+/LO-
	  uint32_t now = HAL_GetTick();
	  if (now - g_t0 >= 1000) {
	    // débit sur la dernière seconde
	    char msg[64];
	    int n = snprintf(msg, sizeof(msg), "rate=%lu sps  LO+=%d LO-=%d\r\n",
	                     (unsigned long)g_samp_cnt,
	                     (int)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0),
	                     (int)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1));
	    HAL_UART_Transmit(&huart2, (uint8_t*)msg, n, 100);
	    g_samp_cnt = 0;
	    g_t0 = now;
	  }

	  // petit sommeil pour laisser tourner les IRQ
	  HAL_Delay(10);
  }
}

/* ---------- ADC complete callback ---------- */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  process_adc_block(&adc_buf[ADC_BUF_LEN / 2], ADC_BUF_LEN / 2);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  process_adc_block(&adc_buf[0], ADC_BUF_LEN / 2);
}

/* ---------- Traitement bloc ECG ---------- */
void process_adc_block(uint16_t *buf, uint32_t len)
{
  char line[32];
  for (uint32_t i = 0; i < len; i++)
  {
    float mv = (buf[i] * 3300.0f) / 4095.0f; // tension en mV
    int n = snprintf(line, sizeof(line), "%.2f\r\n", mv);
    HAL_UART_Transmit(&huart2, (uint8_t*)line, n, 50);
  }
  g_samp_cnt += len;
}

/* ---------- ADC1 config ---------- */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /* ---- ADC clock source + enable AVANT HAL_ADC_Init ---- */
  __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_SYSCLK);   // <- source d’horloge fiable
  __HAL_RCC_ADC_CLK_ENABLE();                      // <- horloge périphérique

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;

  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

  /* Calibration (recommandée sur L4) */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) Error_Handler();

  /* Canal PA0 = ADC1_IN5 */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

/* ---------- DMA init ---------- */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  // DMA1 Channel1 pour ADC1
  hdma_adc1.Instance = DMA1_Channel1;
  hdma_adc1.Init.Request = DMA_REQUEST_0;                 // pour L4
  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1.Init.Mode = DMA_CIRCULAR;                     // double buffer “virtuel” via half/full
  hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
  if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) Error_Handler();

  // Chaîne le DMA au handle ADC
  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

  // NVIC DMA
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/* ---------- TIM6 : 250Hz trigger ---------- */
static void MX_TIM6_Init(void)
{
  __HAL_RCC_TIM6_CLK_ENABLE();
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 7999;   // 80MHz / (7999+1) = 10kHz
  htim6.Init.Period = 39;        // 10kHz / (39+1) = 250Hz
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim6);

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);
}

/* ---------- UART2 init (PD5/PD6) ---------- */
static void MX_USART2_UART_Init(void)
{
  __HAL_RCC_USART2_CLK_ENABLE();

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  HAL_UART_Init(&huart2);
}

/* ---------- GPIO init ---------- */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};

  // PA0: ADC input
  g.Pin = GPIO_PIN_0;
  g.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &g);

  // PA1: SDN̅ AD8232
  g.Pin = GPIO_PIN_1;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &g);

  // PB0, PB1: LO+, LO− (input)
  g.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &g);

  // PD5/PD6: USART2 TX/RX
  g.Pin = GPIO_PIN_5 | GPIO_PIN_6;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &g);
}

/* ---------- System Clock ---------- */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // 4 MHz
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2; // 80 MHz
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}

/* ---------- DMA IRQ ---------- */
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
