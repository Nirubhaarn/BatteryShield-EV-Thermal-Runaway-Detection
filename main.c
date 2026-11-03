/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : EV Battery Thermal Runaway Early Detection (STM32F401RE)
  *                   Team PIXEL PIRATES
  *
  * Inputs  : RV1-RV3 = cell temperature (PA0, PA1, PA2)
  *           RV4     = battery current  (PA3)
  * Outputs : Green / Yellow / Red LEDs (PB0 / PB2 / PB1)
  *           16x2 LCD in 4-bit mode (pins set in the LCD PIN MAP below)
  *
  * Logic (from the presentation, Risk Prediction Logic slide):
  *   Temperature : <45 C Normal | 45-60 C Warning | >60 C Critical
  *   Gradient    : <1 C/min Normal | 1-3 C/min Warning | >3 C/min Critical
  *   Current     : normal | moderate jump = Warning | very large jump = Critical
  *   Gradient + current spike together -> Critical (early detection)
  *   A state changes only after several consecutive samples agree.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { RISK_NORMAL = 0, RISK_WARNING = 1, RISK_CRITICAL = 2 } RiskState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ===================== PIN MAP (change here if your wiring differs) ===================== */
#define LED_PORT          GPIOB
#define LED_GREEN_PIN     GPIO_PIN_0
#define LED_YELLOW_PIN    GPIO_PIN_2
#define LED_RED_PIN       GPIO_PIN_1

/* LCD PIN MAP: 4-bit mode, RW tied to GND in the schematic */
#define LCD_PORT          GPIOB
#define LCD_RS_PIN        GPIO_PIN_10
#define LCD_E_PIN         GPIO_PIN_11
#define LCD_D4_PIN        GPIO_PIN_12
#define LCD_D5_PIN        GPIO_PIN_13
#define LCD_D6_PIN        GPIO_PIN_14
#define LCD_D7_PIN        GPIO_PIN_15

/* ===================== SENSOR SCALING ===================== */
#define NUM_CELLS         3
#define TEMP_MAX_C        100      /* pot full scale = 100 C  (3.3 V -> 100 C)  */
#define CURRENT_MAX_A     200      /* pot full scale = 200 A  (3.3 V -> 200 A)  */

/* ===================== THRESHOLDS (from the slide) ===================== */
#define TEMP_WARN_X10     450      /* 45.0 C  */
#define TEMP_CRIT_X10     600      /* 60.0 C  */
#define GRAD_WARN_X10     10       /* 1.0 C/min (value x10) */
#define GRAD_CRIT_X10     30       /* 3.0 C/min (value x10) */
#define SPIKE_MOD_A       20       /* moderate current jump in 1 s  */
#define SPIKE_LARGE_A     50       /* very large current jump in 1 s */

/* ===================== TIMING / FILTERING ===================== */
#define EVAL_MS           250      /* sensor read + risk evaluation period */
#define HIST_MS           1000     /* history snapshot period              */
#define WINDOW_S          10       /* gradient window (real seconds)       */
#define HIST_LEN          (WINDOW_S + 1)
#define LCD_REFRESH_MS    500
#define ADC_AVG_SAMPLES   8        /* averaging filter                      */
#define CONFIRM_COUNT     3        /* consecutive samples needed to change state */
#define HOLD_MS           3000     /* minimum time before stepping DOWN a state  */

/* Demo time scale: 1 real second counts as TIME_SCALE simulated seconds.
 * With 6, a rise measured over 10 real seconds is treated as a rise over 1 minute.
 * Example: pot moved +1 % (1 C) in 10 s = 1 C/min (Warning), +4 % in 10 s = 4 C/min (Critical).
 * Set to 1 for real-time values (very slow to demo with a pot). */
#define TIME_SCALE        6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */
static uint32_t temp_x10[NUM_CELLS];            /* filtered temperature, 0.1 C units */
static uint32_t current_A;                      /* filtered current in A              */

static uint32_t hist_temp[NUM_CELLS][HIST_LEN]; /* circular history of temperatures   */
static uint32_t hist_stamp[HIST_LEN];           /* time of each snapshot (ms)         */
static uint8_t  hist_idx = 0;
static uint32_t current_ref_A = 0;              /* current 1 s ago                    */

volatile int32_t   dbg_grad_x10 = 0;            /* max gradient, 0.1 C/min (watch in debugger) */
volatile int32_t   dbg_spike_A  = 0;            /* current jump in last second                  */
volatile RiskState_t risk_state = RISK_NORMAL;  /* confirmed state                              */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
static uint32_t Read_ADC(uint32_t channel);
static uint32_t Read_ADC_Avg(uint32_t channel);
static void     Read_Sensors(void);
static RiskState_t Evaluate_Risk(void);
static RiskState_t Debounce_State(RiskState_t raw, uint32_t now);
static void     Set_Outputs(RiskState_t s);
static void     Critical_Action(void);
static void     LCD_Init(void);
static void     LCD_Update(RiskState_t s);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ------------------------------ ADC ------------------------------ */
static uint32_t Read_ADC(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint32_t value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return value;
}

static uint32_t Read_ADC_Avg(uint32_t channel)
{
    uint32_t sum = 0;
    for (int i = 0; i < ADC_AVG_SAMPLES; i++)
        sum += Read_ADC(channel);
    return sum / ADC_AVG_SAMPLES;
}

static void Read_Sensors(void)
{
    static const uint32_t ch[NUM_CELLS] = { ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2 };

    for (int c = 0; c < NUM_CELLS; c++)
        temp_x10[c] = (Read_ADC_Avg(ch[c]) * (TEMP_MAX_C * 10)) / 4095;

    current_A = (Read_ADC_Avg(ADC_CHANNEL_3) * CURRENT_MAX_A) / 4095;
}

/* ------------------------- RISK LOGIC ------------------------- */
static RiskState_t Evaluate_Risk(void)
{
    /* hottest cell */
    uint32_t t_max = temp_x10[0];
    for (int c = 1; c < NUM_CELLS; c++)
        if (temp_x10[c] > t_max) t_max = temp_x10[c];

    /* gradient: worst (fastest rising) cell, in 0.1 C per simulated minute */
    int32_t grad_max = 0;
    uint32_t now = HAL_GetTick();
    uint32_t elapsed_ms = now - hist_stamp[hist_idx];   /* hist_idx = oldest entry */

    if (elapsed_ms >= 1000)
    {
        for (int c = 0; c < NUM_CELLS; c++)
        {
            int32_t dT = (int32_t)temp_x10[c] - (int32_t)hist_temp[c][hist_idx];
            int32_t g  = (dT * 60000) / (int32_t)(elapsed_ms * TIME_SCALE);
            if (g > grad_max) grad_max = g;
        }
    }
    dbg_grad_x10 = grad_max;

    /* current spike: jump compared with 1 second ago */
    int32_t dI = (int32_t)current_A - (int32_t)current_ref_A;
    dbg_spike_A = dI;
    uint8_t spike = 0;                               /* 0 none, 1 moderate, 2 very large */
    if (dI >= SPIKE_LARGE_A)      spike = 2;
    else if (dI >= SPIKE_MOD_A)   spike = 1;

    /* CRITICAL: very hot, rising very fast, huge spike, or fast rise + spike together */
    if (t_max > TEMP_CRIT_X10 ||
        grad_max > GRAD_CRIT_X10 ||
        spike == 2 ||
        (grad_max >= GRAD_WARN_X10 && spike >= 1))
        return RISK_CRITICAL;

    /* WARNING */
    if (t_max > TEMP_WARN_X10 ||
        grad_max >= GRAD_WARN_X10 ||
        spike >= 1)
        return RISK_WARNING;

    return RISK_NORMAL;
}

/* State changes only after CONFIRM_COUNT consecutive agreeing samples.
 * Stepping down also waits HOLD_MS so a short Red/Yellow event stays visible. */
static RiskState_t Debounce_State(RiskState_t raw, uint32_t now)
{
    static RiskState_t state = RISK_NORMAL;
    static RiskState_t candidate = RISK_NORMAL;
    static uint8_t count = 0;
    static uint32_t last_change = 0;

    if (raw == state)
    {
        count = 0;
        candidate = state;
    }
    else
    {
        if (raw == candidate) { if (count < 255) count++; }
        else                  { candidate = raw; count = 1; }

        if (count >= CONFIRM_COUNT)
        {
            if (raw > state || (now - last_change) >= HOLD_MS)
            {
                state = raw;
                last_change = now;
                count = 0;
            }
        }
    }
    return state;
}

/* ------------------------- OUTPUTS ------------------------- */
static void Set_Outputs(RiskState_t s)
{
    HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN | LED_YELLOW_PIN | LED_RED_PIN, GPIO_PIN_RESET);

    switch (s)
    {
        case RISK_CRITICAL:
            HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_SET);
            Critical_Action();
            break;
        case RISK_WARNING:
            HAL_GPIO_WritePin(LED_PORT, LED_YELLOW_PIN, GPIO_PIN_SET);
            break;
        default:
            HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
            break;
    }
}

/* In a real vehicle: drive buzzer and open the contactor / relay here. */
static void Critical_Action(void)
{
}

/* ------------------------- 16x2 LCD (HD44780, 4-bit) ------------------------- */
static void delay_us(uint32_t us)
{
    volatile uint32_t n = us * 4;
    while (n--) { }
}

static void LCD_Pulse(void)
{
    HAL_GPIO_WritePin(LCD_PORT, LCD_E_PIN, GPIO_PIN_SET);
    delay_us(5);
    HAL_GPIO_WritePin(LCD_PORT, LCD_E_PIN, GPIO_PIN_RESET);
    delay_us(60);
}

static void LCD_Nibble(uint8_t n)
{
    HAL_GPIO_WritePin(LCD_PORT, LCD_D4_PIN, (n & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_PORT, LCD_D5_PIN, (n & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_PORT, LCD_D6_PIN, (n & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_PORT, LCD_D7_PIN, (n & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    LCD_Pulse();
}

static void LCD_Send(uint8_t value, uint8_t is_data)
{
    HAL_GPIO_WritePin(LCD_PORT, LCD_RS_PIN, is_data ? GPIO_PIN_SET : GPIO_PIN_RESET);
    LCD_Nibble(value >> 4);
    LCD_Nibble(value & 0x0F);
}

static void LCD_Cmd(uint8_t cmd)
{
    LCD_Send(cmd, 0);
    if (cmd <= 0x03) HAL_Delay(2);   /* clear / home are slow */
}

static void LCD_Print(uint8_t row, const char *text)
{
    LCD_Cmd(row == 0 ? 0x80 : 0xC0);
    for (int i = 0; i < 16 && text[i]; i++)
        LCD_Send((uint8_t)text[i], 1);
}

static void LCD_Init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(LCD_PORT, LCD_RS_PIN | LCD_E_PIN | LCD_D4_PIN | LCD_D5_PIN |
                                LCD_D6_PIN | LCD_D7_PIN, GPIO_PIN_RESET);
    g.Pin   = LCD_RS_PIN | LCD_E_PIN | LCD_D4_PIN | LCD_D5_PIN | LCD_D6_PIN | LCD_D7_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LCD_PORT, &g);

    HAL_Delay(50);
    HAL_GPIO_WritePin(LCD_PORT, LCD_RS_PIN, GPIO_PIN_RESET);
    LCD_Nibble(0x03); HAL_Delay(5);
    LCD_Nibble(0x03); HAL_Delay(1);
    LCD_Nibble(0x03); HAL_Delay(1);
    LCD_Nibble(0x02); HAL_Delay(1);   /* switch to 4-bit mode */

    LCD_Cmd(0x28);   /* 4-bit, 2 lines, 5x8 font */
    LCD_Cmd(0x0C);   /* display on, cursor off   */
    LCD_Cmd(0x06);   /* auto increment           */
    LCD_Cmd(0x01);   /* clear                    */
}

static void LCD_Update(RiskState_t s)
{
    static char last1[20] = "";
    static char last2[20] = "";
    char l1[20], l2[20];

    switch (s)
    {
        case RISK_CRITICAL: strcpy(l1, "BATTERY:CRITICAL"); break;
        case RISK_WARNING:  strcpy(l1, "BATTERY: WARNING"); break;
        default:            strcpy(l1, "BATTERY: NORMAL "); break;
    }
    /* "T 45 46 47 I120A" = 3 cell temperatures (C) + current (A) */
    snprintf(l2, sizeof(l2), "T%3lu%3lu%3lu I%3luA",
             (unsigned long)(temp_x10[0] / 10),
             (unsigned long)(temp_x10[1] / 10),
             (unsigned long)(temp_x10[2] / 10),
             (unsigned long)current_A);

    if (strcmp(l1, last1) != 0) { LCD_Print(0, l1); strcpy(last1, l1); }
    if (strcmp(l2, last2) != 0) { LCD_Print(1, l2); strcpy(last2, l2); }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();

  /* Fill history with the first reading so there is no false gradient at start-up */
  Read_Sensors();
  uint32_t t0 = HAL_GetTick();
  for (int i = 0; i < HIST_LEN; i++)
  {
      for (int c = 0; c < NUM_CELLS; c++) hist_temp[c][i] = temp_x10[c];
      hist_stamp[i] = t0 - (uint32_t)(HIST_LEN - 1 - i) * HIST_MS;
  }
  hist_idx = 0;                       /* oldest entry */
  current_ref_A = current_A;

  uint32_t last_eval = t0, last_hist = t0, last_lcd = 0;
  Set_Outputs(RISK_NORMAL);
  LCD_Update(RISK_NORMAL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    /* 1) Risk evaluation every EVAL_MS */
    if (now - last_eval >= EVAL_MS)
    {
        last_eval = now;
        Read_Sensors();
        RiskState_t raw = Evaluate_Risk();
        risk_state = Debounce_State(raw, now);
        Set_Outputs(risk_state);
    }

    /* 2) History snapshot every HIST_MS (used for gradient and current spike) */
    if (now - last_hist >= HIST_MS)
    {
        last_hist = now;
        for (int c = 0; c < NUM_CELLS; c++) hist_temp[c][hist_idx] = temp_x10[c];
        hist_stamp[hist_idx] = now;
        hist_idx = (uint8_t)((hist_idx + 1) % HIST_LEN);   /* now points to oldest */
        current_ref_A = current_A;
    }

    /* 3) LCD refresh */
    if (now - last_lcd >= LCD_REFRESH_MS)
    {
        last_lcd = now;
        LCD_Update(risk_state);
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
