/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "as5600.h"
#include "stdbool.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define Led_Pin GPIO_PIN_13
#define LedChPort GPIOC

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static uint16_t readADC(int channel);
static void printn(int num);
uint16_t readButtons(void);
uint16_t parse_button_data(bool *button_array, uint16_t button_size);
void SetMotorDirection(uint8_t dir);
void setMotorSpeed(uint8_t speed);
uint16_t getSteeringPosition(void);
uint16_t initSteeringCenter(void);

// uint16_t readButDebound(void) ;
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  // float getAngleRaw(void);
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */
  // AS5600_SetSlowFilter();  //set slowFilter <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< MLAG

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* USER CODE END WHILE */

  uint16_t center_wheel = initSteeringCenter();

  while (1)
  {
    sendUSBReport();
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    uint16_t wheel_pos, angle_pos_motor;
    wheel_pos = getSteeringPosition() / 100;
    angle_pos_motor = getSteeringPosition();
    if (angle_pos_motor < center_wheel)
    {
      SetMotorDirection(GPIO_PIN_SET);
    }
    else if (angle_pos_motor > center_wheel)
    {
      SetMotorDirection(GPIO_PIN_RESET);
    }
    else if (angle_pos_motor == 0)
    {
      setMotorSpeed(0);
    }
          
    wheel_pos =  angle_pos_motor -wheel_pos;
    setMotorSpeed(abs(wheel_pos));

    

    HAL_Delay(10);
  }
  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

extern USBD_HandleTypeDef hUsbDeviceFS;
#pragma pack(push, 1)
typedef struct
{
  uint16_t buttons;  // 16 кнопок (битовая маска)
  int16_t wheel;     // 0-65535 (как в дескрипторе)
  uint16_t throttle; // 0-4096 (12-bit ADC)
  uint16_t brake;    // 0-4096
  uint16_t clutch;   // 0-4096

} USB_HID_Report_t;
#pragma pack(pop);

#include "main.h"
#include "stdbool.h"

#define ANGLE_RESOLUTION 4096
#define UINT16_MID 32767
#define UINT16_MAX 65535

static int32_t accumulated_angle = 0; // "Развернутый" угол, может быть <0 или >4095
static uint16_t prev_raw_angle = 0;
static uint8_t initialized = 0;

uint16_t initSteeringCenter(void)
{
  prev_raw_angle = getAngle();
  accumulated_angle = 0;
  initialized = 1;
  return prev_raw_angle;
}

void SetMotorDirection(uint8_t dir)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void setMotorSpeed(uint8_t speed)
{
  // Ограничение скорости 0-100%

  if (speed > 100)
    speed = 1000;
  uint16_t pulse = (48000 * speed / 100);
  TIM3->CCR1 = pulse - 1;
}

uint16_t getSteeringPosition(void)
{
  if (!initialized)
  {
    initSteeringCenter();
  }

  uint16_t current_raw = getAngle();
  int16_t delta = (int16_t)(current_raw - prev_raw_angle);

  // Обработка "перескока" через 0 (unwrap)
  if (delta > (ANGLE_RESOLUTION / 2))
  {
    delta -= ANGLE_RESOLUTION;
  }
  else if (delta < -(ANGLE_RESOLUTION / 2))
  {
    delta += ANGLE_RESOLUTION;
  }

  accumulated_angle += delta;
  prev_raw_angle = current_raw;

  // Масштабируем accumulated_angle в uint16_t диапазон с центром 32767
  // Предположим, что максимальный ход руля +- 4096*2 (примерно два оборота)
  // Скорректируй множитель по своему ходy руля

  int32_t scaled = ((int64_t)accumulated_angle * UINT16_MID) / ((ANGLE_RESOLUTION * 2));
  int32_t pos = (int32_t)UINT16_MID + scaled;

  // wrap-around для uint16_t
  if (pos < 0)
    pos += UINT16_MAX + 1;
  if (pos > UINT16_MAX)
    pos -= UINT16_MAX + 1;

  return (uint16_t)pos;
}

void sendUSBReport(void)
{
  USB_HID_Report_t report = {0};
  // updateVirtualAngle();

  report.buttons = readButtons();

  //  report.buttons = readButtons();
  report.wheel = getSteeringPosition();

  // 12-битные ADC значения (0..4095)
  report.throttle = 0;
  report.brake = 0;
  report.clutch = 0;

  USBD_StatusTypeDef status = USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)&report, sizeof(report));
  if (status != USBD_OK)
  {
    printn((int)status);
  }
}

uint16_t readButtons(void)
{
  uint16_t buttons = 0;
  // 0x0000;
  buttons |= (HAL_GPIO_ReadPin(GPIOB, gpio_but1_Pin) == GPIO_PIN_SET) << 0;
  buttons |= (HAL_GPIO_ReadPin(GPIOB, gpio_but2_Pin) == GPIO_PIN_SET) << 1;
  buttons |= (HAL_GPIO_ReadPin(gpio_but3_GPIO_Port, gpio_but3_Pin) == GPIO_PIN_SET) << 2; // #define gpio_but3_Pin GPIO_PIN_7!!!!!
  buttons |= (HAL_GPIO_ReadPin(GPIOB, gpio_but4_Pin) == GPIO_PIN_SET) << 3;
  buttons |= (HAL_GPIO_ReadPin(GPIOB, gpio_but5_Pin) == GPIO_PIN_SET) << 4;
  buttons |= (HAL_GPIO_ReadPin(GPIOB, gpio_but6_Pin) == GPIO_PIN_SET) << 5;
  buttons |= (HAL_GPIO_ReadPin(GPIOB, gpio_but7_Pin) == GPIO_PIN_SET) << 6;
  buttons |= (HAL_GPIO_ReadPin(GPIOB, gpio_but8_Pin) == GPIO_PIN_SET) << 7;
  buttons |= (HAL_GPIO_ReadPin(GPIOB, gpio_but9_Pin) == GPIO_PIN_SET) << 8;
  buttons |= (HAL_GPIO_ReadPin(GPIOA, gpio_but10_Pin) == GPIO_PIN_SET) << 9;

  printn(buttons);

  return buttons;
}

static void printn(int num)
{
  char buffer[6];
  sprintf(buffer, "%u\r\n", num);

  HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

// reading adc
static uint16_t readADC(int channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  uint16_t value = HAL_ADC_GetValue(&hadc1);
  return value;
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
  /* USER CODE END TIM3_Init 0 */
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // Альтернативная функция Push-Pull
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 48000 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(gpio_dir_GPIO_Port, gpio_dir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : gpio_dir_Pin */
  GPIO_InitStruct.Pin = gpio_dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(gpio_dir_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : gpio_but1_Pin gpio_but2_Pin gpio_but3_Pin gpio_but4_Pin
                           gpio_but5_Pin gpio_but6_Pin gpio_but7_Pin gpio_but8_Pin
                           gpio_but9_Pin PB5 */
  GPIO_InitStruct.Pin = gpio_but1_Pin | gpio_but2_Pin | gpio_but4_Pin | gpio_but5_Pin | gpio_but6_Pin | gpio_but7_Pin | gpio_but8_Pin | gpio_but9_Pin | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : gpio_but10_Pin , gpio_but3_Pin*/
  GPIO_InitStruct.Pin = gpio_but10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(gpio_but10_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = gpio_but3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(gpio_but3_GPIO_Port, &GPIO_InitStruct);
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
