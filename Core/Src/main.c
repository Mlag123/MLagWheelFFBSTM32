
#include "main.h"
#include "usb_device.h"
#include "as5600.h"
#include "stdbool.h"
#include "main.h"
#include "stdbool.h"

#define Led_Pin GPIO_PIN_13
#define LedChPort GPIOC

#define ANGLE_RESOLUTION 4096
#define UINT16_MID 32767
#define UINT16_MAX 65535

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

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
void readAllADC(uint16_t* throttle, uint16_t* brake);
static void print_text(const char* text);

#pragma pack(push, 1)
typedef struct
{
  uint16_t buttons;  // 16 кнопок (битовая маска)
  uint16_t wheel;     // 0-65535 (как в дескрипторе)
  uint16_t throttle; // 0-4096 (12-bit ADC)
  uint16_t brake;    // 0-4096
  uint16_t clutch;   // 0-4096

} USB_HID_Report_t;
#pragma pack(pop);

/**
 * @brief  The application entry point.
 * @retval int
 */

int main(void)
{

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  uint16_t center_wheel = initSteeringCenter();

  while (1)
  {
    sendUSBReport();

    print_text("Report size: ");
    printn(sizeof(USB_HID_Report_t));
    print_text(" bytes\r\n");


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

    wheel_pos = angle_pos_motor - wheel_pos;
    setMotorSpeed(abs(wheel_pos));

    HAL_Delay(10);
  }
  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

extern USBD_HandleTypeDef hUsbDeviceFS;


static int32_t accumulated_angle = 0; //
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

  int32_t scaled = ((int64_t)accumulated_angle * UINT16_MID) / ((ANGLE_RESOLUTION * 2));
  int32_t pos = (int32_t)UINT16_MID + scaled;

  // wrap-around для uint16_t
  if (pos < 0)
    pos += UINT16_MAX + 1;
  if (pos > UINT16_MAX)
    pos -= UINT16_MAX + 1;

  return (uint16_t)pos;
}
uint16_t throttle_val, brake_val;
int16_t wheel_rotate;

void sendUSBReport(void)
{
  USB_HID_Report_t report = {0};



  static int16_t wheel_angle = 0;       // накопленный угол
  static int16_t last_wheel_angle = 0;  // предыдущее значение
  int16_t raw_wheel_angle;              // текущее положение
  int16_t wheel_delta;                  // разница между текущим и прошлым


  static int16_t throttle_angle = 0;       // накопленный угол
  static int16_t last_throttle_angle = 0;  // предыдущее значение
  int16_t raw_throttle_angle;              // текущее положение
  int16_t throttle_delta;



  static int16_t brake_angle = 0;       // накопленный угол
  static int16_t last_brake_angle = 0;  // предыдущее значение
  int16_t raw_brake_angle;              // текущее положение
  int16_t brake_delta;
  readAllADC(&brake_val, &throttle_val);



  // считываем текущее положение руля (например, от 0 до 4095)
  raw_wheel_angle = getSteeringPosition();

  // вычисляем изменение
  wheel_delta = raw_wheel_angle - last_wheel_angle;

  // фильтруем мелкие шумы (например, ±2 единицы)
  if (abs(wheel_delta) > 2) {
      wheel_angle += wheel_delta;
      last_wheel_angle = raw_wheel_angle;
  }


  //throttle



  raw_throttle_angle = throttle_val;
  throttle_delta = raw_throttle_angle - last_throttle_angle;

  if(abs(throttle_delta)>2){
	  throttle_angle =raw_throttle_angle;
	  last_throttle_angle = raw_throttle_angle;
  }


  //brake

  raw_brake_angle = brake_val;
  brake_delta = raw_brake_angle - last_brake_angle;

    if(abs(brake_delta)>2){
    	brake_angle =raw_brake_angle;
  	  last_brake_angle = raw_brake_angle;
    }



  //brake
  //throttle
  // updateVirtualAngle();


  static uint32_t report_count = 0;

  report.wheel = 0;
 // report.buttons = readButtons();

  report.buttons = readButtons();
  report.wheel = wheel_rotate;


  // 12-битные ADC значения (0..4095)
  report.throttle = throttle_angle;
  report.brake = brake_angle;
  report.clutch = 5; //WIP


  if (report_count % 50 == 0) {
	  print_text("Throttle: "); printn(report.throttle);
	  print_text("Brake: "); printn(report.brake);
	  print_text("Wheel: "); printn(report.wheel);
	  print_text("---\n");
  }


  USBD_StatusTypeDef status = USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)&report, sizeof(report));
  if (status != USBD_OK)
  {
    printn((int)status);
  }
}

void readMatrixButtons(void)
{ // testcode
  uint8_t button_matrix[4][4] = {0};

  uint32_t row_pins[4] = {gpio_but1_Pin, gpio_but2_Pin, gpio_but3_Pin, gpio_but4_Pin};
  uint32_t col_pins[4] = {gpio_but5_Pin, gpio_but6_Pin, gpio_but7_Pin, gpio_but8_Pin};

  for (uint8_t row = 0; row < 4; row++)
  {
    if (row_pins == gpio_but3_Pin)
    {
      HAL_GPIO_WritePin(GPIOA, row_pins[row], GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(GPIOB, row_pins[row], GPIO_PIN_SET);
    }
    for (uint16_t col; col < 4; col++)
    {
      button_matrix[row][col] = HAL_GPIO_ReadPin(GPIOB, row_pins[row]) == GPIO_PIN_RESET;

      if (row_pins == gpio_but3_Pin)
      {
        HAL_GPIO_WritePin(GPIOA, row_pins[row], GPIO_PIN_RESET);
      }
      else
      {
        HAL_GPIO_WritePin(GPIOB, row_pins[row], GPIO_PIN_RESET);
      }
    }
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
static void print_text(const char* text)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)text, strlen(text), HAL_MAX_DELAY);
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

  // Останавливаем ADC перед реконфигурацией
  HAL_ADC_Stop(&hadc1);

  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5; // Увеличить время выборки
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // Запускаем преобразование
  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) != HAL_OK)
  {
    Error_Handler();
  }

  uint16_t value = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  return value;
}
void readAllADC(uint16_t* throttle, uint16_t* brake)
{
  // Читаем каждый канал отдельно
  *throttle = readADC(ADC_CHANNEL_0);
  *brake = readADC(ADC_CHANNEL_1);
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
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Common config
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;        // ОТКЛЮЧИТЬ сканирование!
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;                   // 1 канал!
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  // Настраиваем только канал 0 (остальное уберется)
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

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

  HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

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
