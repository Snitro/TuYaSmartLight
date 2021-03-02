/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "wifi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_0 0xC0
#define LED_1 0xFC

#define	SHT30_ADDR_WRITE	0x44<<1         //10001000
#define	SHT30_ADDR_READ	    (0x44<<1)+1	    //10001011

#define temp_sampling_flash     0x08007000
#define humidity_sampling_flash 0x08007004
#define light_switch_flash      0x08007008
#define system_switch_flash     0x0800700C
#define maxtemp_set_flash       0x08007010
#define minitemp_set_flash      0x08007014
#define maxhum_set_flash        0x08007018
#define minihum_set_flash       0x0800702C
#define bright_value_flash      0x08007030
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t light_switch, system_switch, temp_alarm, hum_alarm, bright_value, reset_flag, sht30_ready;

uint16_t humidity_value, temp_sampling, humidity_sampling, maxhum_set, minihum_set;

int16_t temp_current, maxtemp_set, minitemp_set;

uint8_t u1RX, sampling_flag, random_flag;

uint16_t sampling_cnt;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//设置采样指示灯闪烁频率
void sampling_led_set() {
  uint16_t reload = (temp_sampling + humidity_sampling) * 500 - 1;
  __HAL_TIM_SET_AUTORELOAD(&htim3, reload);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, reload / 2);
}

//开关机
void system_check(uint8_t system) {
      if (system != system_switch) {
        if (system)
          HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        else
          HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
      }
  
  
    system_switch = system;
    sampling_led_set();
    while (system && light_switch > 9)
        light_switch -= 20;
    
    while (!system && light_switch <= 9)
        light_switch += 20;
    
    light_set(light_switch);
}

/* SHT30采样 begin ----------------------------------------------------------*/
static uint8_t SHT30_Send_Cmd(uint16_t cmd) {
    uint8_t cmd_buffer[2];
    cmd_buffer[0] = cmd >> 8;
    cmd_buffer[1] = cmd;
    return HAL_I2C_Master_Transmit(&hi2c1, SHT30_ADDR_WRITE, cmd_buffer, 2, 0xFF);
}

void SHT30_Reset(void) {
    SHT30_Send_Cmd(0x30A2);
    HAL_Delay(20);
}

uint8_t SHT30_Read_Dat(uint8_t* dat) {
	SHT30_Send_Cmd(0xE000);
	return HAL_I2C_Master_Receive(&hi2c1, SHT30_ADDR_READ, dat, 6, 0xFFF);
}

void SHT30_Dat_To_Float(float* temperature, float* humidity) {
    uint8_t dat[6] = {0};
	uint16_t recv_temperature = 0;
	uint16_t recv_humidity = 0;
    
    SHT30_Read_Dat(dat);
	
	/* 转换温度数据 */
	recv_temperature = ((uint16_t)dat[0]<<8)|dat[1];
	*temperature = -45 + 175*((float)recv_temperature/65535);
	
	/* 转换湿度数据 */
	recv_humidity = ((uint16_t)dat[3]<<8)|dat[4];
	*humidity = 100 * ((float)recv_humidity / 65535);
}

uint8_t SHT30_Init(void) {
    return SHT30_Send_Cmd(0x2220);
}
/* SHT30采样 end ------------------------------------------------------------*/

//WS2812 数据
void LED_Transmit(uint8_t *color, uint8_t size) {
    uint8_t pos = 0, R, G, B, color_pos;
    uint8_t data[24 * 8];
    
    for (; pos < size; pos++) {
        R = color[3 * pos];
        G = color[3 * pos + 1];
        B = color[3 * pos + 2];
        
        for (color_pos = 0; color_pos < 8; color_pos++) {
          data[24 * pos + color_pos] = ((G & (1 << 7)) ? (LED_1) : (LED_0));
          G = G << 1;
        }
        
        for (color_pos = 0; color_pos < 8; color_pos++) {
          data[24 * pos + 8 + color_pos] = ((R & (1 << 7)) ? (LED_1) : (LED_0));
          R = R << 1;
        }
        
        for (color_pos = 0; color_pos < 8; color_pos++) {
          data[24 * pos + 16 + color_pos] = ((B & (1 << 7)) ? (LED_1) : (LED_0));
          B = B << 1;
        }
    }
    
    HAL_SPI_Transmit(&hspi1, data, 24 * size, 1000);
    HAL_Delay(1);
}

void color_set(uint8_t R, uint8_t G, uint8_t B) {
  uint8_t color[24], tmp;
    for (tmp = 1; tmp <= 8; tmp++) {
        color[(tmp - 1) * 3] = R;
        color[(tmp - 1) * 3 + 1] = G;
        color[(tmp - 1) * 3 + 2] = B;
    }
  
    LED_Transmit(color, 8);
  
}

// 设置光强
void light_set(uint8_t light) {
  light_switch = light;
  
  uint8_t R = 0, G = 0, B = 0;
  
  switch(light) {
        case 0:
        break;
        
        case 1:
          R = G = B = bright_value;
        break;
        
        case 2:
          R = bright_value;
        break;
        
        case 3:
          G = bright_value;
        break;
        
        case 4:
          B = bright_value;
        break;
        
        case 5:
          G = B = bright_value;
        break;
        
        case 6:
          R = B = bright_value;
        break;
        
        case 7:
          R = G = bright_value;
        break;
        
        case 8:
          B = bright_value;
          R = bright_value * humidity_value / 100;
          G = bright_value - R;
        break;
        
        case 9:
          return;
        break;
    }
  
    color_set(R, G, B);
}

//WS2812  数据处理
void bright_set(uint8_t bright) {
  bright_value = bright;
  light_set(light_switch);
}


//检查是否触发警报
void alerm_checker() {
  if (temp_current < minitemp_set)
    temp_alarm = 0;
  else if (temp_current > maxtemp_set)
    temp_alarm = 1;
  else
    temp_alarm = 2;
  
  if (humidity_value < minihum_set)
    hum_alarm = 0;
  else if (humidity_value > maxhum_set)
    hum_alarm = 1;
  else
    hum_alarm = 2;
  
  mcu_dp_enum_update(DPID_TEMP_ALARM,temp_alarm); //枚举型数据上报;
  mcu_dp_enum_update(DPID_HUM_ALARM,hum_alarm); //枚举型数据上报;
}

//设置并上传温度值
void temp_set(int16_t temp) {
  temp_current = temp;
  
  mcu_dp_value_update(DPID_TEMP_CURRENT,temp_current); //VALUE型数据上报;
  
  alerm_checker();
}

//设置并上传湿度值
void hum_set(uint16_t hum) {
  light_set(light_switch);
  
  humidity_value = hum;
  
  mcu_dp_value_update(DPID_HUMIDITY_VALUE,humidity_value); //VALUE型数据上报;
  
  alerm_checker();
}

// 上电初始化数据
void Data_Init() {
  temp_alarm = hum_alarm = 2;
  light_switch = (uint8_t)readFlash(light_switch_flash);
  system_switch = (uint8_t)readFlash(system_switch_flash);
  temp_sampling = (uint16_t)readFlash(temp_sampling_flash);
  humidity_sampling = (uint16_t)readFlash(humidity_sampling_flash);
  maxtemp_set = (int16_t)readFlash(maxtemp_set_flash);
  minitemp_set = (int16_t)readFlash(minitemp_set_flash);
  maxhum_set = (uint16_t)readFlash(maxhum_set_flash);
  minihum_set = (uint16_t)readFlash(minihum_set_flash);
  bright_value = (uint8_t)readFlash(bright_value_flash);
  sampling_led_set();
    
  bright_set(bright_value);//第一次发送信号不稳
  bright_set(bright_value);
  
  system_check(system_switch);
}

//读取Flash
uint32_t readFlash(uint32_t addr){
  return *(__IO uint32_t*)(addr);
}

//写Flash
void updateFlash(){
  //Unlock
  HAL_FLASH_Unlock();

  //Erase
  FLASH_EraseInitTypeDef f;
  f.TypeErase = FLASH_TYPEERASE_PAGES;
  f.PageAddress = 0x08007000;
  f.NbPages = 1;
  uint32_t PageError = 0;
  HAL_FLASHEx_Erase(&f, &PageError);

  //Write
  HAL_FLASH_Program(TYPEPROGRAM_WORD, temp_sampling_flash, temp_sampling);
  HAL_FLASH_Program(TYPEPROGRAM_WORD, humidity_sampling_flash, humidity_sampling);
  HAL_FLASH_Program(TYPEPROGRAM_WORD, light_switch_flash, light_switch);
  HAL_FLASH_Program(TYPEPROGRAM_WORD, system_switch_flash, system_switch);
  HAL_FLASH_Program(TYPEPROGRAM_WORD, maxtemp_set_flash, maxtemp_set);
  HAL_FLASH_Program(TYPEPROGRAM_WORD, minitemp_set_flash, minitemp_set);
  HAL_FLASH_Program(TYPEPROGRAM_WORD, maxhum_set_flash, maxhum_set);
  HAL_FLASH_Program(TYPEPROGRAM_WORD, minihum_set_flash, minihum_set);
  HAL_FLASH_Program(TYPEPROGRAM_WORD, bright_value_flash, bright_value);

  //Lock
  HAL_FLASH_Lock();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim2) {
      //非实时数据上传
        all_data_update();
    } else if (htim == &htim4 && sht30_ready) {
      //温湿度定时
      if (sampling_cnt-- == 0) {
        sampling_cnt = (temp_sampling / 2 + humidity_sampling / 2);
        sampling_flag = 1;
      }
    } else if (htim == &htim1) {
      random_flag++;
    }
}

//串口发送
void UART1_Transmit(unsigned char value) {
  HAL_UART_Transmit(&huart1, &value, 1, 0xffff);
}

//中断方式接收1字节数据
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  uart_receive_input(u1RX);
  while(HAL_UART_Receive_IT(&huart1, &u1RX,1) != HAL_OK);
}

//重置wifi按钮
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (reset_flag)
    return;
  
  mcu_reset_wifi();
  mcu_set_wifi_mode(SMART_CONFIG);
  
  reset_flag = 1;
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  Data_Init(); 
  
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim1);
  if (system_switch) HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  
  while(HAL_UART_Receive_IT(&huart1, &u1RX,1) != HAL_OK);
  wifi_protocol_init();
  
 
    SHT30_Reset();
    if(SHT30_Init() == HAL_OK) sht30_ready = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    uint8_t R = 255, G = 0, B = 0, pos = 1;
    int8_t color_flag = 1;
  while (1) {
    wifi_uart_service();
    
    if (sampling_flag && system_switch) {
        float temperature, humidity;
        SHT30_Dat_To_Float(&temperature, &humidity);

        hum_set((int16_t)humidity);
        temp_set((int16_t)(temperature * 10));
        
        sampling_flag = 0;
    }
    
    if (random_flag >= 1 && light_switch == 9) {
      random_flag = 0;
      uint8_t over = 0;
      
      if (pos == 0) {
        R += color_flag;
        if ((color_flag == 1 && R == 255) || (color_flag == -1 && R == 0))
          over = 1;
      } else if (pos == 1) {
        G += color_flag;
          if ((color_flag == 1 && G == 255) || (color_flag == -1 && G == 0))
              over = 1;
      } else {
        B += color_flag;
        if ((color_flag == 1 && B == 255) || (color_flag == -1 && B == 0))
          over = 1;
      }
      
      if (over) {
        pos += 2;
        pos %= 3;
        color_flag *= -1;
      }
      
      color_set(R, G, B);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7200 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 50 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7200 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 60000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7200 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7200 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000 - 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
    printf("error");
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
