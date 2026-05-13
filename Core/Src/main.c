/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "usbd_cdc_if.h"
#include "i2c-lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD16xN			// For 16xN LCDs
#define TICK_US 8UL // unsigned long
#define ADC_63PCT 2593U // 63% of 3.3V unsigned int

/* Resistors */
#define R_HIGH 100000UL // 100 kohm — small capacitors
#define R_LOW  1000UL   // 1 kohm — large capacitors

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint8_t rxData;
char * txMessage = "Hello from STM32!\r\n";

/* RC measurement */
static uint32_t cap_pF  = 0;     // result of measurement
static uint8_t  use_low_r = 0;   // 0 = 1Mohm, 1 = 10kohm
static uint8_t  meas_error = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void discharge_cap(void)
{
    /* Drive both resistor paths low to discharge through whichever is wired */
    HAL_GPIO_WritePin(GPIO_R_HIGH_GPIO_Port, GPIO_R_HIGH_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_R_LOW_GPIO_Port,  GPIO_R_LOW_Pin,  GPIO_PIN_RESET);

    /* 5 * R_low * C_max = 5 * 10k * 100uF = 5s was the first method but took too long */
    /* New way: trying to dynamically wait till the ADC reaches 63% instead of hardcoding */
    uint32_t adc_val;
    uint32_t timeout = HAL_GetTick() + 6000; // 6 seconds timeout should suffice

    do 
    {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 1);
        adc_val = HAL_ADC_GetValue(&hadc1);
    } while (adc_val > 41 && HAL_GetTick() < timeout); // wait until capacitor is discharged below 63% of 3.3V

}

static uint32_t measure_capacitance(void)
{
    uint32_t ticks;
    uint32_t adc_val;
    uint32_t R_used;

    const uint32_t MAX_OVERFLOWS = 10; // trying to count max overflows before sampling ADC

    meas_error = 0;
    use_low_r = 0;

    // discharge_cap(); // Doing discharge first for time saving, since large capacitor values take longer to charge than to discharge

    retry_with_low_r: // label for retry when goto is called
    R_used = use_low_r ? R_LOW : R_HIGH;
    uint32_t tim_overflow = 0;

    /* Charge capacitor */
    __HAL_TIM_SET_COUNTER(&htim2, 0); // reset timer count
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE); // clear overflow flag
    HAL_TIM_Base_Start(&htim2);
    
    if (use_low_r) 
    {   /* sets 1 kohm */
        HAL_GPIO_WritePin(GPIO_R_LOW_GPIO_Port,  GPIO_R_LOW_Pin,  GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIO_R_HIGH_GPIO_Port, GPIO_R_HIGH_Pin, GPIO_PIN_RESET);
    } else 
    {   /* sets 100 kohm */
        HAL_GPIO_WritePin(GPIO_R_HIGH_GPIO_Port, GPIO_R_HIGH_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIO_R_LOW_GPIO_Port,  GPIO_R_LOW_Pin,  GPIO_PIN_RESET);
    }

    /* ADC overflow */
    // tim_overflow = 0; // overflow reset
    do 
    {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 1);
        adc_val = HAL_ADC_GetValue(&hadc1);

        if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE)) 
        {
            //__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE); // clear and keep counting
            tim_overflow++;

            if (!use_low_r) 
            {
                use_low_r = 1;
                /* discharge between resistor switch */
                HAL_GPIO_WritePin(GPIO_R_HIGH_GPIO_Port, GPIO_R_HIGH_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIO_R_LOW_GPIO_Port,  GPIO_R_LOW_Pin,  GPIO_PIN_RESET);
                uint32_t adc_d;
                uint32_t t_out = HAL_GetTick() + 6000;

                do {
                    HAL_ADC_Start(&hadc1);
                    HAL_ADC_PollForConversion(&hadc1, 1);
                    adc_d = HAL_ADC_GetValue(&hadc1);
                } while (adc_d > 41 && HAL_GetTick() < t_out);

                goto retry_with_low_r;
            } else 
            {
                meas_error = 1;
                return 0;
            }
        }
      } while (adc_val < ADC_63PCT);


    ticks = tim_overflow * 62500UL + __HAL_TIM_GET_COUNTER(&htim2);
    HAL_TIM_Base_Stop(&htim2);

    /* Finish capacitor charging */
    HAL_GPIO_WritePin(GPIO_R_HIGH_GPIO_Port, GPIO_R_HIGH_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_R_LOW_GPIO_Port,  GPIO_R_LOW_Pin,  GPIO_PIN_RESET);

    uint32_t t_us = ticks * TICK_US; 
    // C = t / (R * ln(1/(1-0.63))) ≈ t / (R * 0.994)
    // C_pF = (t_us * 1e-6) / (R * 0.994) * 1e12 = t_us * 1e6 / (R * 0.994)
    uint32_t C_pF = (uint32_t)(( (uint64_t)t_us * 1000000ULL * 1000ULL ) / ( (uint64_t)R_used * 994ULL ));

    return C_pF;
}

/* simple algorithm for reformating */
static void format_cap(uint32_t pF, char *buf, size_t len)
{
    if (pF >= 1000000) {
        uint32_t whole = pF / 1000000;
        uint32_t frac = ((pF % 1000000) * 100) / 1000000;
        snprintf(buf, len, "%lu.%02lu uF", whole, frac);
    } else if (pF >= 1000) {
        uint32_t whole = pF / 1000;
        uint32_t frac = ((pF % 1000) * 100) / 1000;
        snprintf(buf, len, "%lu.%02lu nF", whole, frac);
    } else {
        snprintf(buf, len, "%lu pF", pF);
    }
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
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(10000); // time for LCD init
  lcd_init();

  /* No JTAG*/
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_AFIO_REMAP_SWJ_NOJTAG();

  /* Built in FSM with switch(case) statement state names */
  typedef enum {
    STATE_DISCHARGE,
    STATE_CHARGE
  } MeasState;
  
  MeasState state = STATE_DISCHARGE;
  uint32_t last_display_tick = 0;
  uint8_t last_r = 0;
  
  /* USER CODE END 2 */
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* FSM for capacitance measurement states */
    switch (state)
    {
        case STATE_DISCHARGE:
        {
            HAL_GPIO_WritePin(GPIO_R_HIGH_GPIO_Port, GPIO_R_HIGH_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIO_R_LOW_GPIO_Port,  GPIO_R_LOW_Pin,  GPIO_PIN_RESET);
        
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 1);
            uint32_t adc_val = HAL_ADC_GetValue(&hadc1);
        
            /* small mandatory wait that prevents false ready on floating ADC */
            static uint32_t discharge_start = 0;
            if (discharge_start == 0) discharge_start = HAL_GetTick();
        
            /* at least 50 ms wait time before STATE_CHARGE */
            if (adc_val <= 41 && (HAL_GetTick() - discharge_start) >= 50)
            {
                discharge_start = 0; // reset for next cycle
                state = STATE_CHARGE;
            }
            break;
        }
  
        case STATE_CHARGE:
        {
            cap_pF = measure_capacitance(); /* still blocking but discharge is done */
            last_r = use_low_r; // without this the lcd and usb display 10kOhms

            if (cap_pF == 0 && !meas_error) meas_error = 1; // if no error but zero result, then it is an error
            state = STATE_DISCHARGE;
            break;
        }
  
        default:
            state = STATE_DISCHARGE;
            break;
    }
    /* Update display every 500 ms */
    if (HAL_GetTick() - last_display_tick >= 500)
    {
      last_display_tick = HAL_GetTick();
      
      char cap_str[16];
      /* Saugiklis capacitor measurement */
      if (meas_error) snprintf(cap_str, sizeof(cap_str), "ERROR");
      else format_cap(cap_pF, cap_str, sizeof(cap_str));

      /* LCD lib */
      char lcd_line1[17], lcd_line2[17];
      snprintf(lcd_line1, sizeof(lcd_line1), "C=%-13s", cap_str);
      snprintf(lcd_line2, sizeof(lcd_line2), "R=%-13s", last_r ? "1k" : "100k");
      lcd_put_cur(0, 0); lcd_send_string(lcd_line1);
      lcd_put_cur(1, 0); lcd_send_string(lcd_line2);

      static char usb_buf[64];
      snprintf(usb_buf, sizeof(usb_buf), "C=%s,R=%s\r\n", cap_str, last_r ? "1k" : "100k");
     
      HAL_Delay(500); // delay to make period of display 1s instead of 0,5 s
      CDC_Transmit_FS((uint8_t*)usb_buf, strlen(usb_buf));
    }
    /* USER CODE END WHILE */

  }
    /* USER CODE BEGIN 3 */
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
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
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5; /* changed to longest sample time */
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 575;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 62499;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_R_LOW_GPIO_Port, GPIO_R_LOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : GPIO_R_HIGH_Pin */
  GPIO_InitStruct.Pin = GPIO_R_HIGH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_R_HIGH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_R_LOW_Pin */
  GPIO_InitStruct.Pin = GPIO_R_LOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_R_LOW_GPIO_Port, &GPIO_InitStruct);

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
