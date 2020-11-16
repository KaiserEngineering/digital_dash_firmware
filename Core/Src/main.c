/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "adc.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "lib_digital_dash.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*******************************************************************************
 * Begin Debug Flags ( 0 = Disable, 1 = Enable )
 ******************************************************************************/

#define LCD_ALWAYS_ON       1 /* Force the LCD on as long as the Pi can boot */
#define CAN_IT              1 /* Enable/disable CAN bus interrupts */

/*******************************************************************************
 * End Debug Flags
 ******************************************************************************/

/* STM32F446 has 14 dedicated filter banks for CAN 1 */
#define STM32F446_NUM_CAN1_FILTERBANK 14

/* STM32F446 has 14 filter banks for CAN 1 or CAN 2 */
#define STM32F446_NUM_CAN2_FILTERBANK 14

/* The current firmware is only using CAN1 and we don't have a need to occupy the upper 14 filters */
#define MAX_NUM_CAN_FILTERS STM32F446_NUM_CAN1_FILTERBANK

/* Ford Backlight timer defines */
#define FORD_FOCUS_BACKLIGHT_TIM                &htim13
#define FORD_FOCUS_BACKLIGHT_TIM_CHANNEL        TIM_CHANNEL_1
#define FORD_FOCUS_BACKLIGHT_TIM_ACTIVE_CHANNEL HAL_TIM_ACTIVE_CHANNEL_1
#define FORD_FOCUS_BACKLIGHT_TIMEOUT_TIM        &htim11

/* Digital Dash LCD timer */
#define BKLT_TIM     htim14
#define PBKLT_TIM    &BKLT_TIM
#define BKLT_CHANNEL TIM_CHANNEL_1
#define BKLT_MIN     10  /* Lowest Duty cycle before the screen turns off */
#define BKLT_MAX     255 /* Max Duty cycle */

#define PI_UART &huart1 /* Raspberry Pi communication channel */
#define ECU_CAN &hcan1  /* Primary CAN bus communication (OEM connector) */

#define FAN_PWM 0       /* PCB-DDMB-STRS REV A does not have PWM functionality */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BITSET(word,nbit)   ((word) |=  (1<<(nbit)))
#define BITCLEAR(word,nbit) ((word) &= ~(1<<(nbit)))
#define BITFLIP(word,nbit)  ((word) ^=  (1<<(nbit)))
#define BITCHECK(word,nbit) ((word) &   (1<<(nbit)))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* UART RX'd byte */
static uint8_t rx_byte;

/* Timers for debug LEDs */
static uint32_t LED1_Tick = 0;
static uint32_t LED2_Tick = 0;
static uint32_t LED3_Tick = 0;

static uint32_t Sys_Tick  = 0;

/* Number of filters in use */
static uint8_t CAN_Filter_Count = 0;

/* Flag to override auto LCD brightness */
static uint8_t gpio_LCD_override = 0;

volatile uint32_t can_tx_mailbox_status = 0;
volatile uint32_t can_rx_mailbox_status = 0;

static volatile uint32_t system_flags = 0;
    #define SD_CARD_PRESENT     0
    #define PI_PWR_EN           1
    #define PI_PWR_FAULT        2
    #define LCD_PWR_EN          3
    #define LCD_BACKLIGHT_EN    4
    #define USB_PWR_EN          5
    #define USB_PWR_FAULT       6
    #define FAN_PWR_EN          7
    #define FAN_PWR_FAULT       8
    #define CAN1_EN             9

#define MCU_THERMAL_THRESH 50
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef enum _sys_pwr_hold {
	SYS_PWR_HOLD_DISABLE,
	SYS_PWR_HOLD_ENABLE
} SYS_PWR_HOLD, *PSYS_PWR_HOLD;

static void System_Power_Hold( SYS_PWR_HOLD state )
{
	if( state == SYS_PWR_HOLD_ENABLE )
		HAL_GPIO_WritePin( PWR_HOLD_GPIO_Port, PWR_HOLD_Pin, PWR_HOLD_Active );
	else
		HAL_GPIO_WritePin( PWR_HOLD_GPIO_Port, PWR_HOLD_Pin, PWR_HOLD_Inactive );
}

typedef enum _debug_led {
    DEBUG_LED_1,
    DEBUG_LED_2,
    DEBUG_LED_3,
} DEBUG_LED, *PDEBUG_LED;

static void flash_led( DEBUG_LED led )
{
    if( led == DEBUG_LED_1 )
    {
        LED1_Tick = 50;
        HAL_GPIO_WritePin( DEBUG_LED_1_GPIO_Port, DEBUG_LED_1_Pin, GPIO_PIN_SET );
    }
    else if( led == DEBUG_LED_2 )
    {
        LED2_Tick = 50;
        HAL_GPIO_WritePin( DEBUG_LED_2_GPIO_Port, DEBUG_LED_2_Pin, GPIO_PIN_SET );
    }
    else if( led == DEBUG_LED_3 )
    {
        LED3_Tick = 50;
        HAL_GPIO_WritePin( DEBUG_LED_3_GPIO_Port, DEBUG_LED_3_Pin, GPIO_PIN_SET );
    }
}


void HAL_CAN_TxMailbox0CompleteCallback( CAN_HandleTypeDef *hcan )
{
    can_tx_mailbox_status &= ~CAN_TX_MAILBOX0;
}

void HAL_CAN_TxMailbox1CompleteCallback( CAN_HandleTypeDef *hcan )
{
    can_tx_mailbox_status &= ~CAN_TX_MAILBOX1;
}

void HAL_CAN_TxMailbox2CompleteCallback( CAN_HandleTypeDef *hcan )
{
    can_tx_mailbox_status &= ~CAN_TX_MAILBOX2;
}

HAL_StatusTypeDef can_filter( CAN_HandleTypeDef *pcan, uint32_t id, uint32_t mask, uint32_t format, uint32_t filterBank, uint32_t FIFO  )
{
	/* Verify correct format */
    if ( (format == CAN_ID_STD) || (format == CAN_ID_EXT) )
    {
        /* Declare a CAN filter configuration */
        CAN_FilterTypeDef  sFilterConfig;

        /* Verify the filter bank is possible */
        if ( ( filterBank >= 0 ) && ( filterBank <= 13 ) )
            sFilterConfig.FilterBank = filterBank;
        else
            return -1;

        sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
        sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

        if (format == CAN_ID_STD) {
            sFilterConfig.FilterIdHigh = ((id << 5) | (id >> (32 - 5))) & 0xFFFF;
            sFilterConfig.FilterIdLow =  (id >> (11-3)) & 0xFFF8;
            sFilterConfig.FilterMaskIdHigh = ((mask << 5) | (mask >> (32-5))) & 0xFFFF;
            sFilterConfig.FilterMaskIdLow = (mask >> (11-3)) & 0xFFF8;
        } else { // format == CANExtended
            sFilterConfig.FilterIdHigh = id >> 13; // EXTID[28:13]
            sFilterConfig.FilterIdLow = (0xFFFF & (id << 3)) | (1 << 2); // EXTID[12:0] + IDE
            sFilterConfig.FilterMaskIdHigh = mask >> 13;
            sFilterConfig.FilterMaskIdLow = (0xFFFF & (mask << 3)) | (1 << 2);
        }

        sFilterConfig.FilterFIFOAssignment = FIFO;
        sFilterConfig.FilterActivation = ENABLE;
        sFilterConfig.FilterBank = filterBank;
        sFilterConfig.SlaveStartFilterBank = 0x12;

        return HAL_CAN_ConfigFilter(pcan, &sFilterConfig);
    }
    return HAL_ERROR;
}

void process_can_packet( CAN_HandleTypeDef *hcan, uint32_t fifo )
{
    flash_led( DEBUG_LED_1 );
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_buf[OBDII_DLC];
    HAL_CAN_GetRxMessage( hcan, fifo, &rx_header, rx_buf );

    if( rx_header.IDE == CAN_ID_STD )
        DigitalDash_Add_CAN_Packet( rx_header.StdId ,rx_buf);

    can_rx_mailbox_status |= fifo;
}

void HAL_CAN_RxFifo0MsgPendingCallback( CAN_HandleTypeDef *hcan )
{
    process_can_packet( hcan, CAN_RX_FIFO0 );
}

void HAL_CAN_RxFifo1MsgPendingCallback( CAN_HandleTypeDef *hcan )
{
    process_can_packet( hcan, CAN_RX_FIFO1 );
}

static void LCD_Brightness( uint8_t brightness)
{
    if( gpio_LCD_override )
    {
        if( BITCHECK( system_flags, LCD_BACKLIGHT_EN ) == 0 )
        {
            /* Backlight is enabled */
            BITSET( system_flags, LCD_BACKLIGHT_EN );

            brightness = 255;

            BKLT_TIM.Instance->CCR1 = brightness;
            HAL_GPIO_WritePin(LCD_DR_ENR_GPIO_Port, LCD_DR_ENR_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LCD_DR_EN_GPIO_Port, LCD_DR_EN_Pin, GPIO_PIN_SET);
            if( (BKLT_TIM.Instance->CR1 & TIM_CR1_CEN) == 0 )
            	HAL_TIM_PWM_Start(PBKLT_TIM, BKLT_CHANNEL);
        }
    }
    else if( brightness == 0x00 )
    {
#if !LCD_ALWAYS_ON
        /* Backlight is not enabled */
        BITCLEAR( system_flags, LCD_BACKLIGHT_EN );

        HAL_GPIO_WritePin(LCD_DR_ENR_GPIO_Port, LCD_DR_ENR_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LCD_DR_EN_GPIO_Port, LCD_DR_EN_Pin, GPIO_PIN_RESET);
        HAL_TIM_PWM_Stop(PBKLT_TIM, BKLT_CHANNEL);
#endif
    } else
    {
        /* Backlight is enabled */
        BITSET( system_flags, LCD_BACKLIGHT_EN );

        BKLT_TIM.Instance->CCR1 = brightness;
        HAL_GPIO_WritePin(LCD_DR_ENR_GPIO_Port, LCD_DR_ENR_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LCD_DR_EN_GPIO_Port, LCD_DR_EN_Pin, GPIO_PIN_SET);
        if( (BKLT_TIM.Instance->CR1 & TIM_CR1_CEN) == 0 )
        	HAL_TIM_PWM_Start(PBKLT_TIM, BKLT_CHANNEL);
    }
}

static void USB_Power( USB_PWR_STATE state )
{
    if( state == USB_PWR_ENABLED )
        HAL_GPIO_WritePin( USB_EN_GPIO_Port, USB_EN_Pin, GPIO_PIN_SET );
    else
        HAL_GPIO_WritePin( USB_EN_GPIO_Port, USB_EN_Pin, GPIO_PIN_RESET );
}

static void Pi_Power( HOST_PWR_STATE state )
{
    if( state == HOST_PWR_ENABLED )
    {
    	HAL_GPIO_WritePin( PI_SHUTDOWN_GPIO_Port, PI_SHUTDOWN_Pin, GPIO_PIN_RESET );
        HAL_GPIO_WritePin( PI_PWR_EN_GPIO_Port, PI_PWR_EN_Pin, GPIO_PIN_SET );
        BITSET(system_flags, PI_PWR_EN);
    } else
    {
    	if( !(htim8.Instance->CR1 & TIM_CR1_CEN) )
    	{
    		LCD_Brightness( 0 );
        	HAL_GPIO_WritePin( PI_SHUTDOWN_GPIO_Port, PI_SHUTDOWN_Pin, GPIO_PIN_SET );
        	System_Power_Hold( SYS_PWR_HOLD_ENABLE );
			__HAL_TIM_CLEAR_FLAG( &htim8, TIM_IT_UPDATE );
			htim8.Instance->CNT = 0;
			HAL_TIM_Base_Start_IT( &htim8 );
    	}
    }

}

static void Get_SD_Card_State( void )
{
    if( HAL_GPIO_ReadPin( CARD_DETECT_SW_GPIO_Port, CARD_DETECT_SW_Pin ) == GPIO_PIN_RESET )
    {
        BITSET(system_flags, SD_CARD_PRESENT);
        dd_update_sd_card_state( SD_PRESENT );
    } else
    {
        BITCLEAR(system_flags, SD_CARD_PRESENT);
        dd_update_sd_card_state( SD_NOT_PRESENT );
    }
}

static void Fan_Control( FAN_PWR_STATE state )
{
    if( state == FAN_PWR_ENABLED )
    {
        #if FAN_PWM
        /* Increase the fan speed. TODO at switch case for fan speeds */
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
        htim2.Instance->CCR3 = htim2.Instance->ARR;

        #else
        /* Enable power to the Fan */
        HAL_GPIO_WritePin( FAN_EN_GPIO_Port, FAN_EN_Pin, GPIO_PIN_SET );
        #endif
    }
    else
    {
        #if FAN_PWM
        /* Keep the fan on, but at the lowest speed */
    	htim2.Instance->CCR3 = (htim2.Instance->ARR) / 7;
        #else
    	/* Disable power to the Fan */
        HAL_GPIO_WritePin( FAN_EN_GPIO_Port, FAN_EN_Pin, GPIO_PIN_RESET );
        #endif
    }
}

static uint8_t Rasp_Pi_Tx( uint8_t *data, uint8_t len )
{
	flash_led(DEBUG_LED_3);
    if( HAL_UART_Transmit_IT(PI_UART, data, len) == HAL_OK )
        return 1;
    else
        return 0;
}

static void Add_CAN_Filter( uint16_t id )
{
	if( CAN_Filter_Count < MAX_NUM_CAN_FILTERS )
	{
		if( can_filter( &hcan1, id, 0x7FF, CAN_ID_STD, CAN_Filter_Count++, CAN_FILTER_FIFO0 ) != HAL_OK )
			Error_Handler();
	}
}

static uint8_t ECU_CAN_Tx( uint8_t data[], uint8_t len )
{

	CAN_TxHeaderTypeDef Header = {
	           .DLC                = len,
	           .ExtId              = 0x7E0,
	           .StdId              = 0x7E0,
	           .IDE                = CAN_ID_STD,
	           .RTR                = CAN_RTR_DATA,
	           .TransmitGlobalTime = DISABLE
	};

    uint32_t pTxMailbox = 0;

    /* Copy the buffer */
    uint8_t tx_buf[OBDII_DLC];
    memcpy(tx_buf, data, OBDII_DLC);

    /* TODO check for free mailbox */
    if( HAL_CAN_AddTxMessage( ECU_CAN, &Header, tx_buf, &pTxMailbox ) == HAL_OK )
    {
    	/* Log which mailbox sent the packet */
    	can_tx_mailbox_status |= pTxMailbox;
        return 1;
    }
    else
    {
    	flash_led(DEBUG_LED_2);
        return 0;
    }
}

void Digitaldash_Init( void )
{
    DIGITALDASH_CONFIG config;
    config.dd_ecu_tx            = &ECU_CAN_Tx;
    config.dd_fan_ctrl          = &Fan_Control;
    config.dd_get_sd_card_state = &Get_SD_Card_State;
    config.dd_host_ctrl         = &Pi_Power;
    config.dd_ke_tx             = &Rasp_Pi_Tx;
    config.dd_set_backlight     = &LCD_Brightness;
    config.dd_usb               = &USB_Power;
    config.dd_filter            = &Add_CAN_Filter;

    if( digitaldash_init( &config ) != DIGITALDASH_INIT_OK )
        Error_Handler();
}

#define TEMP_SENSOR_AVG_SLOPE_MV_PER_CELSIUS                        2.5f
#define TEMP_SENSOR_VOLTAGE_MV_AT_25                                760.0f
#define ADC_REFERENCE_VOLTAGE_MV                                    3300.0f
#define ADC_MAX_OUTPUT_VALUE                                        4095.0f
#define TEMP110_CAL_VALUE                                           ((uint16_t*)((uint32_t)0x1FFF7A2E))
#define TEMP30_CAL_VALUE                                            ((uint16_t*)((uint32_t)0x1FFF7A2C))
#define TEMP110                                                     110.0f
#define TEMP30                                                      30.0f

int32_t get_mcu_internal_temp( void )
{
		int32_t temperature = 0;
		float sensorValue = 0;


		HAL_ADC_Start(&hadc1);
		if(HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
		{
		  sensorValue = (float)HAL_ADC_GetValue(&hadc1);
		  HAL_ADC_Stop(&hadc1);
		  temperature = (int32_t)((TEMP110 - TEMP30) / ((float)(*TEMP110_CAL_VALUE) - (float)(*TEMP30_CAL_VALUE)) * (sensorValue - (float)(*TEMP30_CAL_VALUE)) + TEMP30);
		}
		else
		{
		  temperature = -273;
		}

		return temperature;
}

void HAL_TIM_PeriodElapsedCallback( TIM_HandleTypeDef *htim )
{
	if( htim == &htim8)
	{
		System_Power_Hold( SYS_PWR_HOLD_DISABLE );
        HAL_GPIO_WritePin( PI_PWR_EN_GPIO_Port, PI_PWR_EN_Pin, GPIO_PIN_RESET );
        BITCLEAR(system_flags, PI_PWR_EN);
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
  MX_CAN1_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_TIM13_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start( &htim2, TIM_CHANNEL_3 );
  HAL_GPIO_WritePin( FAN_EN_GPIO_Port, FAN_EN_Pin, GPIO_PIN_RESET );

  /* Prevent unwanted power-down */
  System_Power_Hold( SYS_PWR_HOLD_DISABLE );

  //Fan_Control( FAN_PWR_ENABLED );

  if( get_mcu_internal_temp() > MCU_THERMAL_THRESH )
  {
	  Fan_Control( FAN_PWR_ENABLED );

	  while( get_mcu_internal_temp() > MCU_THERMAL_THRESH ) { }
  }

#if ENABLE_ALL_CAN_MSG
  for(uint8_t filter_bank = 0; filter_bank < STM32F446_NUM_CAN1_FILTERBANK; filter_bank++)
  {
      if( can_filter( &hcan1, 0x0000, 0x0000, CAN_ID_STD, 0, CAN_FILTER_FIFO0 ) != HAL_OK )
          Error_Handler();
  }
#else
      Add_CAN_Filter( 0x7E8 );
#endif

  if( HAL_CAN_Start( &hcan1 ) != HAL_OK )
      Error_Handler();

#if CAN_IT
  if( HAL_CAN_ActivateNotification( &hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK )
      Error_Handler();
#endif

  Digitaldash_Init();


#if LCD_ALWAYS_ON
  LCD_Brightness( 255 );
#endif

  /* Configure the UART intetupt */
  HAL_UART_Receive_IT( PI_UART, &rx_byte, 1 );

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#if !CAN_IT
    if( HAL_CAN_GetRxFifoFillLevel( ECU_CAN, CAN_RX_FIFO0 ) > 0 )
        process_can_packet( ECU_CAN, CAN_RX_FIFO0 );
    else if( HAL_CAN_GetRxFifoFillLevel( ECU_CAN, CAN_RX_FIFO1 ) > 0 )
        process_can_packet( ECU_CAN, CAN_RX_FIFO1 );
#endif

    digitaldash_service();

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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if( huart == PI_UART )
	{
		/* Pass the UART byte to the Digital Dash */
		DigitalDash_Add_UART_byte( rx_byte );

		/* Wait for the next byte */
		HAL_UART_Receive_IT( PI_UART, &rx_byte, 1 );
	}
}

uint32_t HAL_GetTick(void)
{
  return Sys_Tick;
}

void HAL_IncTick(void)
{
	Sys_Tick++;

    digitaldash_tick();

    if( LED1_Tick == 0 )
        HAL_GPIO_WritePin( DEBUG_LED_1_GPIO_Port, DEBUG_LED_1_Pin, GPIO_PIN_RESET );
    else
        LED1_Tick--;

    if( LED2_Tick == 0 )
        HAL_GPIO_WritePin( DEBUG_LED_2_GPIO_Port, DEBUG_LED_2_Pin, GPIO_PIN_RESET );
    else
        LED2_Tick--;

    if( LED3_Tick == 0 )
        HAL_GPIO_WritePin( DEBUG_LED_3_GPIO_Port, DEBUG_LED_3_Pin, GPIO_PIN_RESET );
    else
        LED3_Tick--;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

    while( 1 )
    {
        HAL_GPIO_TogglePin( DEBUG_LED_1_GPIO_Port, DEBUG_LED_1_Pin );
        LED1_Tick = 60;
        while( LED1_Tick != 0 ) { /* Blocking */ }
    }

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
