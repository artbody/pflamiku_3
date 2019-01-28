/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2019 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/**
 * AUTHORS: ACHIM MERATH
 * LICENSE: CC BY SA NC 4.0
 * Last change 2019-01-26
 *
 * Brief   This software is for measure 3 or 4 weight scales.
 * 			Depended on the min and max programmed weight of the plants
 * 			it starts a pump which is pumping water to the plant until it's
 * 			max weight value is reached.
 * 			The min and max values can be programmed very easy by one switch.
 * 			Put a plant with dry plant soil on the device. Put the switch in prog position.
 * 			The pump automatically begins to pump water to your plant.
 * 			If the  potting soil is wet enough put the switch in position run.
 * 			From now on the device waters your plant automatically between these two
 * 			programmed min and max values. Each time the weight goes under the minimum
 * 			the pump runs until the maximum or a timeout is reached.
 * 			The timeout value is that time doubled, that the device needed
 * 			while you programmed it. This is thought as a security feature.
 * 			In example if the water storage tank is empty
 *          Another feature is that the device is going in deep sleep mode if
 *          there is not enough ambient light. Normally in the night.
 *          That switch value can be programmed as well.
 * 			Therefore you've to wait for the according twilight condition.
 * 			Disconnect the power plug, put the switch in PROG mode, connect
 * 			the power plug again. Wait until the red LED blinks. Then put the switch
 * 			in normal run mode.
 * 			That's it.
 * 			Now the device doesn't start until it is brighter then the twilight value.
 * 			Another automatic feature is the brightness of the lED. It is adjusted automatically
 * 			in respect to the ambient light.
 *
 */
//#define test
//#define test_LDR
//#define test_HX712	// water below min
//#define test_HX712_w0min // water below min
//#define test_HX712_w1min // water above min
//#define test_HX712_w2max // water above max
//#define test_eeprom_write_firstTimeValues
//#define test_pump
#include "stdint.h"
#include <stdio.h>
#include <stdbool.h>
#include "fsmDefs.h"

/** MainFsm function definitions */
#include "MainFsm.h"

/** FW Profile function definitions */
#include "FwSmConstants.h"
#include "FwSmSCreate.h"
#include "FwSmConfig.h"
#include "FwSmCore.h"

#include "sc_types.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//typedef struct is in fsmDefs.h
FsmIFace iface;
FsmIFace* piface = &iface;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

IWDG_HandleTypeDef hiwdg;

LPTIM_HandleTypeDef hlptim1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim21;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* NVM key definitions */

/*
 #define FLASH_PDKEY1               ((uint32_t)0x04152637) !< Flash power down key1
 #define FLASH_PDKEY2               ((uint32_t)0xFAFBFCFD) !< Flash power down key2: used with FLASH_PDKEY1 to unlock the RUN_PD bit in FLASH_ACR

 #define FLASH_PEKEY1               ((uint32_t)0x89ABCDEF) !< Flash program erase key1
 #define FLASH_PEKEY2               ((uint32_t)0x02030405) !< Flash program erase key: used with FLASH_PEKEY2
 to unlock the write access to the FLASH_PECR register and data EEPROM

 #define FLASH_PRGKEY1              ((uint32_t)0x8C9DAEBF) !< Flash program memory key1
 #define FLASH_PRGKEY2              ((uint32_t)0x13141516) !< Flash program memory key2: used with FLASH_PRGKEY2
 to unlock the program memory

 #define FLASH_OPTKEY1              ((uint32_t)0xFBEAD9C8) !< Flash option key1
 #define FLASH_OPTKEY2              ((uint32_t)0x24252627) !< Flash option key2: used with FLASH_OPTKEY1 to
 unlock the write access to the option byte block

 Time-out values
 #define HSI_TIMEOUT_VALUE          ((uint32_t)100)   100 ms
 #define PLL_TIMEOUT_VALUE          ((uint32_t)100)   100 ms
 #define CLOCKSWITCH_TIMEOUT_VALUE  ((uint32_t)5000)  5 s
 */

#define DATA_E2_ADDR   ((uint32_t)0x08080000)  /* Data EEPROM address */

/* Error codes used to make the red led blinking */
#define ERROR_ERASE 0x01
#define ERROR_PROG_BYTE  0x02
#define ERROR_PROG_16B_WORD 0x04
#define ERROR_PROG_32B_WORD 0x08
#define ERROR_WRITE_PROTECTION 0x10
#define ERROR_READOUT_PROTECTION 0x20
#define ERROR_FETCH_DURING_PROG 0x40
#define ERROR_SIZE 0x80
#define ERROR_ALIGNMENT 0x100
#define ERROR_NOT_ZERO 0x200
#define ERROR_OPTION_NOT_VALID 0x400
#define ERROR_UNKNOWN 0x800

#define ERROR_HSI_TIMEOUT 0x55
#define ERROR_PLL_TIMEOUT 0xAA
#define ERROR_CLKSWITCH_TIMEOUT 0xBB

volatile uint16_t error = 0;

/* Private variables -----------------------------------------------FSM----*/
//special for testing
#if defined test
uint32_t TEST_LDR_VALUE = 200;
uint32_t TEST_LDR_SWITCH = 400;

uint32_t TEST_MITTELWERT = 8001271;
#endif

volatile int progMode = 0, *pprogMode = &progMode;
volatile uint32_t millis; //,*pmillis=&millis; used for pump timeout
volatile uint32_t blink_millis; // used for blinking while pump on

/* Private variables ---------------------------------------------HX712 + DIV------*/

//uint32_t HX_cnt; //, hx1, hx2, hx3, hx4;
uint32_t buffer_hx[25]; // used in IRQ routine to get measurement values from the HX711 / HX712
volatile uint32_t HXSCK_state = 1, *pHXSCK_state = &HXSCK_state; // used in IRQ routine to set the state
volatile uint32_t hxerrors[] = { 0, 0, 0, 0 }; // used in IRQ routine to indicate errors
volatile uint32_t HX_cnt = 0, *pHX_cnt = &HX_cnt; // used in IRQ routine to get measurement values from the HX711 / HX712
volatile uint32_t hx, hx1, hx2, hx3, hx4, *phx = &hx, *phx1 = &hx1,
		*phx2 = &hx2, *phx3 = &hx3, *phx4 = &hx4; // used in IRQ routine to get measurement values from the HX711 / HX712
volatile uint32_t Messwert_long_time_average = 0, *pMesswert_long_time_average =
		&Messwert_long_time_average; // the measurement average value

uint32_t hxmax = 12000000;
uint32_t hxmin = 8001271; //8000000;

//LDR Value
uint32_t adc[4], adc_buf[2], temperature, vrefint;  // define variables
volatile uint32_t g_ADCValue = 0, *pLDR = &g_ADCValue;

//led color
volatile int ar = 0;
volatile int ag = 0;
volatile int ab = 0;
volatile float hue_angel = 360;
/*has Max Luminosity (RGB): (1000, 1300, 800)mcd
 so we normalize them all to 800 mcd -
 R  800/1000  =  0.8  =   204/255					52428/65535 3276/4095
 G  800/1300  =  0.61538   =    157/255           	40329/65535 2520/4095
 B  800/800  =  1.0                 =   255/255
 // for all three RGB values
 analogWrite(PIN, rgb[k] * bright[k]/255);
 */
//long bright[3] = {
//		3576, 3020, 4095
//};
uint32_t bright[3] = { 3076, 2520, 4095 };  // this are the default values
uint32_t bright_ad[3] = { 3076, 2520, 4095 }; // this are the calculated values according to ambient light
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM2_Init(void);
static void MX_RTC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM21_Init(void);
static void MX_LPTIM1_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void fsm_init(FsmIFace* iface);

void FsmSetVars(void); //set start conditions of the fsm
void TEST_if_first_start_then_eeprom_set_values(void); //setings like min max weight and ldr min max and switch
uint32_t shift_test_switch_is_ON(void);
void test_if_switch_is_on(void);
//hx712 measurement
void HX712_start(void);
void HX712_stop(void);
void HX712_run(void);
// pump run
void pumpOFF(void);
void pumpON(void);
void sleepMode(void);
void progLdrSwitchValue(void);
// ldr measurement
void RUN_prog_ldr(void);
void LDR_Value(void);

//LED settings with RGB and PWM
void LED_RGB_Set(float HSV_value);

// this  function is for mapping a value from one area to another
// int map(int x, int in_min, int in_max, int out_min, int out_max);
uint32_t map2(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min,
		uint32_t out_max);
// Eeprom functions
void Save_2_Eeprom(void);
void Read_from_Eeprom(void);

void UnlockPELOCK(void);
void LockNVM(void);
void EepromErase(uint32_t addr);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_DMA_Init();
	MX_ADC_Init();
	MX_IWDG_Init();
	MX_TIM2_Init();
	MX_RTC_Init();
	MX_USART2_UART_Init();
	MX_TIM21_Init();
	MX_LPTIM1_Init();
	/* USER CODE BEGIN 2 */
	// TIM2 is used for RGB LED
	// HAL_TIM_Base_Start_IT(&hlptim1);
	HAL_TIM_Base_Start(&htim2); //Starts the TIM Base generation
	if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) //Starts the PWM signal generation
			{
		/* PWM Generation Error */
		Error_Handler();
	}

	/* Start channel 2 */
	if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK) {
		/* PWM Generation Error */
		Error_Handler();
	}
	if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK) {
		/* PWM Generation Error */
		Error_Handler();
	}

	//fsm_init(FsmIFace* iface) init the vars of the finite state machine
	fsm_init(&iface);

	//look if switch is in progMode
	piface->sw1_value = shift_test_switch_is_ON();

	// read all volues from eeprom and put them in place THIS is now in fsm_init() the vars of the finite state machine
	Read_from_Eeprom();
	HX712_run();
	LDR_Value();

	//this is called to preinit the microcontroller only at first start up
	TEST_if_first_start_then_eeprom_set_values();

	uint32_t tickstart = 0U, *ptickstart = &tickstart;
	*ptickstart = HAL_GetTick();

	/*------------------------------------------fsm init & start-------------------------------------*/
	/** Define the state machine descriptor (SMD) */
	FwSmDesc_t smDesc = MainFsmCreate(NULL);

	/** Check that the SM is properly configured */
	if (FwSmCheckRec(smDesc) != smSuccess) {
		printf(
				"The state machine MainFsm is NOT properly configured ... FAILURE\n");
		//return EXIT_FAILURE;
	}

	printf("The state machine MainFsm is properly configured ... SUCCESS\n");
	/** Start the SM, send a few transition commands to it, and execute it */
	FwSmStart(smDesc);
	FwSmMakeTrans(smDesc, TCLK);
	/*-------------------------------------------------------------------------------------------------*/
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {
		__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
		FwSmMakeTrans(smDesc, TCLK);
		// this is needed to jump out of substates
		if (piface->run_mode == 3) {
			FwSmMakeTrans(smDesc, TCLK);
			FwSmMakeTrans(smDesc, TCLAK);
			piface->run_mode = 1;
		}

		test_if_switch_is_on();

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		//TODO power reduction
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/**Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
	RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2
			| RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_LPTIM1;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	PeriphClkInit.LptimClockSelection = RCC_LPTIM1CLKSOURCE_LSI;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void) {

	/* USER CODE BEGIN ADC_Init 0 */

	/* USER CODE END ADC_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC_Init 1 */

	/* USER CODE END ADC_Init 1 */
	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc.Instance = ADC1;
	hadc.Init.OversamplingMode = DISABLE;
	hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ContinuousConvMode = DISABLE;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.DMAContinuousRequests = DISABLE;
	hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerFrequencyMode = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	if (HAL_ADC_Init(&hadc) != HAL_OK) {
		Error_Handler();
	}
	/**Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC_Init 2 */

	/* USER CODE END ADC_Init 2 */

}

/**
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void) {

	/* USER CODE BEGIN IWDG_Init 0 */

	/* USER CODE END IWDG_Init 0 */

	/* USER CODE BEGIN IWDG_Init 1 */

	/* USER CODE END IWDG_Init 1 */
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
	hiwdg.Init.Window = 4095;
	hiwdg.Init.Reload = 4095;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN IWDG_Init 2 */

	/* USER CODE END IWDG_Init 2 */

}

/**
 * @brief LPTIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_LPTIM1_Init(void) {

	/* USER CODE BEGIN LPTIM1_Init 0 */

	/* USER CODE END LPTIM1_Init 0 */

	/* USER CODE BEGIN LPTIM1_Init 1 */

	/* USER CODE END LPTIM1_Init 1 */
	hlptim1.Instance = LPTIM1;
	hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
	hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV128;
	hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
	hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
	hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
	hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
	if (HAL_LPTIM_Init(&hlptim1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN LPTIM1_Init 2 */

	/* USER CODE END LPTIM1_Init 2 */

}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void) {

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	RTC_TimeTypeDef sTime = { 0 };
	RTC_DateTypeDef sDate = { 0 };

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */
	/**Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK) {
		Error_Handler();
	}

	/* USER CODE BEGIN Check_RTC_BKUP */

	/* USER CODE END Check_RTC_BKUP */

	/**Initialize RTC and set the Time and Date
	 */
	sTime.Hours = 0x0;
	sTime.Minutes = 0x0;
	sTime.Seconds = 0x0;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) {
		Error_Handler();
	}
	sDate.WeekDay = RTC_WEEKDAY_MONDAY;
	sDate.Month = RTC_MONTH_JANUARY;
	sDate.Date = 0x1;
	sDate.Year = 0x0;

	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 15;
	htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
	htim2.Init.Period = 4095;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.Pulse = 2;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.Pulse = 3;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM21 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM21_Init(void) {

	/* USER CODE BEGIN TIM21_Init 0 */

	/* USER CODE END TIM21_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM21_Init 1 */

	/* USER CODE END TIM21_Init 1 */
	htim21.Instance = TIM21;
	htim21.Init.Prescaler = 32;
	htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim21.Init.Period = 6;
	htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim21.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim21) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM21_Init 2 */

	/* USER CODE END TIM21_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE()
	;

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, mot_out_Pin | HXSCK_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : mot_out_Pin HXSCK_Pin */
	GPIO_InitStruct.Pin = mot_out_Pin | HXSCK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : sw2_Pin sw1_Pin HXD1_Pin HXD2_Pin
	 HXD3_Pin HXD4_Pin */
	GPIO_InitStruct.Pin = sw2_Pin | sw1_Pin | HXD1_Pin | HXD2_Pin | HXD3_Pin
			| HXD4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : LD3_Pin */
	GPIO_InitStruct.Pin = LD3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void fsm_init(FsmIFace* iface) {

	iface->switcher_raised = 0;
	iface->switcher_value = 0;
	iface->started = 0;
	iface->prog_mode = 0;
	iface->bPumpOn = 0;
	iface->bPumpEn = 0;

	iface->lowPower = 0;
	iface->sw1 = 0;
	iface->eepromAbsMax = 12000000;
	iface->eepromMin = *(uint32_t *) (DATA_E2_ADDR);
	iface->eepromMax = *(uint32_t *) (DATA_E2_ADDR + 4);
	iface->eepromPmax = *(uint32_t *) (DATA_E2_ADDR + 8);
	iface->mittelwert = 8001271;
	iface->hx = 0;
	iface->timeOut = 0;

	iface->startTimLDR = 60000;
	iface->timeLDR = 60000;
	iface->startTimHx = 0;
	iface->timeHx = 10000;

	iface->ldr_min = *(uint32_t *) (DATA_E2_ADDR + 20);
	iface->ldr_max = *(uint32_t *) (DATA_E2_ADDR + 24);
	iface->ldr_switch = *(uint32_t *) (DATA_E2_ADDR + 28);
	iface->run_mode = 0;

	iface->menuFsmExitState = 0;
	//char tmp[32];
}

/**
 * Brief   This function runs only once if there is no data in the eeprom .
 *
 * Param   None
 * Retval  None
 */
void TEST_if_first_start_then_eeprom_set_values(void) {
	if ((*(uint32_t *) (DATA_E2_ADDR + 4) == 0)
			|| (*(uint32_t *) (DATA_E2_ADDR + 4)
					== *(uint32_t *) (DATA_E2_ADDR + 16))) {
		// value to write in the eeprom min 8200000 max 12000000 ca 11kg
		piface->eepromMin = hxmin;
		piface->eepromMax = hxmax;
		Save_2_Eeprom();
		RUN_prog_ldr();
	}
#if defined test_eeprom_write_firstTimeValues
	eeprom_value_min = hxmin;
	eeprom_value_max = hxmax;
	Save_2_Eeprom();
	RUN_prog_ldr();
#endif
}

/**
 * Brief   This function debounces SW1 .
 *
 * Param   None
 * Retval  uint32_t
 */
uint32_t shift_test_switch_is_ON(void) {
	volatile uint32_t sw_stable;
	volatile uint32_t tout;
	volatile uint8_t p1;

	sw_stable = 0;
	tout = 0;
	do {
		for (int i = 0; i < 32; i++) {
			p1 = 10;
			if ((sw1_GPIO_Port->IDR & sw1_Pin) != (uint32_t) GPIO_PIN_RESET) {
				p1 = GPIO_PIN_SET;
			} else {
				p1 = GPIO_PIN_RESET;
			}
			HAL_Delay(1);
			sw_stable = (sw_stable << 1) + p1;
		}
		tout++;
	} while ((sw_stable < 0xffffffff && sw_stable > 0) || tout >= 3);
	if (sw_stable == 0xffffffff) {
		p1 = 1;
	} else {
		p1 = 0;
	}
	return p1;
}

/**
 * Brief   This function tests the SW1 if Progmode.
 *
 * Param   None
 * Retval  None
 */
void test_if_switch_is_on(void) {

	uint32_t progMode_ticker;
	progMode_ticker = shift_test_switch_is_ON();
	if (progMode_ticker == 1) {
		if (HAL_GPIO_ReadPin(sw1_GPIO_Port, sw1_Pin) == 1 && *pprogMode == 0) {

			progMode_ticker = HAL_GetTick();
			do {
				//debounce 50ms
			} while ((HAL_GetTick() - progMode_ticker) < 50);
			if (HAL_GPIO_ReadPin(sw1_GPIO_Port, sw1_Pin) == 1
					&& *pprogMode == 0) {
				//RUN_progmode
				*pprogMode = 1;

			}
		}
	} else {
		if (HAL_GPIO_ReadPin(sw1_GPIO_Port, sw1_Pin) == 0 && *pprogMode == 1) {
			progMode_ticker = HAL_GetTick();
			do {
				//debounce 50ms
			} while ((HAL_GetTick() - progMode_ticker) < 50);
			if (HAL_GPIO_ReadPin(sw1_GPIO_Port, sw1_Pin) == 0
					&& *pprogMode == 1) {
				//STOP_progmode
				*pprogMode = 0;

			}
		}
	}
}

/**
 * Brief   This function starts the HX711. This chip was in sleepmode with a few uA.
 *
 * Param   None
 * Retval  None
 */
void HX712_start(void) {
	HAL_GPIO_WritePin(HXSCK_GPIO_Port, HXSCK_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(HXSCK_GPIO_Port, HXSCK_Pin, GPIO_PIN_RESET);
	HAL_Delay(600);

}

/**
 * Brief   This function stops the HX711. This chip enters after 50us the sleepmode with a few uA.
 *
 * Param   None
 * Retval  None
 */
void HX712_stop(void) {
	HAL_GPIO_WritePin(HXSCK_GPIO_Port, HXSCK_Pin, GPIO_PIN_SET);
}

/**
 * Brief   This function is for getting the measured data from the HX711 / HX712 serial interface - weight cells .
 * 			this needs 24 pulses to get 24bit Data and 1 pulse to set the right GAIN of the Chips next measurement
 * 			to run this measurement we must first call the HX_start() ..
 * 			in the TIM21_IRQHandler we do the pulsing and read the hole port 25 times in a buffer. so therefore HXD1-HXD3/4
 * 			must be on the same Portgroupe i.e. GPIOA.
 * 			after the serial read out we calculate by bitoperation the real result of all three or four weight cells.
 *
 * Param   None
 * Retval  None
 */
void HX712_run(void) {
	HX712_start();
	uint16_t bm = HXD1_Pin + HXD2_Pin + HXD3_Pin + HXD4_Pin;
	uint16_t g;	// = HXD1_GPIO_Port->IDR;
	int d;	// = g & bm;
	//set vars to zero
	*pHX_cnt = 0;
	*phx1 = 0;
	*phx2 = 0;
	*phx3 = 0;
	*phx4 = 0;
	*pHXSCK_state = 0;
	// iwdg reset
	IWDG->KR = IWDG_KEY_RELOAD;
	// run the measurement
	int t;
	do {
		do {
			IWDG->KR = IWDG_KEY_RELOAD;
			g = HXD1_GPIO_Port->IDR;
			d = g & bm;
		} while (d > 0);

		HAL_TIM_Base_Start_IT(&htim21);
		while (*pHX_cnt < 52) {

		}
		//HAL_GPIO_WritePin(HXSCK_GPIO_Port, HXSCK_Pin, GPIO_PIN_RESET);
		//*pHXSCK_state = 0;
		hx1 = 0;
		hx2 = 0;
		hx3 = 0;
		IWDG->KR = IWDG_KEY_RELOAD;
		//hx4 = 0;
		// bitbanging for results
		for (int i = 0; i < 24; i++) {
			hx1 = hx1 << 1;
			hx2 = hx2 << 1;
			hx3 = hx3 << 1;
			//hx4 = hx4 << 1;

			if (buffer_hx[i] & HXD1_Pin) {
				hx1++;
			}
			if (buffer_hx[i] & HXD2_Pin) {
				hx2++;
			}
			if (buffer_hx[i] & HXD3_Pin) {
				hx3++;
			}
//		if (buffer_hx[i] & HXD4_Pin) {
//			hx4++;
//		}
		}
		*phx1 = *phx1 ^ 0x800000;
		*phx2 = *phx2 ^ 0x800000;
		*phx3 = *phx3 ^ 0x800000;
		//*phx4 = *phx4 ^ 0x800000;

		//error analyser
		t = 3;
		if (*phx1 > hxmax) {
			t--;
			*phx1 = 0;
			hxerrors[0]++;
		} else if (*phx2 > hxmax) {
			t--;
			*phx2 = 0;
			hxerrors[1]++;
		} else if (*phx3 > hxmax) {
			t--;
			*phx3 = 0;
			hxerrors[2]++;

		}
		hxerrors[3]++;
	} while (t < 3); // so we can be shure all 3 or 4 HX712 are ok

	// long time average calculation
	*phx = (*phx1 + *phx2 + *phx3) / t;

	if (*pMesswert_long_time_average == 0) {
		*pMesswert_long_time_average = *phx;
	}

	*pMesswert_long_time_average = (*pMesswert_long_time_average * 31 + *phx)
			/ 32;
	piface->mittelwert = *pMesswert_long_time_average;
	piface->hx = *phx;

	HX712_stop();
}

void pumpOFF(void) {
	*pMesswert_long_time_average = piface->hx = *phx;
	HAL_GPIO_WritePin(mot_out_GPIO_Port, mot_out_Pin, GPIO_PIN_RESET);
}
void pumpON(void) {
	HAL_GPIO_WritePin(mot_out_GPIO_Port, mot_out_Pin, GPIO_PIN_SET);
}
void sleepMode(void) {
	//assume its night and so do nothing, switch off RGB_LED and go in standby mode

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	//standbymode
	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	SysTick->CTRL = 0;
	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0xFFFF, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
	HAL_PWR_EnterSTANDBYMode();
}
void progLdrSwitchValue(void) {
	// to prog LDR_switch value
	// test if sw1 == 1
	uint32_t progMode_ticker = 0;
	uint32_t LDR_ticker = 0;
	if (shift_test_switch_is_ON() == 1) {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, bright[0]); // switch to red led
		LDR_ticker = HAL_GetTick();
		HAL_Delay(600);
		if (shift_test_switch_is_ON() == 1) {
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0); // switch red led off
			LDR_ticker = HAL_GetTick();
			HAL_Delay(100);
			LDR_Value();
			LDR_ticker = HAL_GetTick();
			//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, bright[0]); // switch to red led
			//do wait for 240 sec so the user can put switch off to set LDR_switch value
			int waitms = 240000;
			while ((HAL_GetTick() - LDR_ticker) < waitms) {

				__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, bright[1]); // switch to green led
				HAL_Delay(500);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); // switch green led off
				HAL_Delay(500);
				__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
				if (shift_test_switch_is_ON() == 0) {
					LDR_Value();
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, bright[1]); // switch to green led
					//save new Values to eeprom
					piface->ldr_switch = *pLDR;
					RUN_prog_ldr();
					LDR_ticker += waitms;
				}

			}
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); // switch green led off
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, bright[0]); // switch to red led
			while (1) {
				// run in watchdogtimer time out
			}
		}
	}
}
/**
 * Brief   This function saves the LDR min max and switch values in the eeprom.
 *
 * Param   None
 * Retval  None
 */
void RUN_prog_ldr(void) {
	__HAL_IWDG_RELOAD_COUNTER(&hiwdg);

	error = 0;
	UnlockPELOCK();
	FLASH->PECR |= FLASH_PECR_ERRIE | FLASH_PECR_EOPIE; /* enable flash interrupts */

	*(uint32_t *) (DATA_E2_ADDR + 20) = piface->ldr_min; /* (1) */
	__WFI();
	if (*(uint32_t *) (DATA_E2_ADDR + 20) != piface->ldr_min) {
		error |= ERROR_PROG_32B_WORD;
	}

	*(uint32_t *) (DATA_E2_ADDR + 24) = piface->ldr_max; /* (1) */
	__WFI();
	if (*(uint32_t *) (DATA_E2_ADDR + 24) != piface->ldr_max) {
		error |= ERROR_PROG_32B_WORD;
	}

	*(uint32_t *) (DATA_E2_ADDR + 28) = piface->ldr_switch; /* (1) */
	__WFI();
	if (*(uint32_t *) (DATA_E2_ADDR + 28) != piface->ldr_switch) {
		error |= ERROR_PROG_32B_WORD;
	}

	LockNVM();
}

/**
 * Brief   This function measures the brightness of the ambient light with an LDR.
 *         and calculates the max brightness values of the RGB LED
 * Param   None
 * Retval  None
 */
void LDR_Value(void) {
	uint32_t tg_ADCValue = *pLDR;

// first switch off the LED so we can be sure we measure the daylight

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	HAL_Delay(100);	//wait a few ms for LDR is ready

	//measurement LDR
	adc_buf[0] = 0;
	//int tistart=TIM22->CNT;
	HAL_ADC_Start_DMA(&hadc, (uint32_t *) adc_buf, 1);
	HAL_Delay(1);
	do {
		IWDG->KR = IWDG_KEY_RELOAD;
	} while (adc_buf[0] == 0);
	//int tistop=TIM22->CNT;
	//adc_buf[1]=tistop-tistart;ca 59ms
	/* Here      HAL_ADC_Start_DMA takes following arguments
	 ADC_HandleTypeDef* hadc --->   ADC handler
	 uint32_t* pData  --->   The destination Buffer address
	 uint32_t Length  --->  The length of data to be transferred from ADC peripheral to memory.*/
	//HAL_Delay(400);
	// reload old LED global settings of PWM because next steps are too long to wait
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ar);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ag);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ab);

	if (tg_ADCValue == 0) {
		tg_ADCValue = adc_buf[0];
	}
	*pLDR = ((tg_ADCValue * 7) + adc_buf[0]) / 8;
	HAL_ADC_Stop_DMA(&hadc);
	piface->ldr_value = ((tg_ADCValue * 7) + adc_buf[0]) / 8;
	// this part calculates the brightness according to the ambientlight
	// the darker it is the less the brightness of the LED
	//map(value, fromLow, fromHigh, toLow, toHigh)
	// BUG if toLow is greater then 0 this gives a wrong calculation
	//LDRvalue_min  LDRvalue_max LDR_switch
	if ((piface->ldr_switch > *pLDR)
			&& (piface->ldr_switch - *pLDR) > piface->ldr_min) {
		bright_ad[0] = map2((piface->ldr_switch - *pLDR), piface->ldr_min,
				piface->ldr_switch, 100, bright[0]);
		bright_ad[1] = map2((piface->ldr_switch - *pLDR), piface->ldr_min,
				piface->ldr_switch, 100, bright[1]);
		bright_ad[2] = map2((piface->ldr_switch - *pLDR), piface->ldr_min,
				piface->ldr_switch, 100, bright[2]);
	} else {
		bright_ad[0] = 0;
		bright_ad[1] = 0;
		bright_ad[2] = 0;
	}
	// recalculate LED values
	if (piface->started == 1) {
		LED_RGB_Set(1.0);
	}
	// reload new LED global settings of PWM
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ar);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ag);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ab);
}

/**
 * Brief   This function saves all data in the eeprom.
 *
 * Param   None
 * Retval  None
 */
void Save_2_Eeprom(void) {
	//eeprom_value_min
	//eeprom_value_max
	__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
	error = 0;
	UnlockPELOCK();
	FLASH->PECR |= FLASH_PECR_ERRIE | FLASH_PECR_EOPIE; /* enable flash interrupts */

	*(uint32_t *) (DATA_E2_ADDR) = piface->eepromMin; /* (1) */
	__WFI();
	if (*(uint32_t *) (DATA_E2_ADDR) != piface->eepromMin) {
		error |= ERROR_PROG_32B_WORD;
	}

	*(uint32_t *) (DATA_E2_ADDR + 4) = piface->eepromMax; /* (1) */
	__WFI();
	if (*(uint32_t *) (DATA_E2_ADDR + 4) != piface->eepromMax) {
		error |= ERROR_PROG_32B_WORD;
	}

	*(uint32_t *) (DATA_E2_ADDR + 8) = piface->eepromPmax; /* (1) */
	__WFI();
	if (*(uint32_t *) (DATA_E2_ADDR + 8) != piface->eepromPmax) {
		error |= ERROR_PROG_32B_WORD;
	}

	LockNVM();
}

/**
 * Brief   This function reads all data from the eeprom.
 *
 * Param   None
 * Retval  None
 */
void Read_from_Eeprom(void) {
	piface->eepromMin = *(uint32_t *) (DATA_E2_ADDR);
	piface->eepromMax = *(uint32_t *) (DATA_E2_ADDR + 4);
	piface->eepromPmax = *(uint32_t *) (DATA_E2_ADDR + 8);
	//uint32_t *pData = 0x08080000;
	piface->ldr_min = *(uint32_t *) (DATA_E2_ADDR + 20);	//LDRvalue_min
	piface->ldr_max = *(uint32_t *) (DATA_E2_ADDR + 24);	//LDRvalue_max
	piface->ldr_switch = *(uint32_t *) (DATA_E2_ADDR + 28);	//LDR_switch
	//eeprom_value_max=
}

/**
 * Brief   This function handles the LED.
 *         it sets the color according to the long time average of the weight.
 * Param   Brightness float 0.0-1.0
 * Retval  None
 */
void LED_RGB_Set(float HSV_value) {
	uint32_t mittelwert = 0;
#if defined test_HX712
	//return mittelwert
	mittelwert = TEST_MITTELWERT;
#else
	mittelwert = *pMesswert_long_time_average;
#endif
	__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
	//map(value, fromLow, fromHigh, toLow, toHigh)
	if (mittelwert <= *(uint32_t *) (DATA_E2_ADDR)) {
		hue_angel = 1;
	} else if (mittelwert >= *(uint32_t *) (DATA_E2_ADDR + 4)) {
		hue_angel = 301;
	} else {
		hue_angel = map2(mittelwert, *(uint32_t *) (DATA_E2_ADDR),
				*(uint32_t *) (DATA_E2_ADDR + 4), 0, 300);
	}

	if (hue_angel <= 150) {
		ar = map2((150 - hue_angel), 0, 150, 0, bright_ad[0]);
		ag = map2(hue_angel, 0, 150, 0, bright_ad[1]);
		ab = 0;
	} else if (hue_angel <= 300) {
		ar = 0;
		//ag=(map((hue_angel-150),150,300,0,bright[1]))*-1;
		//ag = bright_ad[1] + (map2((300 - hue_angel), 150, 300, 0, bright_ad[1]));
		ag = map2((300 - hue_angel), 0, 300, 0, bright_ad[1]);
		ab = map2(hue_angel, 150, 300, 0, bright_ad[2]);
	} else {
		ar = 0;
		ag = 0;
		ab = bright_ad[2];

	}

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ar);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ag);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ab);

}

/**
 * Brief   This function handles a given value and calculates from one range int another
 *
 *         usage; map(value, fromLow, fromHigh, toLow, toHigh)
 * Param   value, fromLow, fromHigh, toLow, toHigh
 * Retval  int
 */
int map(int x, int in_min, int in_max, int out_min, int out_max) {
	return (((x - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min);

}
/**
 * Brief   This function handles a given value and calculates from one range int another
 * 			make sure there is no negative calculation
 *         usage; map(value, fromLow, fromHigh, toLow, toHigh)
 * Param   value, fromLow, fromHigh, toLow, toHigh
 * Retval  uint32_t
 */
uint32_t map2(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min,
		uint32_t out_max) {
	return (((x - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min);

}
__INLINE void UnlockPELOCK(void)

{

	/* (1) Wait till no operation is on going */

	/* (2) Check if the PELOCK is unlocked */

	/* (3) Perform unlock sequence */

	while ((FLASH->SR & FLASH_SR_BSY) != 0) /* (1) */

	{

		/* For robust implementation, add here time-out management */

	}

	if ((FLASH->PECR & FLASH_PECR_PELOCK) != 0) /* (2) */

	{

		FLASH->PEKEYR = FLASH_PEKEY1; /* (3) */

		FLASH->PEKEYR = FLASH_PEKEY2;

	}

}

/******************************************************************************/

/*                 STM32L0xx Peripherals Interrupt Handlers                   */

/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */

/*  available peripheral interrupt handler's name please refer to the startup */

/*  file (startup_stm32l0xx.s).                                               */

/******************************************************************************/

/**

 * Brief   This function handles FLASH interrupt request.

 *         It handles any kind of error even if not used in this example.

 * Param   None

 * Retval  None

 */

void FLASH_IRQHandler(void)

{

	if ((FLASH->SR & FLASH_SR_EOP) != 0) /* (3) */

	{

		FLASH->SR = FLASH_SR_EOP; /* (4) */

	}

	/* Manage the error cases */

	else if ((FLASH->SR & FLASH_SR_FWWERR) != 0) /* Check Fetch while Write error */

	{

		error |= ERROR_FETCH_DURING_PROG; /* Report the error to the main progran */

		FLASH->SR = FLASH_SR_FWWERR; /* Clear the flag by software by writing it at 1*/

	}

	else if ((FLASH->SR & FLASH_SR_NOTZEROERR) != 0) /* Check Not Zero error */

	/* This error occurs if the address content was not cleared/erased

	 before the programming */

	{

		error |= ERROR_NOT_ZERO; /* Report the error to the main progran */

		FLASH->SR = FLASH_SR_NOTZEROERR; /* Clear the flag by software by writing it at 1*/

	}

	else if ((FLASH->SR & FLASH_SR_SIZERR) != 0) /* Check Size error */

	{

		error |= ERROR_SIZE; /* Report the error to the main progran */

		FLASH->SR = FLASH_SR_SIZERR; /* Clear the flag by software by writing it at 1*/

	}

	else if ((FLASH->SR & FLASH_SR_WRPERR) != 0) /* Check Write protection error */

	{

		error |= ERROR_WRITE_PROTECTION; /* Report the error to the main progran */

		FLASH->SR = FLASH_SR_WRPERR; /* Clear the flag by software by writing it at 1*/

	}

	else if ((FLASH->SR & FLASH_SR_RDERR) != 0) /* Check Read-out protection error */

	{

		error |= ERROR_READOUT_PROTECTION; /* Report the error to the main progran */

		FLASH->SR = FLASH_SR_RDERR; /* Clear the flag by software by writing it at 1*/

	}

	else if ((FLASH->SR & FLASH_SR_OPTVERR) != 0) /* Check Option valid error */

	{

		error |= ERROR_OPTION_NOT_VALID; /* Report the error to the main progran */

		FLASH->SR = FLASH_SR_OPTVERR; /* Clear the flag by software by writing it at 1*/

	}

	else if ((FLASH->SR & FLASH_SR_PGAERR) != 0) /* Check alignment error */

	{

		error |= ERROR_ALIGNMENT; /* Report the error to the main progran */

		FLASH->SR = FLASH_SR_PGAERR; /* Clear the flag by software by writing it at 1*/

	}

	else

	{

		error |= ERROR_UNKNOWN;

	}

}

/**

 * Brief   This function locks the NVM.

 *         It first checks no flash operation is on going,

 *         then locks the flash.

 * Param   None

 * Retval  None

 */

__INLINE void LockNVM(void)

{

	/* (1) Wait till no operation is on going */

	/* (2) Locks the NVM by setting PELOCK in PECR */

	while ((FLASH->SR & FLASH_SR_BSY) != 0) /* (1) */

	{

		/* For robust implementation, add here time-out management */

	}

	FLASH->PECR |= FLASH_PECR_PELOCK; /* (2) */

}

/**

 * Brief   This function erases a word of data EEPROM.

 *         The ERASE bit and DATA bit are set in PECR at the beginning

 *         and reset at the endof the function. In case of successive erase,

 *         these two operations could be performed outside the function.

 *         The flash interrupts must have been enabled prior to call

 *         this function.

 * Param   addr is the 32-bt word address to erase

 * Retval  None

 */

__INLINE void EepromErase(uint32_t addr)

{

	/* (1) Set the ERASE and DATA bits in the FLASH_PECR register

	 to enable page erasing */

	/* (2) Write a 32-bit word value at the desired address

	 to start the erase sequence */

	/* (3) Enter in wait for interrupt. The EOP check is done in the Flash ISR */

	/* (6) Reset the ERASE and DATA bits in the FLASH_PECR register

	 to disable the page erase */

	FLASH->PECR |= FLASH_PECR_ERASE | FLASH_PECR_DATA; /* (1) */

	*(__IO uint32_t *) addr = (uint32_t) 0; /* (2) */

	__WFI(); /* (3) */

	FLASH->PECR &= ~(FLASH_PECR_ERASE | FLASH_PECR_DATA); /* (4) */

}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	while (1) {
		uint32_t ERROR_ticker;
		while (1) {
			//  blink red
			//water level low or pump defect
			__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, bright[0]); // switch to red led
			ERROR_ticker = HAL_GetTick();
			while ((HAL_GetTick() - ERROR_ticker) < 300) {
				//wait 600 ms
			}
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0); // switch to red led
			ERROR_ticker = HAL_GetTick();
			while ((HAL_GetTick() - ERROR_ticker) < 300) {
				//wait 600 ms
			}
			// keep off from reseting
			//__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
		}
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
