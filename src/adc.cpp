/**
	this is for ADC voltage detection

**/

// 需要修改的地方：
/*
	1 GPIO， "gpio_msp.c"
*/

#include "adc.h"
#include "main.h"
#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_hal_adc_ex.h"

void (*adc_received_callback)(sAdcVal *);
#define USE_ADC_TIM 0

ADC_HandleTypeDef AdcHandle;
ADC_ChannelConfTypeDef sConfig;
DMA_HandleTypeDef DmaHandle;

uint16_t Adc_Buffer[ADC_BUFFER_LEN];
uint32_t adc_u32val[ADC_NUMBERS];
float adc_fval[ADC_NUMBERS];
sAdcVal adcVal;
const float KV = (100e3f + 20e3f) / 20e3f; // 输出电压分压比
const float KI = 5.0f;					   // current sense IC INA253A2, 200mV/A
const float KB = (20e3f + 10e3f) / 10e3f;  // 母线电压分压比
const float ADC_CALI_K = ((float)((uint32_t)(*VREFINT_CAL_ADDR) * (uint32_t)VREFINT_CAL_VREF)) / 4096e3f;

static float fBVal;

static float temperature_avg_slope;
static float fTVal;
static int32_t tmperature1, tmperature2;
static float tmperature1_voltage, tmperature2_voltage;
static void ADC_Temperature_Sense_Init(void);

static void ADC_Timer_Trigger_Source_Setup(void);
void ADC_DMA_ReStart(void);
static void ADC_data_management(void);
static void ADC_data_fetch(void);

void HAL_ADC_DMA_Init(ADC_HandleTypeDef *hadc);
void HAL_ADC_DMA_DeInit(ADC_HandleTypeDef *hadc);

/** initial function for ADC **/
void ADC_init(void)
{
	uint8_t i;
	for (i = 0; i < ADC_NUMBERS; i++)
	{
		adc_fval[i] = 0.0f;
		adc_u32val[i] = 0;
	}
	adc_received_callback = NULL;

	ADC_Temperature_Sense_Init();

	// ### - 1 - Initialize ADC peripheral ############################################
	AdcHandle.Instance = ADC_HANDLE;
	HAL_ADC_DeInit(&AdcHandle); // 先注销以前的配置

	AdcHandle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;			   // Synchronous clock mode, input ADC clock divided by 4, PCLK = APB2; max=36MHz, see DS page133
	AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;						   // 12-bit resolution for converted data
	AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;						   // Right-alignment for converted data
	AdcHandle.Init.ScanConvMode = ENABLE;								   // Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1)
	AdcHandle.Init.EOCSelection = ADC_EOC_SEQ_CONV;						   // EOC flag picked-up to indicate conversion end                        // Auto-delayed conversion feature disabled
	AdcHandle.Init.ContinuousConvMode = DISABLE;						   // Continuous mode enabled (automatic conversion restart after each conversion)
	AdcHandle.Init.NbrOfConversion = ADC_NUMBERS;						   // Parameter discarded because sequencer is disabled
	AdcHandle.Init.DiscontinuousConvMode = ENABLE;						   // Parameter discarded because sequencer is disabled
	AdcHandle.Init.NbrOfDiscConversion = 1;								   // Parameter discarded because sequencer is disabled
	AdcHandle.Init.ExternalTrigConv = ADC_TRIGGER_MODE;					   // Software start to trig the 1st conversion manually, without external event
	AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING; // Parameter discarded because software trigger chosen
	AdcHandle.Init.DMAContinuousRequests = ENABLE;						   // ADC DMA continuous request to match with DMA circular mode

	// Initialize ADC peripheral according to the passed parameters
	HAL_ADC_DMA_Init(&AdcHandle);
	HAL_ADC_Init(&AdcHandle);

	// ### - 3 - Channel configuration ########################################
	sConfig.Channel = ADC_CHANNEL_0;				// Sampled channel number VI
	sConfig.Rank = 1;								// Rank of sampled channel number ADCx_CHANNEL [1 16]
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES; // Sampling time (number of clock cycles unit)
	sConfig.Offset = 0;								// Parameter discarded because offset correction is disabled
	HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);

	sConfig.Channel = ADC_CHANNEL_1;				// Sampled channel number ADC1
	sConfig.Rank = 2;								// Rank of sampled channel number ADCx_CHANNEL [1 16]
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES; // Sampling time (number of clock cycles unit)
	sConfig.Offset = 0;								// Parameter discarded because offset correction is disabled
	HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);

	sConfig.Channel = ADC_CHANNEL_2;				// Sampled channel number ADC2
	sConfig.Rank = 3;								// Rank of sampled channel number ADCx_CHANNEL [1 16]
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES; // Sampling time (number of clock cycles unit)
	sConfig.Offset = 0;								// Parameter discarded because offset correction is disabled
	HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);

	sConfig.Channel = ADC_CHANNEL_3;				// Sampled channel number ADC3
	sConfig.Rank = 4;								// Rank of sampled channel number ADCx_CHANNEL [1 16]
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES; // Sampling time (number of clock cycles unit)
	sConfig.Offset = 0;								// Parameter discarded because offset correction is disabled
	HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);

	sConfig.Channel = ADC_CHANNEL_6;				// Sampled channel number ADC4
	sConfig.Rank = 5;								// Rank of sampled channel number ADCx_CHANNEL [1 16]
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES; // Sampling time (number of clock cycles unit)
	sConfig.Offset = 0;								// Parameter discarded because offset correction is disabled
	HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);

	sConfig.Channel = ADC_CHANNEL_7;				// Sampled channel number ADC5
	sConfig.Rank = 6;								// Rank of sampled channel number ADCx_CHANNEL [1 16]
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES; // Sampling time (number of clock cycles unit)
	sConfig.Offset = 0;								// Parameter discarded because offset correction is disabled
	HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);

	sConfig.Channel = ADC_CHANNEL_8;				// Sampled channel number ADC6
	sConfig.Rank = 7;								// Rank of sampled channel number ADCx_CHANNEL [1 16]
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES; // Sampling time (number of clock cycles unit)
	sConfig.Offset = 0;								// Parameter discarded because offset correction is disabled
	HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);

	sConfig.Channel = ADC_CHANNEL_9;				// Sampled channel number ADC7
	sConfig.Rank = 9;								// Rank of sampled channel number ADCx_CHANNEL [1 16]
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES; // Sampling time (number of clock cycles unit), sensor min 4us
	sConfig.Offset = 0;								// Parameter discarded because offset correction is disabled
	HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);

	sConfig.Channel = ADC_CHANNEL_17;				// VREFINT
	sConfig.Rank = 8;								// Rank of sampled channel number ADCx_CHANNEL [1 16]
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES; // Sampling time (number of clock cycles unit), vrefint min 4us
	sConfig.Offset = 0;								// Parameter discarded because offset correction is disabled
	HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);

	// ### - 4 - Start conversion in DMA mode #################################
	HAL_ADC_Start_DMA(&AdcHandle,
					  (uint32_t *)Adc_Buffer,
					  ADC_BUFFER_LEN);

	DMA_ADC_ISR_FLAG_ALL_CLEAR;
	DMA_ADC_Stream->CR &= (~(DMA_SxCR_DMEIE)); // direct mode error interrupt disable
	DMA_ADC_Stream->CR &= (~(DMA_SxCR_TEIE));  // transfer error interrupt disable
	DMA_ADC_Stream->CR &= (~(DMA_SxCR_HTIE));  // half transfer interrupt disable
	DMA_ADC_Stream->CR |= (DMA_SxCR_TCIE);	   // tranfer complete interrupt enable

	__HAL_ADC_DISABLE_IT(&AdcHandle, ADC_IT_OVR);
	if (USE_ADC_TIM == 1)
		ADC_Timer_Trigger_Source_Setup();
}

/*
// over-write function
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
	// ADC Periph clock enable
  ADC_CLOCK_ENABLE();

	#ifdef DMA_ADC
	// DMA clock enable
	DMA_ADC_CLOCK_ENABLE;
	#endif
	// GPIO pin setting please refer to "gpio_msp.c" file

	// ********************** Configure DMA parameters ***************************
  DmaHandle.Instance                 = DMA_ADC_Stream;

  DmaHandle.Init.Channel             = DMA_ADC_Channel;
  DmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  DmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
  DmaHandle.Init.MemInc              = DMA_MINC_ENABLE;
  DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  DmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
  DmaHandle.Init.Mode                = DMA_CIRCULAR;
  DmaHandle.Init.Priority            = DMA_PRIORITY_HIGH;
	DmaHandle.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  DmaHandle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_HALFFULL;
  DmaHandle.Init.MemBurst            = DMA_MBURST_SINGLE;
  DmaHandle.Init.PeriphBurst         = DMA_PBURST_SINGLE;
  // Deinitialize  & Initialize the DMA for new transfer
  HAL_DMA_DeInit(&DmaHandle);
  HAL_DMA_Init(&DmaHandle);

  // Associate the DMA handle
  __HAL_LINKDMA(hadc, DMA_Handle, DmaHandle);

  // NVIC configuration for DMA Input data interrupt
	DMA_ADC_ISR_FLAG_ALL_CLEAR;
  HAL_NVIC_SetPriority(DMA_ADC_IRQn, ADC_Prior, 0);
  HAL_NVIC_EnableIRQ(DMA_ADC_IRQn);
}
*/

// over-write function
/*
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{

  // ##-1- Reset peripherals ##################################################
  __HAL_RCC_ADC_FORCE_RESET();
  __HAL_RCC_ADC_RELEASE_RESET();
  // ADC Periph clock disable
  //  (automatically reset all ADC's)
  ADC_CLOCK_DISABLE();
}
*/
void HAL_ADC_DMA_Init(ADC_HandleTypeDef *hadc)
{
	// ADC Periph clock enable
	ADC_CLOCK_ENABLE();

#ifdef DMA_ADC
	// DMA clock enable
	DMA_ADC_CLOCK_ENABLE;
#endif
	// GPIO pin setting please refer to "gpio_msp.c" file

	// ********************** Configure DMA parameters ***************************
	DmaHandle.Instance = DMA_ADC_Stream;

	DmaHandle.Init.Channel = DMA_ADC_Channel;
	DmaHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
	DmaHandle.Init.PeriphInc = DMA_PINC_DISABLE;
	DmaHandle.Init.MemInc = DMA_MINC_ENABLE;
	DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	DmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	DmaHandle.Init.Mode = DMA_CIRCULAR;
	DmaHandle.Init.Priority = DMA_PRIORITY_HIGH;
	DmaHandle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	DmaHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
	DmaHandle.Init.MemBurst = DMA_MBURST_SINGLE;
	DmaHandle.Init.PeriphBurst = DMA_PBURST_SINGLE;
	// Deinitialize  & Initialize the DMA for new transfer
	HAL_DMA_DeInit(&DmaHandle);
	HAL_DMA_Init(&DmaHandle);

	/* Associate the DMA handle */
	__HAL_LINKDMA(hadc, DMA_Handle, DmaHandle);

	// NVIC configuration for DMA Input data interrupt
	DMA_ADC_ISR_FLAG_ALL_CLEAR;
	HAL_NVIC_SetPriority(DMA_ADC_IRQn, ADC_Prior, 0);
	HAL_NVIC_EnableIRQ(DMA_ADC_IRQn);
}

void HAL_ADC_DMA_DeInit(ADC_HandleTypeDef *hadc)
{

	// ##-1- Reset peripherals ##################################################
	__HAL_RCC_ADC_FORCE_RESET();
	__HAL_RCC_ADC_RELEASE_RESET();
	// ADC Periph clock disable
	//  (automatically reset all ADC's)
	ADC_CLOCK_DISABLE();
}

void ADC_DMA_ReStart(void)
{
	DMA_ADC_Stream->CR &= (~(DMA_SxCR_EN)); // bit 0, clear, DMA channel disable

	// enable DMA
	DMA_ADC_Stream->NDTR = ADC_BUFFER_LEN; // length of size
	DMA_ADC_Stream->CR |= (DMA_SxCR_EN);   // bit 0, set, DMA channel enable
	SET_BIT(ADC_HANDLE->CR2, ADC_CR2_DMA); // enable ADC DMA

	if ((ADC_HANDLE->CR2 & ADC_CR2_EXTEN_Msk) != 0x00)
	{									 // 如果不是硬件触发的话，要在其他地方每个一段时间手动写ADSTART
		ADC_HANDLE->CR2 |= ADC_CR2_ADON; // to start ADC , if not set, ADC would not start to work
	}
}

void ADC_DMA_callback(void)
{
	uint32_t isrflags = DMA_ADC_ISR_VALUE;

	DMA_ADC_ISR_FLAG_ALL_CLEAR;

	if ((isrflags &= DMA_ADC_COMPLETE_MASK) == DMA_ADC_COMPLETE_MASK)
	{ // complete
		// ... to do something
		ADC_data_fetch(); // just store the data
		// ADC_DMA_ReStart(); // start a new conversion，留在control里面写
		ADC_data_management(); // maybe use a lot of time to run algorithm
	}

	if ((isrflags &= DMA_ADC_ERROR_MASK) != 0)
	{ // error occurs
	}
}

static void ADC_data_fetch(void)
{
	// squence 1 --> fb
	// squence 2 --> Vrefint
	uint8_t i, j;
	for (i = 0; i < ADC_NUMBERS; i++)
	{
		adc_u32val[i] = 0;
	}

	for (i = 0; i < (ADC_BUFFER_LEN); i += ADC_NUMBERS)
	{
		for (j = 0; j < ADC_NUMBERS; j++)
		{
			adc_u32val[j] += (uint32_t)Adc_Buffer[i + j];
		}
	}
}

static void ADC_data_management(void)
{

	//                  VREFINT_CAL_VREF/1000 * VREFINT_CAL
	// V(channelx) = --------------------------------------- * ADCx_DATA
	//                       VREFINT_DATA * FULL_SCALE
	float tmp;
	float adc_refint = (float)adc_u32val[ADC_NUMBERS - 1];
	tmp = ADC_CALI_K / (float)adc_refint;

	uint8_t i;

	for (i = 0; i < ADC_NUMBERS; i++)
	{
		adc_fval[i] = tmp * (float)adc_u32val[i] / 2.0f;
	}
	// */

	// fTVal = temperature_avg_slope * (adc_fval[ADC_NUMBERS - 2] - tmperature1_voltage) + tmperature1;

	fBVal = adc_fval[0] * KB;

	adcVal.vb = fBVal;
	// adcVal.tmperature = fTVal;
	adcVal.io[0] = adc_fval[1] * KI;
	adcVal.io[1] = adc_fval[2] * KI;
	adcVal.io[2] = adc_fval[3] * KI;
	adcVal.io[3] = adc_fval[4] * KI;
	adcVal.io[4] = adc_fval[5] * KI;
	adcVal.io[5] = adc_fval[6] * KI;
	adcVal.io[6] = adc_fval[7] * KI;

	if (adc_received_callback != NULL)
	{
		adc_received_callback(&adcVal);
	}
	// adc_data_received(adc_fval, ADC_NUMBERS);
}

static TIM_HandleTypeDef htim;
static void ADC_Timer_Trigger_Source_Setup(void)
{
	// TIM_HandleTypeDef  htim;

	uint32_t period = (uint32_t)((float)tim_CK_INT_get(ADC_TRIGGER_TIMER) / ADC_SAMPLE_FREQUENCY);

	ADC_TRIGGER_TIMER_CLOCK_ENABLE();

	// ##-1- Configure the TIM peripheral #######################################
	htim.Instance = ADC_TRIGGER_TIMER;
	htim.Init.Period = period - 1;
	htim.Init.Prescaler = 0;
	htim.Init.ClockDivision = 0;
	htim.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_Base_Init(&htim);

	htim.Instance->CR2 |= 0x20; // set update event as TRGO

	// ##-4- Start PWM signals generation #######################################
	//  Start channel 1
	HAL_TIM_Base_Start(&htim);
}

static void ADC_Temperature_Sense_Init(void)
{
	// 温度系数
	// slope = (T2 - T1)/(Vt2 - Vt1) = C/V
	// T = slope * (Vt - Vt1) + T1
	tmperature1 = TEMPSENSOR_CAL1_TEMP;
	tmperature2 = TEMPSENSOR_CAL2_TEMP;
	tmperature1_voltage = (float)(*TEMPSENSOR_CAL1_ADDR) * (float)TEMPSENSOR_CAL_VREFANALOG / 4096e3f;
	tmperature2_voltage = (float)(*TEMPSENSOR_CAL2_ADDR) * (float)TEMPSENSOR_CAL_VREFANALOG / 4096e3f;
	temperature_avg_slope = (float)(tmperature2 - tmperature1) / (float)(tmperature2_voltage - tmperature1_voltage);
}

float ADC_VBAT_Get(void)
{
	return fBVal;
}

float ADC_Temperature_Get(void)
{
	return fTVal;
}

sAdcVal ADC_FB_Get(void)
{
	return adcVal;
}