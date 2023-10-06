/** 
	F405的最高时钟是144MHz，最低是2MHz或者4MHz
	
	使用PLL = in/M *N /P
		1 in/M*N 要约在100-432MHz范围内，不是绝对要大于等于100MHz
		2 N要大于50，不然自动被分配到50
		3 M和P都要大于等于2
		4 M=(2,63), N=(50-432), P=(2-8)
		
	FLASH_LATENCY
	FLASH_LATENCY_0 = (0-30)MHz
	FLASH_LATENCY_1 = (30-60)MHz
	FLASH_LATENCY_2 = (60-90)MHz
	FLASH_LATENCY_3 = (90-120)MHz
	FLASH_LATENCY_4 = (120-150)MHz
	FLASH_LATENCY_5 = (150-180)MHz

  hardware:
    HSE = 8MHz [4-26MHz is valid], 注意在#include <stm32f4xx_hal_conf.h>里默认为25MHz，要改回来
    HSI = 16MHz
    LSI = 32kHz
    LSE = 32.768kHz	
**/

#include "system_clock_rcc.h"


/**
//  * @brief  System Clock Configuration
//  *         The system Clock is configured as follow : 
//  *            System Clock source            = PLL (HSE)
//  *            SYSCLK(Hz)                     = 168000000
//  *            HCLK(Hz)                       = 168000000
//  *            AHB Prescaler                  = 1
//  *            APB1 Prescaler                 = 4
//  *            APB2 Prescaler                 = 2
//  *            HSE Frequency(Hz)              = 8000000 // HSE_VALUE is set in stm32f4xx_hal_conf.h, default is 25MHz
//  *            PLL_M                          = 4
//  *            PLL_N                          = 168
//  *            PLL_P                          = 2
//  *            PLL_Q                          = 7
//  *            VDD(V)                         = 3.3
//  *            Main regulator output voltage  = Scale2 mode
//  *            Flash Latency(WS)              = 4
//  * @param  None
//  * @retval None
**/
void SystemClock_Init(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	
  // Enable Power Control clock 
  __HAL_RCC_PWR_CLK_ENABLE();

  // The voltage scaling allows optimizing the power consumption when the device is 
  //   clocked below the maximum system frequency, to update the voltage scaling value 
  //   regarding system frequency refer to product datasheet.  
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  // Enable HSE Oscillator and activate PLL with HSE as source 
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;  // notice that HSI = 16MHz
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
 
  // Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
  //   clocks dividers
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  while (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK){};

  // STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  
  if (HAL_GetREVID() == 0x1001)
  {
    // Enable the Flash prefetch 
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}

void SystemClockHSE_PLL_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	
  // -1- Select HSI as system clock source to allow modification of the PLL configuration 
	SystemClockHSI_Config();
  //RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  //RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  //HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  
  // -2- Enable HSE Oscillator, select it as PLL source and finally activate the PLL 
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4; // notice that HSE = 8MHz
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  // -3- Select the PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers 
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  while (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK){};
  
  // -4- Optional: Disable HSI Oscillator (if the HSI is no more needed by the application)
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
}

void SystemClockHSI_PLL_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	
  // -1- Select HSI as system clock source to allow modification of the PLL configuration 
	SystemClockHSI_Config();
  //RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  //RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  //HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  // -2- Enable HSI Oscillator, select it as PLL source and finally activate the PLL
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8; // notice that HSI = 16MHz
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  // -3- Select the PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers 
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  while (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK){};
  
  // -4- Optional: Disable HSE Oscillator (if the HSE is no more needed by the application) 
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
}

void SystemClockHSI_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	
	// -0- Enable HSI Oscillator, and disable PLL
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);
	
	// -1- Select HSI as system clock source to allow modification of the PLL configuration 
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_SYSCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_SYSCLK_DIV1;  
  while (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK){};
	
	// -4- Optional: Disable HSE Oscillator (if the HSE is no more needed by the application)
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
}

void SystemClockHSE_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	
	// -0- Enable HSE Oscillator, and disable PLL
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	while( HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK ){};
	
	// -1- Select HSE as system clock source to allow modification of the PLL configuration 
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_SYSCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_SYSCLK_DIV1;  
  while (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK){};
	
	// -4- Optional: Disable HSI Oscillator (if the HSI is no more needed by the application)
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
}

void SystemClock_HSI_4MHz_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	
	//SystemClockHSI_Config();
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	
	// -2- Enable HSI Oscillator, select it as PLL source and finally activate the PLL
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 32; // notice that HSI = 16MHz   input /M *N /P
  RCC_OscInitStruct.PLL.PLLN = 64;  // 50 < N < 432  input/M*N is in range 100 - 432MHz
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
	RCC_OscInitStruct.PLL.PLLQ = 6;
	while( HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK ){};
  
  // -3- Select the PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers 
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  while (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK){};
	MODIFY_REG(RCC->CFGR, RCC_CFGR_MCO1PRE, RCC_CFGR_MCO1PRE_2 | RCC_CFGR_MCO1PRE_1);
	MODIFY_REG(RCC->CFGR, RCC_CFGR_MCO1, RCC_CFGR_MCO1_0 | RCC_CFGR_MCO1_1);
  
  // -4- Optional: Disable HSE Oscillator (if the HSE is no more needed by the application) 
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
}

void SystemClock_HSI_32MHz_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	
	//SystemClockHSI_Config();
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	
	// -2- Enable HSI Oscillator, select it as PLL source and finally activate the PLL
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16; // notice that HSI = 16MHz   input /M *N /P
  RCC_OscInitStruct.PLL.PLLN = 256;  // 50 < N < 432  input/M*N is in range 100 - 432MHz
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
	RCC_OscInitStruct.PLL.PLLQ = 6;
  while( HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK ){};
  
  // -3- Select the PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers 
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  while (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK){};
  
  // -4- Optional: Disable HSE Oscillator (if the HSE is no more needed by the application) 
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
}

void SystemClock_HSE_4MHz_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	
  // -1- Select HSI as system clock source to allow modification of the PLL configuration 
	SystemClockHSI_Config();
  //RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  //RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  //HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  
  // -2- Enable HSE Oscillator, select it as PLL source and finally activate the PLL 
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16; // notice that HSE = 8MHz
  RCC_OscInitStruct.PLL.PLLN = 64; // 不能小于50
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  while( HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK ){};
  
  // -3- Select the PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers 
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  while (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK){};
	MODIFY_REG(RCC->CFGR, RCC_CFGR_MCO1PRE, RCC_CFGR_MCO1PRE_2 | RCC_CFGR_MCO1PRE_1);
	MODIFY_REG(RCC->CFGR, RCC_CFGR_MCO1, RCC_CFGR_MCO1_0 | RCC_CFGR_MCO1_1);
  
  // -4- Optional: Disable HSI Oscillator (if the HSI is no more needed by the application)
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
}

void gpio_power_save(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* Enable GPIOs clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
  
    /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
    /* Note: Debug using ST-Link is not possible during the execution of this   */
    /*       example because communication between ST-link and the device       */
    /*       under test is done through UART. All GPIO pins are disabled (set   */
    /*       to analog input mode) including  UART I/O pins.           */
    GPIO_InitStructure.Pin = GPIO_PIN_All;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
  
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
  
    /* Disable GPIOs clock */
    __HAL_RCC_GPIOA_CLK_DISABLE();
    __HAL_RCC_GPIOC_CLK_DISABLE();
}



