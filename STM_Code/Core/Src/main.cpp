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
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LeptonFLiR.h"
#include "hm01b0.h"
#include "hm01b0_reg.h"

#include "stdio.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
bool BLUE_BUTTON = false;
bool skip_first = true;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */



  // UNCOMMENT THE OPTION TO USE

//#define LEPTON_ACTIVE 1
//#define HIMAX_ACTIVE 1
#define BOTH_SEPARATE 1
//#define SOVRAPP 1




  // ************************
  //       INIT LEPTON
  // ************************
#if defined LEPTON_ACTIVE || defined BOTH_SEPARATE || defined SOVRAPP

  LEPTON lepton;

  lepton.format 	= LeptonFLiR_ImageStorageMode_RGB888;	// either B&W (LeptonFLiR_ImageStorageMode_RAW14) or RGB (LeptonFLiR_ImageStorageMode_RGB888)
  lepton.temp 		= LeptonFLiR_TemperatureMode_Celsius;	// definition of temperature scale, doesn't do anything (was original of previous code)
  lepton.agc_en 	= LEP_AGC_ENABLE;						// enable or disable AGC (automatic gain control)
  lepton.agc_policy = LEP_AGC_HEQ;							// either linear (LEP_AGC_LINEAR) or histogram (LEP_AGC_HEQ) - refer to section 3.6 of data sheet
  lepton.telemetry 	= false;								// enable or disable Telemetry
  lepton.color 		= LEP_VID_FUSION_LUT;					// select the built in color palette (only for RGB mode)  - refer to page 38 of data sheet



  LeptonFLiR flirController;								// define the class
  flirController.init(lepton.format, lepton.temp);			// init the class

  HAL_Delay(1000);
  flirController.Lepton_setup(&lepton);						// set up Lepton camera as defined in the struct (other more specific set ups are made also in this function)


  // definitions for the transmission cycle
  bool leptonCapture = false;
  uint8_t uart_tx_lep[lepton.W];

#endif



  // ************************
  //        INIT HIMAX
  // ************************

#if defined HIMAX_ACTIVE || defined BOTH_SEPARATE || defined SOVRAPP

  HM01B0 himax = {0};

  himax.mode 		= Streaming;		// either: continuous stream (Streaming), specific number of frames (Streaming2), send trigger for frame (Streaming3)
  himax.resolution 	= QQVGA;			// either: 164x122 (QQVGA), 324x244 (QVGA)
  himax.dataDepth 	= EightB;			// either: 6 bpp (EightB), 8 bpp (SixB)
  himax.dataIo 		= EightL;			// either: 8 Data lines (EightL), 4 Data lines (FourL), 8 Data lines (Serial)
  himax.tPattern 	= Disabled;			// enable the test pattern (Disabled, Walking, ColorBar)


  hm01b0_reg_default_init();			// read and write the standard configuration of the camera

  hm01b0_setup(&himax);					// set up himax camera as defined in the struct


  // definitions for the transmission cycle
  uint32_t himax_pixels;
  uint8_t uart_tx_him[himax.raw_W];

#endif



	// ************************
	//        SOVRAPP
	// ************************
#ifdef SOVRAPP

	uint8_t himax_px;
	uint8_t uart_tx[(himax.raw_W)*3];

	uint16_t lepton_px_R, lepton_px_G, lepton_px_B;
	uint16_t lep_riga=0;
	uint16_t lep_col=0;

#endif

  HAL_Delay(1000);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  // to start or stop reading frames PRESS THE BLUE BUTTON
	  if( BLUE_BUTTON ){
		  HAL_Delay(1);

#if defined LEPTON_ACTIVE && not defined BOTH_SEPARATE && not defined SOVRAPP

		  leptonCapture = flirController.readNextFrame();

		  if( leptonCapture ){
			  for(int riga=lepton.H-1; riga >= 0; --riga){
				  for(int col=0; col< lepton.W; ++col){

					  if(lepton.format == LeptonFLiR_ImageStorageMode_RAW14){
						  uart_tx_lep[lepton.W-1-col] = flirController.RAW_getImageDataRowCol(riga,col);
					  }else if(lepton.format == LeptonFLiR_ImageStorageMode_RGB888){
						  uart_tx_lep[lepton.W-1-col] = flirController.RGB_getImageDataRowCol(riga,col);
					  }

				  }
					  HAL_UART_Transmit(&huart2, (uint8_t *)uart_tx_lep, sizeof(uart_tx_lep), 500);		// send 1 line of pixels
				  }
		  }

		  // for some reason I need to wait a bit only in RGB mode
		  if(lepton.format == LeptonFLiR_ImageStorageMode_RGB888){
			  HAL_Delay(20);
		  }
#endif


#if defined HIMAX_ACTIVE && not defined BOTH_SEPARATE && not defined SOVRAPP

		  // if the resolution selected is QWGA, skip entirely the first frame (should be 79544 pixels but is 20008 pixels)
		  if((himax.resolution==QVGA) && skip_first){
			  himax_pixels = STM_hm01b0_capture(&himax);
			  HAL_Delay(500);
		  }else{
			  skip_first = false;
		  }

		  himax_pixels = STM_hm01b0_capture(&himax);					// capture the image

		  if(himax_pixels == (himax.raw_H*himax.raw_W)) {				// ensure that I got all pixels in the frame
			  for(int riga=0; riga < himax.raw_H; riga++){
				  for(int col=0; col< himax.raw_W; col++){
					  uart_tx_him[col] = get_frame_row_col(&himax, riga, col);
				  }
				  HAL_UART_Transmit(&huart2, (uint8_t *)uart_tx_him, sizeof(uart_tx_him), 500);			// send 1 line of pixels
			  }
		  }
#endif


#ifdef BOTH_SEPARATE

		  himax_pixels = STM_hm01b0_capture(&himax);					// capture HIMAX image
		  leptonCapture = flirController.readNextFrame();				// capture LEPTON image

		  if((leptonCapture==true) && (himax_pixels == (himax.raw_H*himax.raw_W))){

			  // send to PC the HIMAX image
			  for(int riga=0; riga < himax.raw_H; riga++){
				  for(int col=0; col< himax.raw_W; col++){
					  uart_tx_him[col] = get_frame_row_col(&himax, riga, col);
				  }
				  HAL_UART_Transmit(&huart2, (uint8_t *)uart_tx_him, sizeof(uart_tx_him), 500);			// send 1 line of pixels
			  }

			  // send to PC the LEPTON image
			  for(int riga=lepton.H-1; riga >= 0; --riga){
				  for(int col=0; col < lepton.W; ++col){
					  if(lepton.format == LeptonFLiR_ImageStorageMode_RAW14){
						  uart_tx_lep[lepton.W-1-col] = flirController.RAW_getImageDataRowCol(riga,col);
					  }else if(lepton.format == LeptonFLiR_ImageStorageMode_RGB888){
						  uart_tx_lep[lepton.W-1-col] = flirController.RGB_getImageDataRowCol(riga,col);
					  }
				  }
				  HAL_UART_Transmit(&huart2, (uint8_t *)uart_tx_lep, sizeof(uart_tx_lep), 500);			// send 1 line of pixels
			  }

		}
#endif

// in our experiments the overlapping of the two images can be done directly on the STM, it's just a bit trickier to
// send the data on the UART. For what concerns the manipulation of the image (application of median or average filter)
// the results are much better if the manipulation is applied on another device (in our case the laptop). The STM gets
// really slow (less than 1 frame at second) if we try to apply the average filter (because it accesses to 5 pixels)
#ifdef SOVRAPP

		  himax_pixels = STM_hm01b0_capture(&himax);						// capture HIMAX image
		  leptonCapture = flirController.readNextFrame();					// capture LETPON image

		  if((leptonCapture==true) && (himax_pixels == (himax.raw_H*himax.raw_W))){

			  for(int riga=0; riga < himax.raw_H; riga++){
				  for(int col=0; col< himax.raw_W; col++){

					  // put thermo image in the center AND apply average

					  himax_px = get_frame_row_col(&himax, riga, col);

					  if(col<42 || col >122 || riga <31 || riga >91){
						  uart_tx[(col*3)]   = himax_px;
						  uart_tx[(col*3)+1] = himax_px;
						  uart_tx[(col*3)+2] = himax_px;
					  } else {
						  lep_riga = riga-31;
		  				  lep_col  = (col-42)*3;

						  lepton_px_R = flirController.RGB_getImageDataRowCol_AVERAGE(lep_riga,lep_col);
						  lepton_px_G = flirController.RGB_getImageDataRowCol_AVERAGE(lep_riga,lep_col+1);
						  lepton_px_B = flirController.RGB_getImageDataRowCol_AVERAGE(lep_riga,lep_col+2);

		  				  uart_tx[(col*3)]   = lepton_px_R*0.2 + himax_px*0.8;
		  				  uart_tx[(col*3)+1] = lepton_px_G*0.2 + himax_px*0.8;
		  				  uart_tx[(col*3)+2] = lepton_px_B*0.2 + himax_px*0.8;
		  			  }
		  		  }
		  		  HAL_UART_Transmit(&huart2, (uint8_t *)uart_tx, sizeof(uart_tx), 500);					// send 1 line of pixels
		  	  }
	  }

#endif
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_5);
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	// interrupt blue button
	if(GPIO_Pin==B1_Pin){
		BLUE_BUTTON = !BLUE_BUTTON;
	}
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
  __disable_irq();
  while (1)
  {
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
