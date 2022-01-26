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
#include "crc.h"
#include "dma.h"
#include "i2s.h"
#include "pdm2pcm.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_audio_if.h"
#define ARM_MATH_CM4
#include "arm_math.h"
#include "bosz.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BUFF_OGOLNY 480
#define FILTER_NUM 82
#define BLOCK_SIZE_FLOAT 48
#define PDM_SIZE 384

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t pdmRxBuf[PDM_SIZE];
uint16_t pdmRxBuf2[PDM_SIZE];
extern USBD_HandleTypeDef hUsbDeviceFS;

uint16_t MidBuffer[PDM_SIZE/8];
uint16_t * ptr_MidBuffer = &MidBuffer[0];

q31_t MidBuffer_1[PDM_SIZE/8];
q31_t MidBuffer_2[PDM_SIZE/8];

uint8_t rxstate = 0;
uint16_t usbstate = 0;

uint8_t DMA_runn = 0;
uint16_t fir_w = 0;

q31_t fifobuf_f[BUFF_OGOLNY];
q31_t fifobuf_f2[BUFF_OGOLNY];

uint16_t fifo_w_ptr = 0;
uint16_t fifo_r_ptr = 0;

uint16_t fifo_w_ptr2 = 0;
uint16_t fifo_r_ptr2 = 0;

int16_t diffrent = 0;
uint16_t licznik = 0;


q31_t filter_tab[FILTER_NUM]={
	     -29713931,    -1635910,     2131509,    -2941912,     4005173,    -5250669,
	       6591543,    -7930096,     9158950,   -10166461,    10840805,   -11075856,
	      10776764,    -9865628,     8287330,    -6013730,     3048806,      570251,
	      -4768478,     9438571,   -14406657,    19577807,   -24598702,    29310608,
	     -33430069,    36670039,   -38729859,    39301807,   -38069546,    34706520,
	     -28864852,    20153421,    -8093191,    -7968794,    29029755,   -56792706,
	      94509382,  -149271075,   239781175,  -435214886,  1360236750,  1360236750,
	    -435214886,   239781175,  -149271075,    94509382,   -56792706,    29029755,
	      -7968794,    -8093191,    20153421,   -28864852,    34706520,   -38069546,
	      39301807,   -38729859,    36670039,   -33430069,    29310608,   -24598702,
	      19577807,   -14406657,     9438571,    -4768478,      570251,     3048806,
	      -6013730,     8287330,    -9865628,    10776764,   -11075856,    10840805,
	     -10166461,     9158950,    -7930096,     6591543,    -5250669,     4005173,
	      -2941912,     2131509,    -1635910,   -29713931
	}; //od22k-do23.5k-dla48kHz //82

arm_fir_instance_q31 firsetting1, firsetting2;

q31_t fir_state1[BLOCK_SIZE_FLOAT + FILTER_NUM - 1];
q31_t fir_state2[BLOCK_SIZE_FLOAT + FILTER_NUM - 1];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void HAL_I2S_RxHalfCpltCallback (I2S_HandleTypeDef *hi2s);
void HAL_I2S_RxCpltCallback (I2S_HandleTypeDef *hi2s);
void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);

void FifoWrite_Q31();
uint16_t FifoRead_Q31();

void FifoWrite_Q31_2();
uint16_t FifoRead_Q31_2();

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);


void ADC_to_MIC(void){ //funkcja Call dla USB Audio

	HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)pdmRxBuf2, PDM_SIZE);
	HAL_I2S_Receive_DMA(&hi2s2, &pdmRxBuf[0], PDM_SIZE/2);


	LED4_ON();
	HAL_TIM_Base_Start_IT(&htim14);
	__HAL_TIM_SET_COUNTER(&htim14,0);

	if(DMA_runn==1){
		usbstate = 1;
	}
	else {
		usbstate = 2;
	}


}


void Zatrzymaj_DMA(){



	fifo_w_ptr = fifo_r_ptr + 240;
	if (fifo_w_ptr>BUFF_OGOLNY){fifo_w_ptr=fifo_w_ptr-BUFF_OGOLNY;}
	fifo_w_ptr2 = fifo_w_ptr;

	(GPIOD->ODR ^= (GPIO_PIN_15));

}


void Zatrzymaj_DMA_WatchDog(){

	HAL_I2S_DMAStop(&hi2s2);
	HAL_SPI_DMAStop(&hspi1);
	//memset(&fifobuf_f_out, (uint8_t) 0, sizeof(fifobuf_f_out));
	memset(&fifobuf_f, (uint8_t) 0, sizeof(fifobuf_f));
	memset(&pdmRxBuf, (uint8_t) 0, sizeof(pdmRxBuf));
	//memset(&fir_state1, (uint8_t) 0, sizeof(fir_state1));
	arm_fir_init_q31(&firsetting1, FILTER_NUM, &filter_tab[0], &fir_state1[0], BLOCK_SIZE_FLOAT);

	//memset(&fifobuf_f2_out, (uint8_t) 0, sizeof(fifobuf_f2_out));
	memset(&fifobuf_f2, (uint8_t) 0, sizeof(fifobuf_f2));
	memset(&pdmRxBuf2, (uint8_t) 0, sizeof(pdmRxBuf2));
	//memset(&fir_state2, (uint8_t) 0, sizeof(fir_state2));
	arm_fir_init_q31(&firsetting2, FILTER_NUM, &filter_tab[0], &fir_state2[0], BLOCK_SIZE_FLOAT);

	fifo_w_ptr = 0;
	fifo_r_ptr = 0;
	fifo_w_ptr2 = 0;
	fifo_r_ptr2 = 0;
	rxstate = 0;
	DMA_runn = 0;
	usbstate = 2;
	fir_w = 0;
	LED1_OFF();
	LED4_OFF();


}
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
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_CRC_Init();
  MX_PDM2PCM_Init();
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
	
  arm_fir_init_q31(&firsetting1, FILTER_NUM, &filter_tab[0], &fir_state1[0], BLOCK_SIZE_FLOAT);
  arm_fir_init_q31(&firsetting2, FILTER_NUM, &filter_tab[0], &fir_state2[0], BLOCK_SIZE_FLOAT);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	if (rxstate==1) {

		PDM_Filter(&pdmRxBuf[0],ptr_MidBuffer, &PDM1_filter_handler);
	 	PDM_Filter(&pdmRxBuf[64],ptr_MidBuffer+16, &PDM1_filter_handler);
	 	PDM_Filter(&pdmRxBuf[128],ptr_MidBuffer+32, &PDM1_filter_handler);

	 	for (int i=0; i<PDM_SIZE/8;++i) {

	 	    	FifoWrite_Q31();
	 	    	MidBuffer_1[i]= ((int) (MidBuffer[i]<<16)|0x0000);

	 	}

	 	PDM_Filter(&pdmRxBuf2[0],ptr_MidBuffer, &PDM2_filter_handler);
	 	PDM_Filter(&pdmRxBuf2[64],ptr_MidBuffer+16, &PDM2_filter_handler);
	 	PDM_Filter(&pdmRxBuf2[128],ptr_MidBuffer+32, &PDM2_filter_handler);

	 	for (int i=0; i<PDM_SIZE/8;++i) {

	 	    	FifoWrite_Q31_2();
	 	    	MidBuffer_2[i]= ((int) (MidBuffer[i]<<16)|0x0000);

	 	}



	 	fir_w=fifo_w_ptr;
	 	rxstate=0;



	}

	if (rxstate==2) {

 	      	PDM_Filter(&pdmRxBuf[192],ptr_MidBuffer, &PDM1_filter_handler);
 	      	PDM_Filter(&pdmRxBuf[256],ptr_MidBuffer+16, &PDM1_filter_handler);
 	      	PDM_Filter(&pdmRxBuf[320],ptr_MidBuffer+32, &PDM1_filter_handler);

 	      	for (int i=0; i<PDM_SIZE/8;++i) {

 	      		FifoWrite_Q31();
 	      		MidBuffer_1[i]= ((int) (MidBuffer[i]<<16)|0x0000);

 	      	}

 	      	PDM_Filter(&pdmRxBuf2[192],ptr_MidBuffer, &PDM2_filter_handler);
 	      	PDM_Filter(&pdmRxBuf2[256],ptr_MidBuffer+16, &PDM2_filter_handler);
 	      	PDM_Filter(&pdmRxBuf2[320],ptr_MidBuffer+32, &PDM2_filter_handler);

 	      	for (int i=0; i<PDM_SIZE/8;++i) {

 	      		FifoWrite_Q31_2();
 	      		MidBuffer_2[i]= ((int) (MidBuffer[i]<<16)|0x0000);

 	      	}

 	      	fir_w=fifo_w_ptr;
 	      	rxstate=0;
	 }


	 if (fir_w > 0){
	  	  arm_fir_q31(&firsetting1, &MidBuffer_1[0] , &fifobuf_f[fir_w - BLOCK_SIZE_FLOAT], BLOCK_SIZE_FLOAT);
	  	  arm_fir_q31(&firsetting2, &MidBuffer_2[0] , &fifobuf_f2[fir_w - BLOCK_SIZE_FLOAT], BLOCK_SIZE_FLOAT);


	    	  if (fir_w-fifo_r_ptr>48 && DMA_runn==0 ) {DMA_runn = 1;}
		    	  ++licznik;
		    	  if(licznik%1000==0){
		    		  diffrent = ((int16_t) fir_w) - ((int16_t)fifo_r_ptr);
		    		  if(diffrent<0){diffrent=diffrent+BUFF_OGOLNY;}
	   		  	  if(diffrent>96){
		  		  	fifo_w_ptr = fifo_r_ptr + 96;
	  		    	  	if (fifo_w_ptr>BUFF_OGOLNY){fifo_w_ptr=fifo_w_ptr-BUFF_OGOLNY;}
	    			  	fifo_w_ptr2 = fifo_w_ptr;
	  	 			}

		  	  	  licznik = 0;
	  	}
	  	fir_w = 0;
	  }
	  if(usbstate == 1){
		  
	    	  USBD_AUDIO_HandleTypeDef *haudio = hUsbDeviceFS.pClassData;
	    	  int16_t *buf_part = haudio->in_buffer + (AUDIO_IN_PACKET / 2) * haudio->in_buffer_half;  // USB mic buffer access

	    	  for (uint16_t i = 0; i < (AUDIO_IN_PACKET / 2); i+=2) {

	    		  //uint16_t temp = FifoRead_Q31();
	  	  	  *(buf_part+i) = FifoRead_Q31();
	  	  	  *(buf_part+i+1) = FifoRead_Q31_2();	
		  }

	    	  usbstate=0;

	   }

	   if(usbstate == 2){ //gdy USB potrzebuje danych a ich jeszcze niema

	    	  USBD_AUDIO_HandleTypeDef *haudio = hUsbDeviceFS.pClassData;
	    	  int16_t *buf_part = haudio->in_buffer + (AUDIO_IN_PACKET / 2) * haudio->in_buffer_half;  // USB mic buffer access
	    	  //pobranie adresu do wypelnienia

	    	  for (uint16_t i = 0; i < (AUDIO_IN_PACKET / 2); i+=2) {

	  	  	*(buf_part+i) = 0;
	  	  	*(buf_part+i+1) = 0;

	  	  }

	    	  usbstate=0;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 210;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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

