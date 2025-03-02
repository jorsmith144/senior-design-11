/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f767xx.h"
#include "stm32f7xx.h"
#include "stm32f7xx_hal_def.h"
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.14159
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
//buffers for adc input and output
static int y[3] = {0, 0, 0};
static int x[3] = {0, 0, 0};

//buffers for filters
static int h1[3] = {0, 0, 0};
static int h2[3] = {0, 0, 0};
static int h3[3] = {0, 0, 0};
static int z[3] = {0, 0, 0};

//switching filters
static int filter_count = 0;       // Tracks selected filter (1 to 4)
//filter toggle
static bool filter_toggles[4] = {1, 1, 1, 1};
static int last_state_filter = 1;  // Last state for filter select button
static int last_state_toggle = 1;  // Last state for ON/OFF toggle button




// Delay Indices for Comb Filters
static int C_index_1 = 1;
static int C_index_2 = 1;
static int C_index_3 = 1;
static int C_index_4 = 1;

// Delay Indices for All-pass Filters
static int A_index_1 = 1;
static int A_index_2 = 1;
static int A_index_3 = 1;

// Reverb Parameters
float wet = 1.0, delay = 1.0;
float Cgain_1 = 0.805, Cgain_2 = 0.827, Cgain_3 = 0.783, Cgain_4 = 0.764; // Reverb gains
int Csum = 0, Aout_1 = 0, Aout_2 = 0, Aout_3 = 0;  // Intermediate reverb variables
int Cout_1 = 0, Cout_2 = 0, Cout_3 = 0, Cout_4 = 0;

int Clim_1 = 0, Clim_2 = 0, Clim_3 = 0, Clim_4 = 0;
int Alim_1 = 0, Alim_2 = 0, Alim_3 = 0;

int Adelay_1 = 220;
float Again_1 = 0.7;
int Adelay_2 = 74;
float Again_2 = 0.7;
int Adelay_3 = 21;
float Again_3 = 0.7;
int Cdelay_1 = 1590;
int Cdelay_2 = 1372;
int Cdelay_3 = 1782;
int Cdelay_4 = 1980;

int Cgain_sum = 0.25;

uint32_t adc_result = 0;

// Comb Filter Buffers
static int Cbuff_1[1590] = {0};
static int Cbuff_2[1372] = {0};
static int Cbuff_3[1782] = {0};
static int Cbuff_4[1980] = {0};

// All-pass Filter Buffers
static int Abuff_1[220] = {0};
static int Abuff_2[74] = {0};
static int Abuff_3[21] = {0};

float Fs = 48000.0;
// High Cut / Low Pass Filter Settings
int LPF_f0 = 10000;                         // Center / Corner Frequency
float LPF_Q = 1.0;                          // Quality Factor

// Low Cut / High Pass Filter Settings
int HPF_f0 = 20;                           // Center / Corner Frequency
float HPF_Q = 1.0;                         // Quality Factor

// Notch Filter Settings
int NCH1_f0 = 100;                         // Center / Corner Frequency
float NCH1_Q = 1.0;                        // Quality Factor

// Peaking Filter
int PKG_f0 = 500;                          // Center / Corner Frequency
float PKG_Q = 2.0;                         // Quality Factor

//static int PKG_dB_gain = 1;                       // Gain in dB
//float PKG_BW = 1.0;                            // Bandwidth in Octaves

// LPF Variables
float LPF_w0 = 0, LPF_cos_w0 = 0, LPF_sin_w0 = 0, LPF_alpha = 0;
float LPF_b0 = 0, LPF_b1 = 0, LPF_b2 = 0, LPF_a0 = 0, LPF_a1 = 0, LPF_a2 = 0;
float LPF_norm_b0 = 0, LPF_norm_b1 = 0, LPF_norm_b2 = 0, LPF_norm_a1 = 0, LPF_norm_a2 = 0;

// HPF Variables
float HPF_w0 = 0, HPF_cos_w0 = 0, HPF_sin_w0 = 0, HPF_alpha = 0;
float HPF_b0 = 0, HPF_b1 = 0, HPF_b2 = 0, HPF_a0 = 0, HPF_a1 = 0, HPF_a2 = 0;
float HPF_norm_b0 = 0, HPF_norm_b1 = 0, HPF_norm_b2 = 0, HPF_norm_a1 = 0, HPF_norm_a2 = 0;

// Notch Variables
float NCH1_w0 = 0, NCH1_cos_w0 = 0, NCH1_sin_w0 = 0, NCH1_alpha = 0;
float NCH1_b0 = 0, NCH1_b1 = 0, NCH1_b2 = 0, NCH1_a0 = 0, NCH1_a1 = 0, NCH1_a2 = 0;
float NCH1_norm_b0 = 0, NCH1_norm_b1 = 0, NCH1_norm_b2 = 0, NCH1_norm_a1 = 0, NCH1_norm_a2 = 0;

// Peaking EQ Variables
float PKG_w0 = 0, PKG_A = 0, PKG_cos_w0 = 0, PKG_sin_w0 = 0, PKG_alpha = 0;
float PKG_b0 = 0, PKG_b1 = 0, PKG_b2 = 0, PKG_a0 = 0, PKG_a1 = 0, PKG_a2 = 0;
float PKG_norm_b0 = 0, PKG_norm_b1 = 0, PKG_norm_b2 = 0, PKG_norm_a1 = 0, PKG_norm_a2 = 0;


void LPF() {
	 // LPF Coefficients and Parameters

	    LPF_w0 = 2 * PI * (LPF_f0 / Fs);    	// Cutoff Freq in radians
	    LPF_cos_w0 = cos(LPF_w0);           	// cosine omega
	    LPF_sin_w0 = sin(LPF_w0);           	// sine omega
	    LPF_alpha = LPF_sin_w0 / (2 * LPF_Q);

	    LPF_b0 = (1 - LPF_cos_w0) / 2;
	    LPF_b1 = (1 - LPF_cos_w0);
	    LPF_b2 = (1 - LPF_cos_w0) / 2;
	    LPF_a0 = 1 + LPF_alpha;
	    LPF_a1 = -2 * LPF_cos_w0;
	    LPF_a2 = 1 - LPF_alpha;

	    LPF_norm_b0 = LPF_b0 / LPF_a0;
	    LPF_norm_b1 = LPF_b1 / LPF_a0;
	    LPF_norm_b2 = LPF_b2 / LPF_a0;
	    LPF_norm_a1 = LPF_a1 / LPF_a0;
	    LPF_norm_a2 = LPF_a2 / LPF_a0;
}
void Notch() {
	  // Notch Coefficients and Parameters

	    NCH1_w0 = 2 * PI * (NCH1_f0 / Fs);      // Center Freq in radians
	    NCH1_cos_w0 = cos(NCH1_w0);             // cosine omega
	    NCH1_sin_w0 = sin(NCH1_w0);             // sine omega
	    NCH1_alpha = NCH1_sin_w0 / (2 * NCH1_Q);

	    NCH1_b0 = 1;
	    NCH1_b1 = -2 * NCH1_cos_w0;
	    NCH1_b2 = 1;
	    NCH1_a0 = 1 + NCH1_alpha;
	    NCH1_a1 = -2 * NCH1_cos_w0;
	    NCH1_a2 = 1 - NCH1_alpha;

	    NCH1_norm_b0 = NCH1_b0 / NCH1_a0;
	    NCH1_norm_b1 = NCH1_b1 / NCH1_a0;
	    NCH1_norm_b2 = NCH1_b2 / NCH1_a0;
	    NCH1_norm_a1 = NCH1_a1 / NCH1_a0;
	    NCH1_norm_a2 = NCH1_a2 / NCH1_a0;
}
void PEAK() {
	  // Peaking EQ Coefficients and Parameters

	    PKG_w0 = 2 * PI * (PKG_f0 / Fs);    	// Center Freq in radians
	    PKG_A = 0.1; //pow(10, PKG_dB_gain / 40);      	// Linear Gain from dB
	    PKG_cos_w0 = cos(PKG_w0);      			// cosine omega
	    PKG_sin_w0 = sin(PKG_w0);      			// sine omega
	    PKG_alpha = PKG_sin_w0 / (2 * PKG_Q);
	    // Case BW
	    //PKG_alpha = sin(PKG_w0) * sinh((log(2)/2) * PKG_BW * (PKG_w0 / sin(PKG_w0)));


	    PKG_b0 = 1 + PKG_alpha * PKG_A;
	    PKG_b1 = -2 * PKG_cos_w0;
	    PKG_b2 = 1 - PKG_alpha * PKG_A;
	    PKG_a0 = 1 + ( PKG_alpha / PKG_A );
	    PKG_a1 = -2 * PKG_cos_w0;
	    PKG_a2 = 1 - ( PKG_alpha / PKG_A );

	    PKG_norm_b0 = PKG_b0 / PKG_a0;
	    PKG_norm_b1 = PKG_b1 / PKG_a0;
	    PKG_norm_b2 = PKG_b2 / PKG_a0;
	    PKG_norm_a1 = PKG_a1 / PKG_a0;
	    PKG_norm_a2 = PKG_a2 / PKG_a0;
}

void HPF() {
	 // HPF Coefficients and Parameters

	    HPF_w0 = 2 * PI * (HPF_f0 / Fs);        // Cutoff Freq in radians
	    HPF_cos_w0 = cos(HPF_w0);               // cosine omega
	    HPF_sin_w0 = sin(HPF_w0);               // sine omega
	    HPF_alpha = HPF_sin_w0 / (2 * HPF_Q);

	    HPF_b0 = (1 + HPF_cos_w0) / 2;
	    HPF_b1 = -(1 + HPF_cos_w0);
	    HPF_b2 = (1 + HPF_cos_w0) / 2;
	    HPF_a0 = 1 + HPF_alpha;
	    HPF_a1 = -2 * HPF_cos_w0;
	    HPF_a2 = 1 - HPF_alpha;

	    HPF_norm_b0 = HPF_b0 / HPF_a0;
	    HPF_norm_b1 = HPF_b1 / HPF_a0;
	    HPF_norm_b2 = HPF_b2 / HPF_a0;
	    HPF_norm_a1 = HPF_a1 / HPF_a0;
	    HPF_norm_a2 = HPF_a2 / HPF_a0;
}



void Delay() {

    Clim_1 = (Cdelay_1 * delay);
    Clim_2 = (Cdelay_2 * delay);
    Clim_3 = (Cdelay_3 * delay);
    Clim_4 = (Cdelay_4 * delay);

    // Calculate Alim values
    Alim_1 = (Adelay_1 * delay);
    Alim_2 = (Adelay_2 * delay);
    Alim_3 = (Adelay_3 * delay);

    // Delay Calculated: Delay / Sample Time = Sample Buffer Length


}


void buttonDebouncer()
{
	 	 int button_press = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0);  // Filter select button
	 	 int toggle_press = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1);   // Filter ON/OFF toggle button
	    // Filter select button: Cycle through filters
	    if (button_press == 0 && last_state_filter == 1) {
	    	filter_count = (filter_count % 4) + 1;  // Cycle through filters
	    }
	    last_state_filter = button_press;

	    // Toggle button: Toggle the corresponding filter ON/OFF
	    if (toggle_press == 0 && last_state_toggle == 1) {
	        filter_toggles[filter_count - 1] = !filter_toggles[filter_count - 1];  // Toggle the current filter's state
	    }
	    last_state_toggle = toggle_press;
}

void ProcessFilters()
{
		        h1[2] = h1[1];
		       	h1[1] = h1[0];
		        h1[0] = LPF_norm_b0 * x[0] + LPF_norm_b1 * x[1] + LPF_norm_b2 * x[2] - LPF_norm_a1 * h1[1] - LPF_norm_a2 * h1[2];

		        h2[2] = h2[1];
		        h2[1] = h2[0];
		        h2[0] = NCH1_norm_b0 * h1[0] + NCH1_norm_b1 * h1[1] + NCH1_norm_b2 * h1[2] - NCH1_norm_a1 * h2[1] - NCH1_norm_a2 * h2[2];

		        h3[2] = h3[1];
		        h3[1] = h3[0];
		        h3[0] = PKG_norm_b0 * h2[0] + PKG_norm_b1 * h2[1] + PKG_norm_b2 * h2[2] - PKG_norm_a1 * h3[1] - PKG_norm_a2 * h3[2];

		        z[2] = z[1];
		        z[1] = z[0];
		       	z[0] = HPF_norm_b0 * h3[0] + HPF_norm_b1 * h3[1] + HPF_norm_b2 * h3[2] - HPF_norm_a1 * z[1] - HPF_norm_a2 * z[2];

}

void reverb()
{
    // --- Reverb Processing ---
   	      // Process the comb filters (using separate indices and buffers)
   	     // Comb Filter 1:
   	     Cout_1 = Cbuff_1[C_index_1];
   	     Cbuff_1[C_index_1] = z[0] + Cout_1 * Cgain_1;
   	     C_index_1++;
   	     if (C_index_1 >= Cdelay_1) { C_index_1 = 0; }

   	     // Comb Filter 2:
   	     Cout_2 = Cbuff_2[C_index_2];
   	     Cbuff_2[C_index_2] = z[0] + Cout_2 * Cgain_2;
   	     C_index_2++;
   	     if (C_index_2 >= Cdelay_2) { C_index_2 = 0; }

   	     // Comb Filter 3:
   	     Cout_3 = Cbuff_3[C_index_3];
   	     Cbuff_3[C_index_3] = z[0] + Cout_3 * Cgain_3;
   	     C_index_3++;
   	     if (C_index_3 >= Cdelay_3) { C_index_3 = 0; }

   	     // Comb Filter 4:
   	     Cout_4 = Cbuff_4[C_index_4];
   	     Cbuff_4[C_index_4] = z[0] + Cout_4 * Cgain_4;
   	     C_index_4++;
   	     if (C_index_4 >= Cdelay_4) { C_index_4 = 0; }

   	     // Combine the outputs of the comb filters:
   	     Csum = (Cout_1 + Cout_2 + Cout_3 + Cout_4) * 0.25f;

   	      // Process the all-pass filters:
   	      // All-pass Filter 1:
   	      Aout_1 = Abuff_1[A_index_1] - Csum * Again_1;
   	      Abuff_1[A_index_1] = Csum + Aout_1 * Again_1;
   	      A_index_1++;
   	      if (A_index_1 >= Adelay_1) { A_index_1 = 0; }

   	      // All-pass Filter 2:
   	      Aout_2 = Abuff_2[A_index_2] - Aout_1 * Again_2;
   	      Abuff_2[A_index_2] = Aout_1 + Aout_2 * Again_2;
   	      A_index_2++;
   	      if (A_index_2 >= Adelay_2) { A_index_2 = 0; }

   	      // All-pass Filter 3:
   	      Aout_3 = Abuff_3[A_index_3] - Aout_2 * Again_3;
   	      Abuff_3[A_index_3] = Aout_2 + Aout_3 * Again_3;
   	      A_index_3++;
   	      if (A_index_3 >= Adelay_3) { A_index_3 = 0; }
}

void FilterToggle()
{
		       	// Biquad Low Pass Filter
		       	if (filter_toggles[0]) {
		       		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);  // Turn on Low-Pass LED
		       	    h1[2] = h1[1];
		       	    h1[1] = h1[0];
		       	    h1[0] = LPF_norm_b0 * x[0] + LPF_norm_b1 * x[1] + LPF_norm_b2 * x[2] - LPF_norm_a1 * h1[1] - LPF_norm_a2 * h1[2];

		       	} else {
		       		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);  // Turn off Low-Pass LED
		       	    h1[2] = x[2];  // Bypass filter by passing input directly
		       	    h1[1] = x[1];
		       	    h1[0] = x[0];
		       	}

		       	// Biquad Notch Filter
		       	if (filter_toggles[1]) {
		       	    HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);  // Turn on Notch LED

		       	    h2[2] = h2[1];
		       	    h2[1] = h2[0];
		       	    h2[0] = NCH1_norm_b0 * h1[0] + NCH1_norm_b1 * h1[1] + NCH1_norm_b2 * h1[2] - NCH1_norm_a1 * h2[1] - NCH1_norm_a2 * h2[2];

		       	} else {
		       	    HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);  // Turn off Notch LED
		       	    h2[2] = h1[2];  // Bypass filter
		       	    h2[1] = h1[1];
		       	    h2[0] = h1[0];
		       	}

		       	// Biquad Peak Filter
		       	if (filter_toggles[2]) {
		       	    HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);  // Turn on Peak LED
		       	    h3[2] = h3[1];
		       	    h3[1] = h3[0];
		       	    h3[0] = PKG_norm_b0 * h2[0] + PKG_norm_b1 * h2[1] + PKG_norm_b2 * h2[2] - PKG_norm_a1 * h3[1] - PKG_norm_a2 * h3[2];

		       	} else {
		       	    HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);  // Turn off Peak LED
		       	    h3[2] = h2[2];  // Bypass filter
		       	    h3[1] = h2[1];
		       	    h3[0] = h2[0];
		       	}

		       	// Biquad High Pass Filter
		       	if (filter_toggles[3]) {
		       	    HAL_GPIO_WritePin(GPIOG, LD4_Pin, GPIO_PIN_SET);  // Turn on High-Pass LED

		       	    z[2] = z[1];
		       	    z[1] = z[0];
		       	    z[0] = HPF_norm_b0 * h3[0] + HPF_norm_b1 * h3[1] + HPF_norm_b2 * h3[2] - HPF_norm_a1 * z[1] - HPF_norm_a2 * z[2];

		       	} else {
		       	    HAL_GPIO_WritePin(GPIOG, LD4_Pin, GPIO_PIN_RESET);  // Turn off High-Pass LED
		       	    z[2] = h3[2];  // Bypass filter
		       	    z[1] = h3[1];
		       	    z[0] = h3[0];
		       	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        char message[100]={'\0'};//Null character '\0'
        int cnt_value = __HAL_TIM_GET_COUNTER(&htim1);
        sprintf(message, "%d", cnt_value);  // Convert integer to string
        //HAL_UART_Transmit(&huart3,(uint8_t)message, sizeof(message), 100);
        HAL_UART_Transmit_IT(&huart3, (uint8_t *)message, 100);

    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	   if (hadc->Instance == ADC1) {

		    buttonDebouncer();
		    ProcessFilters();
		    reverb();
		    FilterToggle();
		    //LED used to check duty cycle and ADC utilization
		    HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
	        // Get ADC result
	        adc_result = HAL_ADC_GetValue(hadc);

	        // Shift previous input and output samples

	        // Combined LED control and filter logic
	        // Biquad Low Pass Filter

	    	      x[2] = x[1];
	    		  x[1] = x[0];
	    		  x[0] = adc_result - 2048;
	     	      // Mix the dry filtered signal with the reverb effect:
	     	      y[2] = y[1];
	     	      y[1] = y[0];
	     	      y[0] = z[0] * (1.0 - wet) + Aout_3 * wet;


	     	HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);

	     	//send the value to the dac
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (y[0] + 2048));



	    }
}


// ADC1 interrupt handler
void ADC1_IRQHandler(void) {
    HAL_ADC_IRQHandler(&hadc1); // Call HAL handler to process interrupt
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART3_UART_Init(void);
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
  LPF();
  HPF();
  Notch();
  PEAK();
  Delay();


 	   //delay();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM8_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_IT(&hadc1);
  HAL_TIM_Base_Start(&htim8);

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim6);

  GPIOB->MODER &= ~(0x3 << (0 * 2)); // Clear mode for PB0
  GPIOB->MODER |= (0x1 << (0 * 2));  // Set PB0 to output mode (01)



  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

  //dac_voltage = dac_result /convert;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 21600 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 10800-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 10800 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 10800-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 10800 - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 100-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 45-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LD4_Pin|USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD5_Pin */
  GPIO_InitStruct.Pin = LD5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /**/
  HAL_I2CEx_EnableFastModePlus(SYSCFG_PMC_I2C_PB7_FMP);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Global ADC Callback

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
