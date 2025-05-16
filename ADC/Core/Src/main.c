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
typedef struct {
 	uint32_t cnt_value;
 	uint32_t cnt_max;
 	uint32_t cnt_min;
 	uint32_t cnt_mid;
 	uint32_t cnt_store;
 	uint8_t state;
 	uint8_t prev_state;
 } EncoderConfig;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.14159
#define NUM_ENCODERS 4
#define NUM_FILTERS 4
#define BUFFER_SIZE 4
#define FREQ_SAMPLE 46875

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockB1;
DMA_HandleTypeDef hdma_sai1_a;
DMA_HandleTypeDef hdma_sai1_b;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

 int i = 0;
 char message[16] = {'\0'};
 int temp = 0;
 int selected_filter = 0;

 static int last_state_filter = 1;
 static int last_state_toggle = 1;
 static volatile int filter_count = 0;       // Tracks selected filter (1 to 4)
 static bool filter_toggles[NUM_FILTERS] = {0,0,0,0};

 EncoderConfig encoders[NUM_ENCODERS] = {
 		{ 0, 10000, 0, 5000, 7},
 		{ 0, 10000, 0, 5000, 7},
 		{ 0, 10000, 0, 5000, 7},
 		{ 0, 10000, 0, 5000, 7}
 };

//buffers for adc input and output
//static int y[3] = {0, 0, 0};


uint32_t adcData[BUFFER_SIZE] = {0};
uint32_t dacData[BUFFER_SIZE] = {0};

static volatile uint32_t *inBufPtr;
static volatile uint32_t *outBufPtr = &dacData[0];

static uint32_t leftIn0, rightIn0 = 0;

static int16_t lSample = 0;
static int16_t rSample = 0;

float Fs = 46875.0;

float volatile filter_f0[NUM_FILTERS] = {2000, 1000, 14000, 60};  // LPF, Notch, Peak, HPF
float volatile filter_Q[NUM_FILTERS]  = {0.5, 5, 5, 0.5};  // Q values for all

/*
// High Cut / Low Pass Filter Settings
int16_t LPF_f0 = 20000;                         // Center / Corner Frequency
float LPF_Q = 1;                          	// Quality Factor

// Low Cut / High Pass Filter Settings
int16_t HPF_f0 = 20;                            // Center / Corner Frequency
float HPF_Q = 1;                         		// Quality Factor

// Notch Filter Settings
int16_t NCH1_f0 = 14000;                          // Center / Corner Frequency
float NCH1_Q = 10;                        		// Quality Factor


// Peaking Filter
int16_t PKG_f0 = 60;                           // Center / Corner Frequency
float PKG_Q = 10;                         		// Quality Factor
float PKG_GAIN = 1;								// Attenuation
*/

float wet = 0.0;
float delay = 1.0;

static int16_t x[3] = {0, 0, 0};
static int16_t  outputBuf[3] = {0, 0, 0};


//buffers for filters
static int16_t h1[3] = {0, 0, 0};
static int16_t h2[3] = {0, 0, 0};
static int16_t h3[3] = {0, 0, 0};
static int16_t z[3] = {0, 0, 0};


// Delay Indices for Comb Filters
static int16_t C_index_1 = 1;
static int16_t C_index_2 = 1;
static int16_t C_index_3 = 1;
static int16_t C_index_4 = 1;

// Delay Indices for All-pass Filters
static int16_t A_index_1 = 1;
static int16_t A_index_2 = 1;
static int16_t A_index_3 = 1;

// Reverb Parameters

float Cgain_1 = 0.805, Cgain_2 = 0.827, Cgain_3 = 0.783, Cgain_4 = 0.764; // Reverb gains

int16_t Csum = 0, Aout_1 = 0, Aout_2 = 0, Aout_3 = 0;  // Intermediate reverb variables
int16_t Cout_1 = 0, Cout_2 = 0, Cout_3 = 0, Cout_4 = 0;

int16_t Clim_1 = 0, Clim_2 = 0, Clim_3 = 0, Clim_4 = 0;
int16_t Alim_1 = 0, Alim_2 = 0, Alim_3 = 0;

int16_t Adelay_1 = 220;
float Again_1 = 0.7;
int16_t Adelay_2 = 74;
float Again_2 = 0.7;
int16_t Adelay_3 = 21;
float Again_3 = 0.7;
int16_t Cdelay_1 = 1590;
int16_t  Cdelay_2 = 1372;
int16_t  Cdelay_3 = 1782;
int16_t Cdelay_4 = 1980;

int Cgain_sum = 0.25;


// Comb Filter Buffers
static int16_t Cbuff_1[1590] = {0};
static int16_t Cbuff_2[1372] = {0};
static int16_t Cbuff_3[1782] = {0};
static int16_t Cbuff_4[1980] = {0};

// All-pass Filter Buffers
static int16_t Abuff_1[220] = {0};
static int16_t Abuff_2[74] = {0};
static int16_t Abuff_3[21] = {0};



static float PKG_dB_gain = 1.0;                       // Gain in dB

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
		//HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(GPIOG, LD4_Pin, GPIO_PIN_SET);
	 	//LPF_w0 = 2 * PI * (LPF_f0 / Fs);    	// Cutoff Freq in radians
	    LPF_w0 = 2 * PI * (filter_f0[0] / Fs);    	// Cutoff Freq in radians
	    LPF_cos_w0 = cos(LPF_w0);           	// cosine omega
	    LPF_sin_w0 = sin(LPF_w0);           	// sine omega
	    LPF_alpha = LPF_sin_w0 / (2 * filter_Q[0]);
	   // LPF_alpha = LPF_sin_w0 / (2 * LPF_Q);

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
	   // HAL_GPIO_WritePin(GPIOG, LD4_Pin, GPIO_PIN_RESET);
	   // HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
}

void Notch() {
		//HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
	 	//NCH1_w0 = 2 * PI * (NCH1_f0 / Fs);
	    NCH1_w0 = 2 * PI * (filter_f0[1] / Fs);      // Center Freq in radians
	    NCH1_cos_w0 = cos(NCH1_w0);             // cosine omega
	    NCH1_sin_w0 = sin(NCH1_w0);             // sine omega
	    NCH1_alpha = NCH1_sin_w0 / (2 * filter_Q[1]);
	    //NCH1_alpha = NCH1_sin_w0 / (2 * NCH1_Q);

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
	   // HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
}

void PEAK() {
	//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	 	//PKG_w0 = 2 * PI * (PKG_f0 / Fs);
	    PKG_w0 = 2 * PI * (filter_f0[2] / Fs);    	// Center Freq in radians
	    PKG_A = PKG_dB_gain;      	// Linear Gain from dB
	    PKG_cos_w0 = cos(PKG_w0);      			// cosine omega
	    PKG_sin_w0 = sin(PKG_w0);      			// sine omega
	    PKG_alpha = PKG_sin_w0 / (2 * filter_Q[2]);
	    //PKG_alpha = PKG_sin_w0 / (2 * PKG_Q);



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
	   // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
}

void HPF() {
		//HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
	 	//HPF_w0 = 2 * PI * (HPF_f0 / Fs);
	    HPF_w0 = 2 * PI * (filter_f0[3] / Fs);        // Cutoff Freq in radians
	    HPF_cos_w0 = cos(HPF_w0);               // cosine omega
	    HPF_sin_w0 = sin(HPF_w0);               // sine omega
	    HPF_alpha = HPF_sin_w0 / (2 * filter_Q[3]);
	    //HPF_alpha = HPF_sin_w0 / (2 * HPF_Q);


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
	  //  HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
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
	    	wet = 0;
	    	if(filter_count < 3){
	    		filter_count = filter_count + 1;  // Cycle through filters

	    	}
	    	else{
	    		filter_count = 0;
	    	}
	    	selected_filter = filter_count;

	    }
	    last_state_filter = button_press;


	    // Toggle button: Toggle the corresponding filter ON/OFF
	    if (toggle_press == 0 && last_state_toggle == 1) {
	    	wet = 0;
	        filter_toggles[selected_filter] = !filter_toggles[selected_filter];  // Toggle the current filter's state
	    }
	    last_state_toggle = toggle_press;



}


void readEncoders() {
	//HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);

    for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
    	temp = filter_count;


        // Read encoder states
    	//encoders[0].state = (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) << 1) | HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);
        encoders[0].state = (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) << 1) | HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);
        encoders[1].state = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) << 1) | HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
        encoders[2].state = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) << 1) | HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
        encoders[3].state = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) << 1) | HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13);

        if ((encoders[i].prev_state == 0b00 && encoders[i].state == 0b10) ||
            (encoders[i].prev_state == 0b10 && encoders[i].state == 0b11) ||
            (encoders[i].prev_state == 0b11 && encoders[i].state == 0b01) ||
            (encoders[i].prev_state == 0b01 && encoders[i].state == 0b00)) {

            if (encoders[i].cnt_value < encoders[i].cnt_max) {
                encoders[i].cnt_value++;
            } else {
                encoders[i].cnt_value = encoders[i].cnt_mid;
            }


            // **Encoder 1 (Change Frequency of Selected Filter)**
            if (i == 0) {
                filter_f0[temp] *= 0.99f;  // Decrease by 1%
                if(filter_f0[temp] <= 20.0f){
                	filter_f0[temp] = 21.0f;
                }
                sprintf(message, "F: %d\r\n", (int)filter_f0[temp]);
                HAL_UART_Transmit_IT(&huart3, (uint8_t *)message, strlen(message));
            }

            // **Encoder 2 (Change Q Factor of Selected Filter)**
            if (i == 1) {
                filter_Q[temp] -= 0.01f; // Decrease Q
                if (filter_Q[0] <= 0.5f){
                    filter_Q[0] = 0.51f; // Limit min Q factor
                }
                if (filter_Q[1] <= 2.0f){
                    filter_Q[1] = 2.01f; // Limit min Q factor
                }
                if (filter_Q[2] <= 0.5f){
                    filter_Q[2] = 0.51f; // Limit min Q factor
                }
                if (filter_Q[3] <= 0.5f){
                    filter_Q[3] = 0.51f; // Limit min Q factor
                }
                sprintf(message, "Q: %d.%02d\r\n", (int)filter_Q[temp], (int)(filter_Q[temp] * 100) % 100);
                HAL_UART_Transmit_IT(&huart3, (uint8_t *)message, strlen(message));
            }

            // **Encoder 3 (Decrease Gain)**
            if (i == 2) {
            	PKG_dB_gain -= 0.005f; // Decrease gain in dB
            	if (PKG_dB_gain <= 0.01f)  // Limit min gain to -20 dB
            		PKG_dB_gain = 0.02f;
            	sprintf(message, "dB: %d.%02d\r\n", (int)PKG_dB_gain, (int)(PKG_dB_gain * 100) % 100);
            	HAL_UART_Transmit_IT(&huart3, (uint8_t *)message, strlen(message));
            }

            // **Encoder 4 (Decrease Wet Mix)**
            if (i == 3) {
            	wet -= 0.01f; // Decrease wet mix
            	if (wet <= 0.01f)  // Limit min wet mix to 0 (dry)
            		wet = 0.0f;
            	sprintf(message, "W: %d.%02d\r\n", (int)wet, (int)(wet * 100) % 100);
            	HAL_UART_Transmit_IT(&huart3, (uint8_t *)message, strlen(message));
            }



        }

        else if ((encoders[i].prev_state == 0b00 && encoders[i].state == 0b01) ||
                 (encoders[i].prev_state == 0b01 && encoders[i].state == 0b11) ||
                 (encoders[i].prev_state == 0b11 && encoders[i].state == 0b10) ||
                 (encoders[i].prev_state == 0b10 && encoders[i].state == 0b00)) {

            if (encoders[i].cnt_value > encoders[i].cnt_min) {
                encoders[i].cnt_value--;
            } else {
                encoders[i].cnt_value = encoders[i].cnt_mid;
            }


            // **Encoder 1 (Change Frequency of Selected Filter)**
                        if (i == 0) {
                            filter_f0[temp] *= 1.01f;  // Increase by 1%
                            if (filter_f0[temp] >= 20000.0f){
                                filter_f0[temp] = 19999.0f; // Limit max Q factor
                            }
                            sprintf(message, "F: %d\r\n", (int)filter_f0[temp]);
                            HAL_UART_Transmit_IT(&huart3, (uint8_t *)message, strlen(message));
                        }

                        // **Encoder 2 (Change Q Factor of Selected Filter)**

                        if (i == 1) {
                            filter_Q[temp] += 0.01f; // Increase Q
                            if (filter_Q[0] >= 1.41f){
                                filter_Q[0] = 1.40f; // Limit max Q factor
                            }
                            if(filter_Q[1] >= 10.0f){
                            	filter_Q[1] = 9.9f;
                            }
                            if(filter_Q[2] >= 20.0f){
                               filter_Q[2] = 19.9f;
                            }
                            if(filter_Q[3] >= 1.41f){
                               filter_Q[3] = 1.40f;
                            }
                            sprintf(message, "Q: %d.%02d\r\n", (int)filter_Q[temp], (int)(filter_Q[temp] * 100) % 100);
                            HAL_UART_Transmit_IT(&huart3, (uint8_t *)message, strlen(message));
                       }


                        // **Encoder 3 (Increase Gain)**
                        if (i == 2) {
                        	PKG_dB_gain += 0.005f; // Increase gain in dB
                        	if (PKG_dB_gain >= 1.0f)  // Limit max gain to 12 dB
                        		PKG_dB_gain = 0.99f;
                        	sprintf(message, "dB: %d.%02d\r\n", (int)PKG_dB_gain, (int)(PKG_dB_gain * 100) % 100);
                        	HAL_UART_Transmit_IT(&huart3, (uint8_t *)message, strlen(message));
                        }


                        // **Encoder 4 (Increase Wet Mix)**

                        if (i == 3) {
                        	wet += 0.01f; // Increase wet mix
                        	if (wet >= 0.99f)  // Limit max wet mix to 1.0 (100%)
                        		wet = 1.0f;
                        	sprintf(message, "W: %d.%02d\r\n", (int)wet, (int)(wet * 100) % 100);
                        	HAL_UART_Transmit_IT(&huart3, (uint8_t *)message, strlen(message));
                        }


        }

        encoders[i].prev_state = encoders[i].state;
    }
    //HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
}


void FilterToggle()
{

    // Low-Pass Filter (PB0)
    if (filter_toggles[0]) {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);


        h1[2] = h1[1];
        h1[1] = h1[0];
        h1[0] = LPF_norm_b0 * x[0] + LPF_norm_b1 * x[1] + LPF_norm_b2 * x[2] - LPF_norm_a1 * h1[1] - LPF_norm_a2 * h1[2];

    }
    else {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

        h1[2] = h1[1];
        h1[1] = h1[0];
        h1[0] = x[0];

    }

    // Notch Filter (PB7)
    if (filter_toggles[1]) {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

        h2[2] = h2[1];
        h2[1] = h2[0];
        h2[0] = NCH1_norm_b0 * h1[0] + NCH1_norm_b1 * h1[1] + NCH1_norm_b2 * h1[2] - NCH1_norm_a1 * h2[1] - NCH1_norm_a2 * h2[2];

    }
    else {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);


        h2[2] = h2[1];
        h2[1] = h2[0];
        h2[0] = h1[0];

    }

    // Peak Filter (PG0)
    if (filter_toggles[2]) {
    	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

        h3[2] = h3[1];
        h3[1] = h3[0];
        h3[0] = PKG_norm_b0 * h2[0] + PKG_norm_b1 * h2[1] + PKG_norm_b2 * h2[2] - PKG_norm_a1 * h3[1] - PKG_norm_a2 * h3[2];

    }
    else {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);


        h3[2] = h3[1];
        h3[1] = h3[0];
        h3[0] =  h2[0];


    }

    // High-Pass Filter (PF0)
    if (filter_toggles[3]) {
    	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);

        z[2] = z[1];
        z[1] = z[0];
        z[0] = HPF_norm_b0 * h3[0] + HPF_norm_b1 * h3[1] + HPF_norm_b2 * h3[2] - HPF_norm_a1 * z[1] - HPF_norm_a2 * z[2];
    }
    else {
    	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);



        z[2] = z[1];
        z[1] = z[0];
        z[0] = h3[0];

    }
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
   	     Cbuff_1[C_index_1] = x[0] + Cout_1 * Cgain_1;
   	     C_index_1++;
   	     if (C_index_1 >= Cdelay_1) { C_index_1 = 0; }

   	     // Comb Filter 2:
   	     Cout_2 = Cbuff_2[C_index_2];
   	     Cbuff_2[C_index_2] = x[0] + Cout_2 * Cgain_2;
   	     C_index_2++;
   	     if (C_index_2 >= Cdelay_2) { C_index_2 = 0; }

   	     // Comb Filter 3:
   	     Cout_3 = Cbuff_3[C_index_3];
   	     Cbuff_3[C_index_3] = x[0] + Cout_3 * Cgain_3;
   	     C_index_3++;
   	     if (C_index_3 >= Cdelay_3) { C_index_3 = 0; }

   	     // Comb Filter 4:
   	     Cout_4 = Cbuff_4[C_index_4];
   	     Cbuff_4[C_index_4] = x[0] + Cout_4 * Cgain_4;
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



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim->Instance == TIM6)
    {


    	readEncoders();


    	buttonDebouncer();


    	switch (selected_filter)
    	    		{
    	    		case 0:
    	    			LPF();
    	    		case 1:
    	    			Notch();
    	    		case 2:
    	    			PEAK();
    	    		case 3:
    	    			HPF();
    	    		}


     }


   //sprintf(message, "%d.%02d ", (int)filter_f0[filter_count], (int)(filter_f0[filter_count] * 100) % 100);
   //HAL_UART_Transmit_IT(&huart3, (uint8_t *)message, strlen(message));
    //HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
    //sprintf(message, "%lu ", encoders[i].cnt_value); //encoders[i].cnt_value);  // Convert integer to string
    //HAL_UART_Transmit_IT(&huart3, (uint8_t *)message, 4);

}


void processData(){



	// Read Data In
	//HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);

	// Convert to 2's Comp * 2^7
	leftIn0 	= (int16_t) ((inBufPtr[0] >> 8) & 0x0000FFFF);
	rightIn0 	= (int16_t) ((inBufPtr[1] >> 8) & 0x0000FFFF);

	// Sum to mono
	lSample = (int32_t)leftIn0 >> 1;
	rSample = (int32_t)leftIn0 >> 1;

	int16_t monoSample = lSample + rSample;

	x[2] = x[1];
	x[1] = x[0];
	x[0] = monoSample;


	if(wet < 0.01){
		FilterToggle();
		outputBuf[2] = outputBuf[1];
		outputBuf[1] = outputBuf[0];
		outputBuf[0] = z[0];

	}
	else {
		reverb();
		outputBuf[2] = outputBuf[1];
		outputBuf[1] = outputBuf[0];
	    outputBuf[0] = x[0] * (1.0 - wet) + Aout_3 * wet;
	}



	outBufPtr[0] = (uint32_t)(outputBuf[0] << 8) & 0x00FFFF00;
	outBufPtr[1] = (uint32_t)(outputBuf[0] << 8) & 0x00FFFF00;

	//HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);

}

// SAI TX and RX Are Synchronized, only use the RX callback to process Data

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai) {  	// First Half of Data is ready for processing
    // Handle DAC data refill or processing
	//HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  // Turn on PB0 LED

	inBufPtr 			= &adcData[0];
	outBufPtr			= &dacData[0];
	processData();

	//HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
}


void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai) {			//Second Half of Data is ready for processing
	// Handle DAC data refill or processing
	//HAL_GPIO_WritePin(GPIOG, LD4_Pin, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);  // Turn on PA5 LED

	inBufPtr 			= &adcData[BUFFER_SIZE/2];
	outBufPtr			= &dacData[BUFFER_SIZE/2];

	processData();

	//HAL_GPIO_WritePin(GPIOG, LD4_Pin, GPIO_PIN_RESET);
}


// ADC1 interrupt handler

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SAI1_Init(void);
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */



 	   Delay();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_SAI1_Init();
  /* USER CODE BEGIN 2 */
  //HAL_ADC_Start_IT(&hadc1);
  HAL_TIM_Base_Start(&htim8);


  HAL_TIM_Base_Start_IT(&htim6);

 // GPIOB->MODER &= ~(0x3 << (0 * 2)); // Clear mode for PB0
 // GPIOB->MODER |= (0x1 << (0 * 2));  // Set PB0 to output mode (01)


  //HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

  HAL_SAI_Transmit_DMA(&hsai_BlockB1, (uint8_t*) dacData, BUFFER_SIZE);
  HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t*) adcData, BUFFER_SIZE);

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
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 96;
  PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLP_DIV2;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
  PeriphClkInitStruct.PLLI2SDivQ = 1;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLI2S;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_RX;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  if (HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_24BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  hsai_BlockB1.Instance = SAI1_Block_B;
  hsai_BlockB1.Init.AudioMode = SAI_MODESLAVE_TX;
  hsai_BlockB1.Init.Synchro = SAI_SYNCHRONOUS;
  hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockB1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_24BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

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
  htim6.Init.Period = 10-1;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|GPIO_PIN_12|GPIO_PIN_13|LD3_Pin
                          |GPIO_PIN_5|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LD4_Pin|USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD5_Pin */
  GPIO_InitStruct.Pin = LD5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin PB12 PB13 LD3_Pin
                           PB5 LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|GPIO_PIN_12|GPIO_PIN_13|LD3_Pin
                          |GPIO_PIN_5|LD2_Pin;
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

  /*Configure GPIO pins : PE9 PE11 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

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
