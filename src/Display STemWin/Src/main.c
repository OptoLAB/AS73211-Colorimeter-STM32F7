
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2020 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "stm32f7xx_hal.h"
#include "STMPE610.h"
#include "SSD1963.h"
#include "GUI.h"
#include "Window1DLG.h"
#include "Window2DLG.h"
#include "AS73211.h"
#include "DIALOG.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "GUI_Type.h"
#include "WM.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
WM_HWIN hWin;

int keyPressed=0;

uint8_t Divider_,Time_,Gain_,Freq_; //vrednosti ocitane sa slajdera
uint8_t Div_map,Gain_map,Time_map,Freq_map; //vrednosti slajdera posle kalibracije
int8_t Start=0;
int8_t Stop=1; // flagovi za dugmad
int8_t m=0;

uint16_t cie_x;
uint16_t cie_y;

uint16_t X_data,Y_data,Z_data;
char X_DATA[20];
char Y_DATA[20];
char Z_DATA[20];


float temperature;
char TEMPERATURA[20];

float Ee_X_data,Ee_Y_data,Ee_Z_data;
char Ee_X_DATA[20];
char Ee_Y_DATA[20];
char Ee_Z_DATA[20];

float x_malo,y_malo,z_malo; //malo x..
char malo_x[10];//za ispis malog x
char malo_y[10];
char malo_z[10];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
extern volatile GUI_TIMER_TIME OS_TimeMS;
void HAL_SYSTICK_Callback(void)
{
	OS_TimeMS++;
}

uint8_t Divider_kalibracija(uint8_t div_raw);
uint8_t Gain_kalibracija(uint8_t gain_raw);
uint8_t Time_kalibracija(uint8_t Time_raw);
uint8_t Freq_kalibracija(uint8_t Freq_raw);
void reverse(char* str, int len);
void ftoa(float n, char* res, int afterpoint);
int intToStr(int x, char str[], int d);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  Init_LCD_GPIO();
  Init_TOUCH_GPIO(hi2c1);
  STMPE610_Init();

  WM_SetCreateFlags(WM_CF_MEMDEV); // eliminise flickering
  GUI_Init();
  GUI_Clear();

  hWin=CreateWindow1();
  GUI_Exec();

  if(AS73211_init(hi2c2)) CDC_Transmit_FS((uint8_t*)"Device connected!\n",18);
  else CDC_Transmit_FS((uint8_t*)"Device not found!\n",18);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  STMPE610_read_xyz();
	  GUI_TOUCH_Exec();
	  GUI_Delay(20);
	  keyPressed=GUI_GetKey();

	  if(Stop==1)
	  {
	  	Divider_=SLIDER_GetValue(WM_GetDialogItem(hWin,ID_SLIDER_0));
	  	Gain_=SLIDER_GetValue(WM_GetDialogItem(hWin, ID_SLIDER_1));
	  	Time_=SLIDER_GetValue(WM_GetDialogItem(hWin,ID_SLIDER_2));
	  	Freq_=LISTBOX_GetSel(WM_GetDialogItem(hWin,ID_LISTBOX_0));

	  	Div_map=Divider_kalibracija(Divider_);
	  	Gain_map=Gain_kalibracija(Gain_);
	  	Time_map=Time_kalibracija(Time_);
	  	Freq_map=Freq_kalibracija(Freq_);
	  	CDC_Transmit_FS("Ocitao slajdere!\n",17);

	  	EDIT_SetText(WM_GetDialogItem(hWin,ID_EDIT_20),malo_x);
	  	EDIT_SetText(WM_GetDialogItem(hWin,ID_EDIT_21),malo_y);
	  	EDIT_SetText(WM_GetDialogItem(hWin,ID_EDIT_22),malo_z);


	  }


	  switch(keyPressed)
	  {
	  	  case ID_BUTTON_1:
	      CDC_Transmit_FS("Start\n",6);
	  	  Start=1;
	  	  Stop=0;

	  	  	  if(m==0)
	  	  	  {
	  	  		  HAL_Delay(100);
	  	  		  AD73211_startConfiguration();
	  	  		  AS73211_writeByte(AS73211_CFG1_REG, Gain_map|Time_map);
	  	  		  AS73211_writeByte(AS73211_CFG2_REG, 0x80| 0x00|0x08|Div_map);
	  	  		  AS73211_writeByte(AS73211_CFG3_REG, 0x40);
	  	  		  AS73211_writeByte(AS73211_BREAK_REG, 0xff);
	  	  		  AS73211_writeByte(AS73211_EDGES_REG, 0x01);
	  	  		  HAL_Delay(100);
	  	  		  m=1;
	  	  	  	}
	     	break;

	  	  case ID_BUTTON_2:

	  		  	  CDC_Transmit_FS("Stop\n",5);
	  		  	  Start=0;
	  		  	  Stop=1;
	  		  	  m=0;
	  		break;

	  	 case ID_BUTTON_0:

	  		 CDC_Transmit_FS("Next\n",5);
	  		 hWin=CreateWindow2();
	  		 GUI_Exec();
	  		 HAL_Delay(300);
	  		 break;

//previous
	  	 case ID_BUTTON_20:
	    	 CDC_Transmit_FS("Previous\n",9);
	  		 hWin=CreateWindow1();
	  		 GUI_Exec();
	  		 HAL_Delay(300);
	  		 break;

	  }


	  if(Start==1 && Stop==0 && m==1)
	  	  {
		  	  CDC_Transmit_FS("Measurement\n",12);
	  		  AD73211_startMeasurement();
	  		  HAL_Delay(500); // ovaj delay bi trebalo da odgovara vremenu konverzije

	  /*					citanje X,Y,Z kordinata								*/
	  		  X_data=AD73211_readXYZChannel(AS73211_X_CHANNEL);
	  		  Y_data=AD73211_readXYZChannel(AS73211_Y_CHANNEL);
	  		  Z_data=AD73211_readXYZChannel(AS73211_Z_CHANNEL);

	  		  temperature = color6_getTemperature();
	  		  ftoa(temperature,TEMPERATURA, 4);

	  		  sprintf(X_DATA,"%d",X_data);
	  		  sprintf(Y_DATA,"%d",Y_data);
	  		  sprintf(Z_DATA,"%d",Z_data);

	  		  EDIT_SetText(WM_GetDialogItem(hWin,ID_EDIT_0),X_DATA);
	  		  EDIT_SetText(WM_GetDialogItem(hWin,ID_EDIT_1),Y_DATA);
	  		  EDIT_SetText(WM_GetDialogItem(hWin,ID_EDIT_2),Z_DATA);
	  		  EDIT_SetText(WM_GetDialogItem(hWin,ID_EDIT_3),TEMPERATURA);

	  /*					racunanje Ee vrednosti							*/
	  		  Ee_X_data=color6_convertingToEe(AS73211_X_CHANNEL, X_data);
	  		  Ee_Y_data=color6_convertingToEe(AS73211_Y_CHANNEL, Y_data);
	  		  Ee_Z_data=color6_convertingToEe(AS73211_Z_CHANNEL, Z_data);

	  		  ftoa(Ee_X_data,Ee_X_DATA, 4);
	  		  ftoa(Ee_Y_data,Ee_Y_DATA, 4);
	  		  ftoa(Ee_Z_data,Ee_Z_DATA, 4);

	  		  EDIT_SetText(WM_GetDialogItem(hWin,ID_EDIT_4),Ee_X_DATA);
	  		  EDIT_SetText(WM_GetDialogItem(hWin,ID_EDIT_5),Ee_Y_DATA);
	  		  EDIT_SetText(WM_GetDialogItem(hWin,ID_EDIT_6),Ee_Z_DATA);

	  /*					racunanje malih x,y,z							*/

	  		 x_malo=(float)X_data/(X_data+Y_data+Z_data);
	  		 ftoa(x_malo,malo_x, 4);

	  		 y_malo=(float)Y_data/(X_data+Y_data+Z_data);
	  		 ftoa(y_malo,malo_y, 4);

	  	     z_malo=(float)Z_data/(X_data+Y_data+Z_data);
	  		 ftoa(z_malo,malo_z, 4);

	  		 EDIT_SetText(WM_GetDialogItem(hWin,ID_EDIT_20),malo_x);
	  		 EDIT_SetText(WM_GetDialogItem(hWin,ID_EDIT_21),malo_y);
	  		 EDIT_SetText(WM_GetDialogItem(hWin,ID_EDIT_22),malo_z);


	  		 cie_x=(uint16_t)(x_malo*308/0.75);
	  		 cie_y=(uint16_t)(272-y_malo*272/0.85);


	  		 GUI_RECT r={cie_x-100,cie_y-100,cie_x+100,cie_y+100};
	  		 WM_InvalidateRect(hWin,&r);




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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x007074AF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x007074AF;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PG15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint8_t Divider_kalibracija(uint8_t div_raw)
{
	 uint8_t provera=div_raw;
	 uint8_t rezultat=0;

	 if(provera==0)
	 {
		 rezultat=0x00;
	 }
	 else if(provera>0 && provera<=15)
	 {
		 rezultat=0x01;
	 }
	 else if(provera>15 && provera<=30)
	 {
	 	 rezultat=0x02;
	 }

	 else if(provera>30 && provera<=45)
	 {
		 rezultat=0x03;
	 }
	 else if(provera>45 && provera<=60)
	 {
		 rezultat=0x04;
	 }

	 else if(provera>60 && provera<75)
     {
		 rezultat=0x05;
	}
	 else if(provera>75 && provera<=90)
	{
		 rezultat=0x06;
	}

	 else if(provera>90)
	 {
		 rezultat=0x07;
	 }

	 return rezultat;
}

uint8_t Gain_kalibracija(uint8_t gain_raw)
{
	 uint8_t provera=gain_raw;
	 uint8_t rezultat=0xB0;

	  if(provera>=0 && provera<=7)
	 {
		  rezultat=0xB0;
	 }
	 else if(provera>7 && provera<=15)
	 {
		 rezultat=0xA0;
	 }

	 else if(provera>15 && provera<=22)
	 {
		 rezultat=0x90;
	 }
	 else if(provera>22 && provera<=30)
	 {
		 rezultat=0x80;
	 }

	 else if(provera>30 && provera<37)
     {
		 rezultat=0x70;
	}
	 else if(provera>37 && provera<=45)
	{
		 rezultat=0x60;
	}

	 else if(provera>45 && provera<=52)
	 {
		 rezultat=0x50;
	 }

	 else if(provera>52 && provera<=60)
	 {
		 rezultat=0x40;
	 }
	 else if(provera>60 && provera<=68)
	{
		 rezultat=0x30;
	}
	 else if(provera>68 && provera<=75)
	{
		 rezultat=0x20;
	}
	 else if(provera>75 && provera<=85)
	{
		 rezultat=0x10;
	}

	 else if(provera>=85)
	 {
		 rezultat=0x00;
	 }
	  return rezultat;
}

uint8_t Time_kalibracija(uint8_t Time_raw)
{
	 uint8_t provera=Time_raw;
	 uint8_t rezultat=0;
	  if(provera>=0 && provera<=6)
	 {
		  rezultat=0x00;
	 }
	 else if(provera>6 && provera<=12)
	 {
		 rezultat=0x01;
	 }

	 else if(provera>12 && provera<=18)
	 {
		 rezultat=0x02;
	 }
	 else if(provera>18 && provera<=24)
	 {
		 rezultat=0x03;
	 }

	 else if(provera>24 && provera<30)
     {
		 rezultat=0x04;
	}
	 else if(provera>30 && provera<=36)
	{
		 rezultat=0x05;
	}

	 else if(provera>36 && provera<=42)
	 {
		 rezultat=0x06;
	 }

	 else if(provera>42 && provera<=48)
	 {
		 rezultat=0x07;
	 }
	 else if(provera>48 && provera<=54)
	{
		 rezultat=0x08;
	}
	 else if(provera>54 && provera<=60)
	{
		 rezultat=0x09;
	}

	else if(provera>60 && provera<=66)
	{
		 rezultat=0x0A;
	}

	else if(provera>66 && provera<=72)
	{
		rezultat=0x0B;
	}

	else if(provera>72 && provera<=80)
	{
		rezultat=0x0C;
	}

	 else if(provera>80 && provera<=90)
	{
		 rezultat=0x0D;
	}
	 else if(provera>=90)
	 {
		 rezultat=0x0E;
	}
	  return rezultat;
}

uint8_t Freq_kalibracija(uint8_t Freq_raw)
{
	uint8_t provera=Freq_raw;
	uint8_t rezultat=0;

	if(provera==0)
	{
		rezultat=0x00;
	}
	else if(provera==1)
	{
		rezultat=0x01;
	}
	else if(provera==2)
	{
		rezultat=0x02;
	}
	else if(provera==3)
	{
		rezultat=0x03;
	}
	return rezultat;
}

void reverse(char* str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

// Converts a given integer x to string str[].
// d is the number of digits required in the output.
// If d is more than the number of digits in x,
// then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x) {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

// Converts a floating-point/double number to a string.
void ftoa(float n, char* res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0) {
        res[i] = '.'; // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter
        // is needed to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}






/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
