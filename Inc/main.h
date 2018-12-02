/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#define USBD_DEVICE_VER_MAJ	0x00
#define USBD_DEVICE_VER_MIN	0x22
#include <key_define.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "string.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_dma.h"
#include "stm32f0xx_hal_tim.h"
#include "led.h"
#include "i2c-lcd.h"
#include "bitcount.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define SDA_Pin GPIO_PIN_0
#define SDA_GPIO_Port GPIOF
#define SCL_Pin GPIO_PIN_1
#define SCL_GPIO_Port GPIOF
#define M0_Pin GPIO_PIN_0
#define M0_GPIO_Port GPIOA
#define M1_Pin GPIO_PIN_1
#define M1_GPIO_Port GPIOA
#define M2_Pin GPIO_PIN_2
#define M2_GPIO_Port GPIOA
#define M3_Pin GPIO_PIN_3
#define M3_GPIO_Port GPIOA
#define ENC_A_Pin GPIO_PIN_4
#define ENC_A_GPIO_Port GPIOA
#define ENC_A_EXTI_IRQn EXTI4_15_IRQn
#define ENC_B_Pin GPIO_PIN_5
#define ENC_B_GPIO_Port GPIOA
#define ENC_B_EXTI_IRQn EXTI4_15_IRQn
#define LED_R_Pin GPIO_PIN_6
#define LED_R_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_7
#define LED_G_GPIO_Port GPIOA
#define LED_B_Pin GPIO_PIN_0
#define LED_B_GPIO_Port GPIOB
#define BL_ON_Pin GPIO_PIN_1
#define BL_ON_GPIO_Port GPIOB
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define L0_Pin GPIO_PIN_3
#define L0_GPIO_Port GPIOB
#define L1_Pin GPIO_PIN_4
#define L1_GPIO_Port GPIOB
#define L2_Pin GPIO_PIN_5
#define L2_GPIO_Port GPIOB
#define L3_Pin GPIO_PIN_6
#define L3_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

//USB descriptor definition (instead of usbd.desc.h)
#define USBD_VID     1155
#define USBD_LANGID_STRING     1033
#define USBD_MANUFACTURER_STRING     "Ruffles Inc."
#define USBD_PID_FS     3002
#define USBD_PRODUCT_STRING_FS     "LrLite"
#define USBD_SERIALNUMBER_STRING_FS     "prototype"
#define USBD_CONFIGURATION_STRING_FS     "LrLite Config"
#define USBD_INTERFACE_STRING_FS     "LrLite Keyboard"

//LrLite Ports on Board
#define Mx_GPIO_Port GPIOA
#define ENCx_GPIO_Port GPIOA
#define KEY_COUNT 16

typedef union {
    uint32_t wd;
    struct{
        unsigned char n0:4;
        unsigned char n1:4;
        unsigned char n2:4;
		unsigned char n3:4;
		unsigned char lrot:2;	//lower rotator
        unsigned int  uu:14;	//dummy
    } nb;
} KEYSCAN;

typedef struct {
    uint8_t modifier;
    uint8_t keycode;
    char	*message;
} KEY_DEFINE;

typedef struct {
	uint8_t element[4];
} KEY_MODIFIER;

//Moved From Harmony keyboard.h
typedef struct {
	uint8_t	modifier;
	uint8_t keys[4];
} KEYBOARD_INPUT_REPORT;

#define L2MASK 0x09
#define L013MASK 0x0F
#define MOD_SW_BIT_MASK    0x0003ffff
//
#define TIM_PRESC_1uS		47
#define TIM_PERIOD_125uS	125
#define TIM_PERIOD_4mS		4000
#define TIM_PERIOD_10mS		10000
//
#define TIM_PRESC_100uS		4799
#define TIM_PERIOD_1SEC		10000
//other definitions
#define LCD_TIMER_DEFAULT   1000    //4 sec (1tick=4ms)
#define LCD_TIMER_INIT      10      //40m sec initialze time
#define LCD_TIMER_UPDATE	250		//1 sec (LCD update in non HID)

#define ROT_NOT_MOVE        0
#define ROT_MOVE_CW         1
#define ROT_MOVE_CCW        2

#define HID_RPT_KEY_IDX		1

void Delay_us(uint32_t microsec);
#define TEMP110_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7C2))
#define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7B8))
#define VDD_CALIB ((uint16_t) (330))
#define VDD_APPLI ((uint16_t) (300))
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
