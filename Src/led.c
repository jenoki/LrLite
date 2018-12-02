/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */
#include "led.h"
/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */
#include "main.h"
/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */

/*  A brief description of a section can be given directly below the section
    banner.
 */
extern TIM_HandleTypeDef htim3;
uint32_t period;


void LED_Initialize(){
    period = htim3.Init.Period;
}

// *****************************************************************************

/** 
  @Function
    int ExampleInterfaceFunctionName ( int param1, int param2 ) 

  @Summary
    Brief one-line description of the function.

  @Remarks
    Refer to the example_file.h interface header for function usage details.
 */
void LED_Set(uint8_t color, uint8_t blink) {
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM2;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	if (color==LED_COLOR_OFF){
		  HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
		  HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);
		  HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_3);
		return;
	}
	switch (blink){
		case LED_BLINK_FAST:
			htim3.Init.Period = period / 2;
			sConfigOC.Pulse = period / 4;
			break;
		case LED_BLINK_SLOW:
	    	htim3.Init.Period = period;
			sConfigOC.Pulse = period / 2;
			break;
		default:	//always on
			sConfigOC.Pulse = 0;
			break;
	}
	HAL_TIM_Base_Init(&htim3);

    if (color & LED_BIT_RED) {
    	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
    	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
    }else{
    	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
    }

    if (color & LED_BIT_GREEN) {
    	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
    	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
    }else{
    	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);
    }
    if (color & LED_BIT_BLUE){
    	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
    	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
    }else{
    	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_3);
    }
}

void LED_2ToneFlash(uint8_t color1,uint8_t color2,uint8_t blink){
	TIM_OC_InitTypeDef sConfigOC1,sConfigOC2;
	bool c1,c2;
	sConfigOC1.OCMode = TIM_OCMODE_PWM1;
	sConfigOC2.OCMode = TIM_OCMODE_PWM2;
	sConfigOC1.OCPolarity = sConfigOC2.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC1.OCFastMode = sConfigOC2.OCFastMode = TIM_OCFAST_DISABLE;

	htim3.Init.Period = (blink == LED_BLINK_FAST) ? (period / 2) : period;
	HAL_TIM_Base_Init(&htim3);

	c1=(color1 & LED_BIT_RED) ? true:false;
	c2=(color2 & LED_BIT_RED) ? true:false;
    if (c1 || c2) {
    	switch (blink){
    		case LED_BLINK_FAST:
    			sConfigOC1.Pulse = sConfigOC2.Pulse = period / 4;
    			break;
    		case LED_BLINK_SLOW:
    			sConfigOC1.Pulse = sConfigOC2.Pulse =  period / 2;
    			break;
    		default:	//always on
    			sConfigOC2.Pulse = 0;
    			break;
    	}
    	if (c1 && c2) sConfigOC2.Pulse = 0;	//same as always on

    	HAL_TIM_PWM_ConfigChannel(&htim3, (c2)? &sConfigOC2 : &sConfigOC1, TIM_CHANNEL_1);
    	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
    }else{
    	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
    }

	c1=(color1 & LED_BIT_GREEN) ? true:false;
	c2=(color2 & LED_BIT_GREEN) ? true:false;
    if (c1 || c2) {
    	switch (blink){
    		case LED_BLINK_FAST:
    			sConfigOC1.Pulse = sConfigOC2.Pulse = period / 4;
    			break;
    		case LED_BLINK_SLOW:
    			sConfigOC1.Pulse = sConfigOC2.Pulse =  period / 2;
    			break;
    		default:	//always on
    			sConfigOC2.Pulse = 0;
    			break;
    	}
    	if (c1 && c2) sConfigOC2.Pulse = 0;	//same as always on

    	HAL_TIM_PWM_ConfigChannel(&htim3, (c2)? &sConfigOC2 : &sConfigOC1, TIM_CHANNEL_2);
    	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
    }else{
    	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);
    }

	c1=(color1 & LED_BIT_BLUE) ? true:false;
	c2=(color2 & LED_BIT_BLUE) ? true:false;
    if (c1 || c2){
    	switch (blink){
    		case LED_BLINK_FAST:
    			sConfigOC1.Pulse = sConfigOC2.Pulse = period / 4;
    			break;
    		case LED_BLINK_SLOW:
    			sConfigOC1.Pulse = sConfigOC2.Pulse =  period / 2;
    			break;
    		default:	//always on
    			sConfigOC2.Pulse = 0;
    			break;
    	}
    	if (c1 && c2) sConfigOC2.Pulse = 0;	//same as always on

    	HAL_TIM_PWM_ConfigChannel(&htim3, (c2)? &sConfigOC2 : &sConfigOC1, TIM_CHANNEL_3);
    	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
    }else{
    	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_3);
    }
}
//Set LEDs Forcibley (no relation to PWM)
void LED_SetForced(uint8_t color){
	TIM_OC_InitTypeDef sConfigOC_Force;

	sConfigOC_Force.OCMode = (color & LED_BIT_RED)? TIM_OCMODE_FORCED_ACTIVE :TIM_OCMODE_FORCED_INACTIVE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC_Force, TIM_CHANNEL_1);
	sConfigOC_Force.OCMode = (color & LED_BIT_GREEN)? TIM_OCMODE_FORCED_ACTIVE : TIM_OCMODE_FORCED_INACTIVE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC_Force, TIM_CHANNEL_2);
    sConfigOC_Force.OCMode = (color & LED_BIT_BLUE)? TIM_OCMODE_FORCED_ACTIVE : TIM_OCMODE_FORCED_INACTIVE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC_Force, TIM_CHANNEL_3);

}

//Set LED flash
void LED_SetPulse(uint8_t color){
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.Pulse = period / 32;
	sConfigOC.OCMode = TIM_OCMODE_PWM2;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	htim3.Init.Period = period / 16;
	HAL_TIM_Base_Init(&htim3);

	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_3);

	if(color == LED_COLOR_OFF){
    	htim3.Instance->CR1 &= ~(TIM_CR1_OPM);	// Release "one pulse mode"
		return;
	}else{
		htim3.Instance->CR1 |= TIM_CR1_OPM;	// Enter "one pulse mode"
    	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
    	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
    	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
	}

    if (color & LED_BIT_RED) {
    	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
    }
    if (color & LED_BIT_GREEN) {
    	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
    }
    if (color & LED_BIT_BLUE){
    	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
    }
}

void LED_SetBackLight(bool light,uint16_t cycle){
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM2;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
<<<<<<< HEAD

	htim3.Init.Period = period;
	HAL_TIM_Base_Init(&htim3);

	if (light){
		if (cycle == LED_BL_STATIC){
			sConfigOC.Pulse = 0; //always on
		}else{
			htim3.Init.Period = cycle;
=======
    if (light){
    	if (cycle == LED_BL_STATIC){
    		sConfigOC.Pulse = 0; //always on
    	}else{
    		htim3.Init.Period = cycle;
>>>>>>> refs/heads/board_R1.2
			sConfigOC.Pulse = cycle / 2;
		}
		HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);
		HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	}else{
		HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_4);
	}
}
/* *****************************************************************************
 End of File
 */
