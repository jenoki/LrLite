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
#include <stdint.h>
/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */
#include "main.h"
extern I2C_HandleTypeDef hi2c1;
/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */

/*  A brief description of a section can be given directly below the section
    banner.
 */


void LCD_Initialize(){
    uint8_t buf[2];
    
    buf[0] = LCD_I2C_TAIL | LCD_I2C_INST | LCD_I2C_WRITE;
    buf[1] = LCD_CMD_FUNC | LCD_IF_8BIT | LCD_2LINE | LCD_NORMAL_INST;
    HAL_I2C_Master_Transmit(&hi2c1,LCD_I2C_ADDR,buf,sizeof(buf),500);
    Delay_us(LCD_CMD_WAIT_US);

    //Go to Extended Instruction sets.
    buf[1] = LCD_CMD_FUNC | LCD_IF_8BIT | LCD_2LINE | LCD_EXT_INST;
    HAL_I2C_Master_Transmit(&hi2c1,LCD_I2C_ADDR,buf,sizeof(buf),500);
    Delay_us(LCD_CMD_WAIT_US);

    buf[1] = LCD_CMD_FREQ | LCD_CLK_347K;
    HAL_I2C_Master_Transmit(&hi2c1,LCD_I2C_ADDR,buf,sizeof(buf),500);
    Delay_us(LCD_CMD_WAIT_US);

    buf[1] = LCD_CMD_CONTRAST | LCD_CONTRAST;
    HAL_I2C_Master_Transmit(&hi2c1,LCD_I2C_ADDR,buf,sizeof(buf),500);
    Delay_us(LCD_CMD_WAIT_US);

    buf[1] = LCD_CMD_ICON | LCD_BOOST | LCD_CONT_H;
    HAL_I2C_Master_Transmit(&hi2c1,LCD_I2C_ADDR,buf,sizeof(buf),500);
    Delay_us(LCD_CMD_WAIT_US);

    buf[1] = LCD_CMD_FOLLOWER;
    HAL_I2C_Master_Transmit(&hi2c1,LCD_I2C_ADDR,buf,sizeof(buf),500);
    HAL_Delay(LCD_FLW_WAIT_MS);
    
    //Revert to normal instruction sets
    buf[1] = LCD_CMD_FUNC | LCD_IF_8BIT | LCD_2LINE | LCD_NORMAL_INST;
    HAL_I2C_Master_Transmit(&hi2c1,LCD_I2C_ADDR,buf,sizeof(buf),500);
    Delay_us(LCD_CMD_WAIT_US);

    buf[1] = LCD_CMD_ONOFF | LCD_DISPLAY_ON;
    HAL_I2C_Master_Transmit(&hi2c1,LCD_I2C_ADDR,buf,sizeof(buf),500);
    Delay_us(LCD_CMD_WAIT_US);

    buf[1] = LCD_CMD_CLEAR;
    HAL_I2C_Master_Transmit(&hi2c1,LCD_I2C_ADDR,buf,sizeof(buf),500);
    Delay_us(LCD_CLR_WAIT_US);

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

void LCD_Clear(){
    uint8_t buf[2];
    buf[0] = LCD_I2C_TAIL | LCD_I2C_INST | LCD_I2C_WRITE;
    buf[1] = LCD_CMD_CLEAR;
    HAL_I2C_Master_Transmit(&hi2c1,LCD_I2C_ADDR,buf,sizeof(buf),500);
    Delay_us(LCD_CMD_WAIT_US);
}

//line 0:upper 1:lower
void LCD_Locate(uint8_t column,uint8_t line){
    uint8_t buf[2];
    uint8_t ddadr = (line * LCD_DD_PER_LINE) + column;
    buf[0] = LCD_I2C_TAIL | LCD_I2C_INST | LCD_I2C_WRITE;
    buf[1] = LCD_CMD_DDADR | ddadr;
    HAL_I2C_Master_Transmit(&hi2c1,LCD_I2C_ADDR,buf,sizeof(buf),500);
    Delay_us(LCD_CMD_WAIT_US);
}

void LCD_SetDisplay(bool on, bool cursor, bool blink){
    uint8_t buf[2];
    buf[0] = LCD_I2C_TAIL | LCD_I2C_INST | LCD_I2C_WRITE;
    buf[1] = LCD_CMD_ONOFF | (on)? LCD_DISPLAY_ON:0 | (cursor)? LCD_CURSOR_ON:0 | (blink)? LCD_BLINK_ON:0 ;
    HAL_I2C_Master_Transmit(&hi2c1,LCD_I2C_ADDR,buf,sizeof(buf),500);
    Delay_us(LCD_CMD_WAIT_US);
}

void LCD_Putchar(char c){
    uint8_t buf[2];
    buf[0] = LCD_I2C_TAIL | LCD_I2C_DATA | LCD_I2C_WRITE;
    buf[1] = c;
    HAL_I2C_Master_Transmit(&hi2c1,LCD_I2C_ADDR,buf,sizeof(buf),500);
    Delay_us(LCD_DAT_WAIT_US);
}

void LCD_Print(char *str){
    size_t len = strlen(str);

    if(len > LCD_WIDTH) len = LCD_WIDTH;
#if 0
    uint8_t i;
    for (i = 0; i < len; i++){
        LCD_Putchar(*(str+i));
    }
#else
    uint8_t i2cbuf[LCD_WIDTH*2],*p = i2cbuf;
    uint8_t i;
    uint8_t cmd= LCD_I2C_CONTINUE | LCD_I2C_DATA | LCD_I2C_WRITE;    // =0xC0;
    for(i = 0;i < (len-1) ; i++){
        *p++ = cmd;
        *p++ = str[i];
    }
    *p++ = LCD_I2C_TAIL | LCD_I2C_DATA | LCD_I2C_WRITE;    // =0x40;
    *p++ = str[len-1];
    
    HAL_I2C_Master_Transmit(&hi2c1,LCD_I2C_ADDR,i2cbuf,(len*2),500);
    Delay_us(LCD_DAT_WAIT_US);
#endif
}
#if 0 //moved to led.c
void LCD_SetBL(bool light) {
    HAL_GPIO_WritePin(BL_ON_GPIO_Port,BL_ON_Pin,(light)?GPIO_PIN_SET:GPIO_PIN_RESET);
}
#endif
/* *****************************************************************************
 End of File
 */
