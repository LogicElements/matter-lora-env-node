/*****************************************************************************
* | File      	:   DEV_Config.c
* | Author      :   Waveshare team
* | Function    :   Hardware underlying interface
* | Info        :
*                Used to shield the underlying layers of each master
*                and enhance portability
*----------------
* |	This version:   V2.0
* | Date        :   2018-10-30
* | Info        :
# ******************************************************************************
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
******************************************************************************/
#include "DEV_Config.h"
#include "stm32u0xx_hal_spi.h"

extern SPI_HandleTypeDef hspi1;
void DEV_SPI_WriteByte(UBYTE value)
{
    HAL_SPI_Transmit(&hspi1, &value, 1, 1000);
}

void DEV_SPI_Write_nByte(UBYTE *value, UDOUBLE len)
{
    HAL_SPI_Transmit(&hspi1, value, len, 1000);
}

void DEV_GPIO_Mode(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, UWORD Mode)
{
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(Mode == 0) {
        GPIO_InitStruct.Pin = GPIO_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	} else {
		GPIO_InitStruct.Pin = GPIO_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		// Debug (" %d OUT \r\n",Pin);
	}
}

void DEV_GPIO_Init()
{
    HAL_SPI_MspDeInit(&hspi1);
    
    //HAL_SPI_DeInit(&hspi1); 
//    __HAL_RCC_SPI1_CLK_DISABLE();
//    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_7);
}

void DEV_SPI_Init()
{
    HAL_SPI_MspInit(&hspi1);
    //HAL_SPI_DeInit(&hspi1); 
//    __HAL_RCC_SPI1_CLK_DISABLE();
//    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_7);
}


void DEV_SPI_SendData(UBYTE Reg)
{
	UBYTE i,j=Reg;
	DEV_GPIO_Mode(EPD_MOSI_PIN, 1);
    DEV_GPIO_Mode(EPD_SCLK_PIN, 1);
	DEV_Digital_Write(EPD_CS_PIN, 0);
	for(i = 0; i<8; i++)
    {
        DEV_Digital_Write(EPD_SCLK_PIN, 0);     
        if (j & 0x80)
        {
            DEV_Digital_Write(EPD_MOSI_PIN, 1);
        }
        else
        {
            DEV_Digital_Write(EPD_MOSI_PIN, 0);
        }
        
        DEV_Digital_Write(EPD_SCLK_PIN, 1);
        j = j << 1;
    }
	DEV_Digital_Write(EPD_SCLK_PIN, 0);
	DEV_Digital_Write(EPD_CS_PIN, 1);
}

UBYTE DEV_SPI_ReadData()
{
	UBYTE i,j=0xff;
	DEV_GPIO_Mode(EPD_MOSI_PIN, 0);
    DEV_GPIO_Mode(EPD_SCLK_PIN, 1);
	DEV_Digital_Write(EPD_CS_PIN, 0);
	for(i = 0; i<8; i++)
	{
		DEV_Digital_Write(EPD_SCLK_PIN, 0);
		j = j << 1;
		if (DEV_Digital_Read(EPD_MOSI_PIN))
		{
            j = j | 0x01;
		}
		else
		{
            j= j & 0xfe;
		}
		DEV_Digital_Write(EPD_SCLK_PIN, 1);
	}
	DEV_Digital_Write(EPD_SCLK_PIN, 0);
	DEV_Digital_Write(EPD_CS_PIN, 1);
	return j;
}

int DEV_Module_Init(void)
{
  // Set initial logic levels
    DEV_Digital_Write(EPD_DC_PIN, 0);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    DEV_Digital_Write(EPD_PWR_PIN, 1);
    DEV_Digital_Write(EPD_RST_PIN, 1);
    return 0;
}

void DEV_Module_Exit(void)
{
    // Put control pins low
    DEV_Digital_Write(EPD_DC_PIN, 0);
    DEV_Digital_Write(EPD_CS_PIN, 0);

    // Turn off power to the display
    DEV_Digital_Write(EPD_PWR_PIN, 0);
    DEV_Digital_Write(EPD_RST_PIN, 0);

    // Put all EPD interface pins into High-Z state
    EPD_Pins_HiZ();
}

void EPD_Pins_HiZ(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;   // Analog mode = true Hi-Z, lowest leakage
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    // SCK
    GPIO_InitStruct.Pin = EPD_SCK_Pin;
    HAL_GPIO_Init(EPD_SCK_GPIO_Port, &GPIO_InitStruct);

    // CS
    GPIO_InitStruct.Pin = EPD_CS_Pin;
    HAL_GPIO_Init(EPD_CS_GPIO_Port, &GPIO_InitStruct);

    // DIN (MOSI)
    GPIO_InitStruct.Pin = EPD_DIN_Pin;
    HAL_GPIO_Init(EPD_DIN_GPIO_Port, &GPIO_InitStruct);

    // DC
    GPIO_InitStruct.Pin = EPD_DC_Pin;
    HAL_GPIO_Init(EPD_DC_GPIO_Port, &GPIO_InitStruct);

    // RST
    GPIO_InitStruct.Pin = EPD_RST_Pin;
    HAL_GPIO_Init(EPD_RST_GPIO_Port, &GPIO_InitStruct);

    // BUSY (usually an input from the display)
    GPIO_InitStruct.Pin = EPD_BUSY_Pin;
    HAL_GPIO_Init(EPD_BUSY_GPIO_Port, &GPIO_InitStruct);
}

void EPD_Pins_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // CS, DC, RST jako výstupy
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin = EPD_CS_Pin;  HAL_GPIO_Init(EPD_CS_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = EPD_DC_Pin;  HAL_GPIO_Init(EPD_DC_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = EPD_RST_Pin; HAL_GPIO_Init(EPD_RST_GPIO_Port, &GPIO_InitStruct);

    // BUSY jako vstup
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pin  = EPD_BUSY_Pin;
    HAL_GPIO_Init(EPD_BUSY_GPIO_Port, &GPIO_InitStruct);


}
