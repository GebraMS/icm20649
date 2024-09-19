/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2020 GebraBit Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to GebraBit and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws. 
 *
 * GebraBit and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from GebraBit is strictly prohibited.
 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT 
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT IN  
 * NO EVENT SHALL GebraBit BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, 
 * OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * @Author       	: Mehrdad Zeinali
 * ________________________________________________________________________________________________________
 */
#include	"GebraBit_ICM20649.h"

extern SPI_HandleTypeDef hspi1;
	
/*=========================================================================================================================================
 * @brief     Read data from spacial register.
 * @param     regAddr Register Address of ICM20649
 * @param     regBank Register Bank number .
 * @param     data    Pointer to Variable that register value is saved .
 * @return    stat    Return status
 ========================================================================================================================================*/
uint8_t	GB_ICM20649_Read_Reg_Data ( uint8_t regAddr, ICM20649_Bank_Sel regBank, uint8_t* data)
{	
	uint8_t txBuf[2] = {regAddr|0x80 , 0x00}; //Read operation: set the 8th-bit to 1.
	uint8_t rxBuf[2];
	HAL_StatusTypeDef stat = HAL_ERROR ;
	GB_ICM20649_Bank_Selection(regBank);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	stat = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 2, HAL_MAX_DELAY));
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	if (stat == HAL_OK)
	{
		*data = rxBuf[1];
	}
	return stat;
}
/*========================================================================================================================================= 
 * @brief     Read data from spacial bits of a register.
 * @param     regAddr     Register Address of ICM20649 .
 * @param     regBank     Register Bank number .
 * @param     start_bit   Start Bit location .(0 to 7)
 * @param     len         Quantity of Bits want to read(1 to 8) 
 * @param     data        Pointer to Variable that register Bits value is saved .
 * @return    status      Return status
 ========================================================================================================================================*/
uint8_t GB_ICM20649_Read_Reg_Bits (uint8_t regAddr, ICM20649_Bank_Sel regBank, uint8_t start_bit, uint8_t len, uint8_t* data)
{
	uint8_t status = HAL_ERROR;
	uint8_t tempData = 0;

	if (len>8 || start_bit>7)
	{
		return HAL_ERROR;
	}

	if (GB_ICM20649_Read_Reg_Data( regAddr, regBank, &tempData) == HAL_OK)
	{
		uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1); //formula for making a broom of 1&0 for gathering desired bits
		tempData &= mask; // zero all non-important bits in data
		tempData >>= (start_bit - len + 1); //shift data to zero position
		*data = tempData;
		status = HAL_OK;
	}
	else
	{
		status = HAL_ERROR;
		*data = 0;
	}
	return status;
}
/*========================================================================================================================================= 
 * @brief     Read multiple data from first spacial register address.
 * @param     regAddr First Register Address of ICM20649 that reading multiple data start from this address
 * @param     regBank Register Bank number .
 * @param     data    Pointer to Variable that multiple data is saved .
 * @param     byteQuantity Quantity of data that we want to read .
 * @return    stat    Return status
 ========================================================================================================================================*/
uint8_t GB_ICM20649_Burst_Read(uint8_t regAddr, ICM20649_Bank_Sel regBank, uint8_t *data, uint16_t byteQuantity)
{
	uint8_t *pTxBuf;
	uint8_t *pRxBuf;
	uint8_t status = HAL_ERROR;
	GB_ICM20649_Bank_Selection(regBank);
	pTxBuf = ( uint8_t * )malloc(sizeof(uint8_t) * (byteQuantity + 1)); // reason of "+1" is for register address that comes in first byte
	pRxBuf = ( uint8_t * )malloc(sizeof(uint8_t) * (byteQuantity + 1));
	memset(pTxBuf, 0, (byteQuantity + 1)*sizeof(uint8_t));

	pTxBuf[0] = regAddr | 0x80; //Read operation: set the 8th-bit to 1.

	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	status = (HAL_SPI_TransmitReceive(&hspi1, pTxBuf, pRxBuf, byteQuantity+1, HAL_MAX_DELAY));
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	
	if (status == HAL_OK)
	{
		memcpy(data, &pRxBuf[1], byteQuantity*sizeof(uint8_t)); //here we dont have "+1" beacause we don't need first byte that was register data , we just need DATA itself
	}
	free(pTxBuf);
	free(pRxBuf);
	return status;
}
/*=========================================================================================================================================
 * @brief     Write data to spacial register.
 * @param     regAddr Register Address of ICM20649
 * @param     regBank Register Bank number .
 * @param     data    Value that will be writen to register .
 * @return    stat    Return status
 ========================================================================================================================================*/
uint8_t GB_ICM20649_Write_Reg_Data(uint8_t regAddr, ICM20649_Bank_Sel regBank, uint8_t data)
{
	uint8_t txBuf[2] = {regAddr|0x00 , data}; //Write operation: set the 8th-bit to 0.
	uint8_t rxBuf[2];
	uint8_t status = HAL_ERROR;
	GB_ICM20649_Bank_Selection(regBank);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	status = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 2, HAL_MAX_DELAY));
	while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	
	return status;	
}

/*=========================================================================================================================================
 * @brief     Write data to spacial bits of a register.
 * @param     regAddr     Register Address of ICM20649 .
 * @param     regBank     Register Bank number .
 * @param     start_bit   Start Bit location .(0 to 7)
 * @param     len         Quantity of Bits want to write(1 to 8) 
 * @param     data        Value that will be writen to register bits .
 * @return    status      Return status
 ========================================================================================================================================*/
uint8_t GB_ICM20649_Write_Reg_Bits(uint8_t regAddr, ICM20649_Bank_Sel regBank, uint8_t start_bit, uint8_t len, uint8_t data)
{
	uint8_t txBuf[2];
	uint8_t rxBuf[2];
	uint8_t status = HAL_ERROR;
	uint8_t tempData = 0;
	if (len>8 || start_bit>7)
	{
		return HAL_ERROR;
	}
	if (GB_ICM20649_Read_Reg_Data( regAddr, regBank, &tempData) == HAL_OK)	
	{
		uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1);
		data <<= (start_bit - len + 1); // shift data into correct position
		data &= mask; // zero all non-important bits in data
		tempData &= ~(mask); // zero all important bits in existing byte
		tempData |= data; // combine data with existing byte

		txBuf[0] = regAddr;
		txBuf[1] = tempData;
	
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
		status = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 2, HAL_MAX_DELAY));
		while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	}
	return status;
}
/*========================================================================================================================================= 
 * @brief     Write value to Multiple register address.
 * @param     regAddr First Register Address of ICM20649 that writing multiple data start from this address
 * @param     regBank Register Bank number .
 * @param     data    Pointer to Variable that multiple data are writen from .
 * @param     byteQuantity Quantity of data that we want to write .
 * @return    stat    Return status
 ========================================================================================================================================*/
uint8_t GB_ICM20649_Burst_Write		( uint8_t regAddr, ICM20649_Bank_Sel regBank, uint8_t *data, 	uint16_t byteQuantity)
{
	uint8_t txBuf[byteQuantity + 1]; // +1 is for register address that is 1 byte
	uint8_t rxBuf[byteQuantity + 1];
	uint8_t status = HAL_ERROR;
	GB_ICM20649_Bank_Selection(regBank);
	txBuf[0] = regAddr | 0x00; //Write operation: set the 8th-bit to 0.
	memcpy(txBuf+1, data, byteQuantity); // +1 is for set the address of data from [1]th position of array

	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	status = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, byteQuantity+1, HAL_MAX_DELAY));
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

	return status;
}
/*=========================================================================================================================================
 * @brief     Select Register Bank.
 * @param     bsel   Bank number
 * @return    Nothing
 ========================================================================================================================================*/
void GB_ICM20649_Bank_Selection( ICM20649_Bank_Sel bsel)
{
  uint8_t rtxBuf[2] = {ICM20649_BANK_SEL|0x80 , 0x00}; //Read operation: set the 8th-bit to 1.
	uint8_t rrxBuf[2];
	uint8_t wtxBuf[2];
	uint8_t wrxBuf[2];
	HAL_StatusTypeDef stat = HAL_ERROR ;
	uint8_t tempData = 0;
	uint8_t start_bit = START_MSB_BIT_AT_5 ;
	uint8_t len = BIT_LENGTH_2 ;
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	stat = (HAL_SPI_TransmitReceive(&hspi1, rtxBuf, rrxBuf, 2, HAL_MAX_DELAY));
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	if (stat == HAL_OK)
	{
		tempData = rrxBuf[1];
	}
		uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1);
		bsel <<= (start_bit - len + 1); // shift data into correct position
		bsel &= mask; // zero all non-important bits in data
		tempData &= ~(mask); // zero all important bits in existing byte
		tempData |= bsel; // combine data with existing byte

		wtxBuf[0] = ICM20649_BANK_SEL;
		wtxBuf[1] = tempData;
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
		stat = (HAL_SPI_TransmitReceive(&hspi1, wtxBuf, wrxBuf, 2, HAL_MAX_DELAY));
		while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
}
/*=========================================================================================================================================
 * @brief     Reset ICM20649
 * @param     ICM20649   ICM20649 Struct RESET  variable
 * @param     ICM20649   ICM20649 Struct
 * @return    Nothing
 ========================================================================================================================================*/
void GB_ICM20649_Soft_Reset ( GebraBit_ICM20649 * ICM20649 )
{
	do 
	 {
		GB_ICM20649_Write_Reg_Bits(ICM20649_PWR_MGMT_1,BANK_0, START_MSB_BIT_AT_7, BIT_LENGTH_1 , 1);
		HAL_Delay(50);
		GB_ICM20649_Read_Reg_Bits (ICM20649_PWR_MGMT_1,BANK_0 ,START_MSB_BIT_AT_7, BIT_LENGTH_1, &ICM20649->RESET);
		if ( ICM20649->RESET == DONE )
			break;
	 }while(1);
}
/*=========================================================================================================================================
 * @brief     Get Who am I Register Value From Sensor
 * @param     ICM20649     ICM20649 Struct WHO_AM_I variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void	GB_ICM20649_Who_am_I(GebraBit_ICM20649 * ICM20649)
{
	GB_ICM20649_Read_Reg_Data( ICM20649_WHO_AM_I, BANK_0,&ICM20649->WHO_AM_I);
}	

/*=========================================================================================================================================
 * @brief     Select SPI 4 Wire as interface
 * @param     ICM20649   ICM20649 Struct INTERFACE  variable
 * @param     spisel Determines SPI 4 Wire as interface or not 
 * @return    Nothing
 ========================================================================================================================================*/
void GB_ICM20649_Select_SPI4_Interface(GebraBit_ICM20649 * ICM20649 , ICM20649_Interface spisel)
{
 GB_ICM20649_Write_Reg_Bits( ICM20649_USER_CTRL,BANK_0, START_MSB_BIT_AT_4, BIT_LENGTH_1 , spisel);
 ICM20649->INTERFACE = spisel ; 
}
/*=========================================================================================================================================
 * @brief     Enable Or Disable DMP and Set it to Low Power mode if needed.
 * @param     ICM20649   ICM20649 Struct DMP  variable
 * @param     dmp    Enable Or Disable DMP
 * @param     dmp_lp Determines DMP in 	DMP_LOW_POWER or NOT_DMP_LOW_POWER
 * @return    Nothing
 ========================================================================================================================================*/

void GB_ICM20649_DMP(GebraBit_ICM20649* ICM20649 ,ICM20649_Ability dmp,ICM20649_DMP_LP dmp_lp)
{
	if(dmp_lp==NOT_DMP_LOW_POWER)
	{
		GB_ICM20649_Write_Reg_Bits (ICM20649_USER_CTRL ,BANK_0, START_MSB_BIT_AT_7, BIT_LENGTH_1 , dmp);
	  GB_ICM20649_Write_Reg_Bits (ICM20649_MOD_CTRL_USR ,BANK_0, START_MSB_BIT_AT_0, BIT_LENGTH_1 , dmp_lp);
	}
	else if(dmp_lp==DMP_LOW_POWER)
	{
		GB_ICM20649_Write_Reg_Bits (ICM20649_USER_CTRL ,BANK_0, START_MSB_BIT_AT_7, BIT_LENGTH_1 , dmp);
	  GB_ICM20649_Write_Reg_Bits (ICM20649_MOD_CTRL_USR ,BANK_0, START_MSB_BIT_AT_0, BIT_LENGTH_1 , dmp_lp);
	}
	ICM20649->DMP = dmp ;
}

/*=========================================================================================================================================
 * @brief     Reset DMP
 * @param     rst     Determines reset DMP or not
 * @return    Nothing
 ========================================================================================================================================*/ 

void GB_ICM20649_DMP_Reset(GebraBit_ICM20649* ICM20649 ,ICM20649_Ability rst)
{
  GB_ICM20649_Write_Reg_Bits (ICM20649_USER_CTRL ,BANK_0, START_MSB_BIT_AT_3, BIT_LENGTH_1 , rst);
}

/*=========================================================================================================================================
 * @brief     Set DMP interrupt on which Pin set
 * @param     pin    Determines INTERRUPT_ON_PIN_1 or INTERRUPT_ON_PIN_2
 * @param     interrupt Enable Or Disable interrupt
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_DMP_Interrupt(ICM20649_Interrupt_Pin pin,ICM20649_Ability interrupt)///DMP_INTERRUPT---->STRUCT
{
  if(pin==INTERRUPT_ON_PIN_1)
		GB_ICM20649_Write_Reg_Bits (ICM20649_INT_ENABLE ,BANK_0, START_MSB_BIT_AT_1, BIT_LENGTH_1 , interrupt);
	else if(pin==INTERRUPT_ON_PIN_2)
		GB_ICM20649_Write_Reg_Bits (ICM20649_INT_ENABLE ,BANK_0, START_MSB_BIT_AT_4, BIT_LENGTH_1 , interrupt);
}
/*=========================================================================================================================================
 * @brief     SET ICM20649 Sleep or Awake
 * @param     ICM20649   ICM20649 Struct IS_ICM20649_Sleep  variable
 * @param     working   Determines ICM20649_AWAKE or ICM20649_SLEEP
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Sleep_Awake (GebraBit_ICM20649 * ICM20649, ICM20649_Sleep  working  ) 
{
  GB_ICM20649_Write_Reg_Bits (ICM20649_PWR_MGMT_1 ,BANK_0, START_MSB_BIT_AT_6, BIT_LENGTH_1 , working);
  ICM20649->IS_ICM20649_SLEEP = working ;
}

/*=========================================================================================================================================
 * @brief     Set ICM20649 Accelerometer Power Mode
 * @param     ICM20649   ICM20649 Struct ACCEL_POWER_MODE  variable
 * @param     pmode        Determines ICM20649 Accelerometer Power Mode in ICM20649_LOW_NOISE or ICM20649_LOW_POWER
 * @return    Nothing
 ========================================================================================================================================*/ 
 void GB_ICM20649_ACCEL_Power_Mode(GebraBit_ICM20649* ICM20649 ,ICM20649_Power_Mode pmode)
{
	GB_ICM20649_Write_Reg_Bits (ICM20649_LP_CONFIG ,BANK_0, START_MSB_BIT_AT_5, BIT_LENGTH_1 , pmode);
	ICM20649->ACCEL_POWER_MODE = pmode ;
}
/*=========================================================================================================================================
 * @brief     Set ICM20649 Gyroscope Power Mode
 * @param     ICM20649   ICM20649 Struct GYRO_POWER_MODE  variable
 * @param     pmode        Determines ICM20649 Gyroscope Power Mode in ICM20649_LOW_NOISE or ICM20649_LOW_POWER
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_GYRO_Power_Mode(GebraBit_ICM20649* ICM20649 ,ICM20649_Power_Mode pmode)
{
	GB_ICM20649_Write_Reg_Bits (ICM20649_LP_CONFIG ,BANK_0, START_MSB_BIT_AT_4, BIT_LENGTH_1 , pmode);
	ICM20649->GYRO_POWER_MODE = pmode ;
}
/*=========================================================================================================================================
 * @brief     Set ICM20649 Clock Source
 * @param     ICM20649   ICM20649 Struct CLOCK_SOURCE  variable
 * @param     clk    Determines between INTERNAL_20MHZ_OSCILLATOR , AUTO_SELECT and CLOCK_STOP
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Set_Clock_Source(GebraBit_ICM20649 * ICM20649 , ICM20649_CLK clk)
{ 
 GB_ICM20649_Write_Reg_Bits( ICM20649_PWR_MGMT_1,BANK_0, START_MSB_BIT_AT_2, BIT_LENGTH_3 , clk);
 ICM20649->CLOCK_SOURCE = clk ;
} 
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Temperature Sensor
 * @param     ICM20649   ICM20649 Struct TEMPERATURE  variable
 * @param     temp     Determines DISABLE or ENABLE Temperature Sensor
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Temperature(GebraBit_ICM20649* ICM20649 ,ICM20649_Ability temp)
{
	GB_ICM20649_Write_Reg_Bits (ICM20649_PWR_MGMT_1 ,BANK_0, START_MSB_BIT_AT_3, BIT_LENGTH_1 , !temp);
  ICM20649->TEMPERATURE = temp ;
}
/*
M403Z 
*/
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Accelerometer Sensor
 * @param     ICM20649   ICM20649 Struct ACCEL  variable
 * @param     accel     Determines DISABLE or ENABLE Accelerometer Sensor
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_ICM20649_Accelerometer(GebraBit_ICM20649 * ICM20649 , ICM20649_Sensor accel)
{
	GB_ICM20649_Write_Reg_Bits (ICM20649_PWR_MGMT_2 ,BANK_0, START_MSB_BIT_AT_5, BIT_LENGTH_3 , accel);
  ICM20649->ACCEL = accel ; 
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Gyroscope Sensor
 * @param     ICM20649   ICM20649 Struct GYRO  variable
 * @param     gyro     Determines DISABLE or ENABLE Gyroscope Sensor
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Gyroscope(GebraBit_ICM20649 * ICM20649 , ICM20649_Sensor gyro)
{
	GB_ICM20649_Write_Reg_Bits (ICM20649_PWR_MGMT_2 ,BANK_0, START_MSB_BIT_AT_2, BIT_LENGTH_3 , gyro);
  ICM20649->GYRO = gyro ; 
}
/*=========================================================================================================================================
 * @brief     Configure hardware interrupt pin (INT1) 
 * @param     ICM20649  ICM20649 struct INT1_PIN_LEVEL , INT1_PIN_TYPE and INT1_PIN_LATCH  variables
 * @param     level   ACTIVE_HIGH or  ACTIVE_LOW 
 * @param     type    PUSH_PULL   or  OPEN_DRAIN
 * @param     latch   _50_US      or  HELD_STATUS_CLEAR
 * @return    Nothing
 ========================================================================================================================================*/
void GB_ICM20649_Set_INT1_Pin(GebraBit_ICM20649 * ICM20649 , ICM20649_INT_Level level ,ICM20649_INT_Type type , ICM20649_Latch_Type latch )
{
  GB_ICM20649_Write_Reg_Bits( ICM20649_INT_PIN_CFG,BANK_0, START_MSB_BIT_AT_7, BIT_LENGTH_1 , level);
	GB_ICM20649_Write_Reg_Bits( ICM20649_INT_PIN_CFG,BANK_0, START_MSB_BIT_AT_6, BIT_LENGTH_1 , type);
	GB_ICM20649_Write_Reg_Bits( ICM20649_INT_PIN_CFG,BANK_0, START_MSB_BIT_AT_5, BIT_LENGTH_1 , latch);
	ICM20649->INT1_PIN_LEVEL = level ; 
	ICM20649->INT1_PIN_TYPE  = type  ;
	ICM20649->INT1_PIN_LATCH = latch ;
}
/*=========================================================================================================================================
 * @brief     Configure hardware interrupt pin (INT2) 
 * @param     ICM20649  ICM20649 struct INT2_PIN_LEVEL , INT2_PIN_TYPE and INT2_PIN_LATCH  variables
 * @param     level   ACTIVE_HIGH or  ACTIVE_LOW 
 * @param     type    PUSH_PULL   or  OPEN_DRAIN
 * @param     latch   _50_US      or  HELD_STATUS_CLEAR
 * @return    Nothing
 ========================================================================================================================================*/
void GB_ICM20649_Set_INT2_Pin(GebraBit_ICM20649 * ICM20649 , ICM20649_INT_Level level ,ICM20649_INT_Type type , ICM20649_Latch_Type latch )
{
  GB_ICM20649_Write_Reg_Bits( ICM20649_INT_ENABLE_1,BANK_0, START_MSB_BIT_AT_7, BIT_LENGTH_1 , level);
	GB_ICM20649_Write_Reg_Bits( ICM20649_INT_ENABLE_1,BANK_0, START_MSB_BIT_AT_6, BIT_LENGTH_1 , type);
	GB_ICM20649_Write_Reg_Bits( ICM20649_INT_ENABLE_1,BANK_0, START_MSB_BIT_AT_5, BIT_LENGTH_1 , latch);
	ICM20649->INT2_PIN_LEVEL = level ; 
	ICM20649->INT2_PIN_TYPE  = type  ;
	ICM20649->INT2_PIN_LATCH = latch ;
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Interrupt Status
 * @param     ICM20649  ICM20649 struct INTERRUPT  variable 
 * @param     interrupt    DISABLE or ENABLE
 * @return    Nothing
 ========================================================================================================================================*/ 
void Interrupt_Status_Enable(GebraBit_ICM20649 * ICM20649 , ICM20649_Ability interrupt )
{
  GB_ICM20649_Write_Reg_Bits (ICM20649_FIFO_CFG ,BANK_0, START_MSB_BIT_AT_0, BIT_LENGTH_1 , interrupt);
	ICM20649->INTERRUPT = interrupt ;
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Access Serial Interface To FIFO
 * @param     ICM20649  ICM20649 struct INTERFACE_ACCESS_FIFO  variable
 * @param     interface_access_fifo    DISABLE or ENABLE
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Access_Serial_Interface_To_FIFO(GebraBit_ICM20649 * ICM20649 , ICM20649_Ability interface_access_fifo) 
{ 
	GB_ICM20649_Write_Reg_Bits (ICM20649_USER_CTRL , BANK_0, START_MSB_BIT_AT_6, BIT_LENGTH_1,  interface_access_fifo);
  ICM20649->INTERFACE_ACCESS_FIFO = interface_access_fifo ;  
}
/*=========================================================================================================================================
 * @brief     Check if FIFO Overflow
 * @param     ICM20649    Store FIFO Overflow status on ICM20649 Struct FIFO_OVERFLOW variable
 * @return    NOT_FIFO_OVERFLOW or FIFO_OVERFLOW
 ========================================================================================================================================*/ 
ICM20649_FIFO_Overflow GB_ICM20649_Check_FIFO_Overflow(GebraBit_ICM20649 * ICM20649)
{
  GB_ICM20649_Read_Reg_Bits (ICM20649_INT_STATUS_2,BANK_0, START_MSB_BIT_AT_4, BIT_LENGTH_5, &ICM20649->FIFO_OVERFLOW); 
	return ICM20649->FIFO_OVERFLOW;
}
/*=========================================================================================================================================
 * @brief     Check if Data is ready
 * @param     ICM20649    Store data ready status on ICM20649 Struct DATA_STATUS variable
 * @return    IS_Ready or IS_NOT_Ready
 ========================================================================================================================================*/ 
ICM20649_Preparation GB_ICM20649_Check_Data_Preparation(GebraBit_ICM20649 * ICM20649)
{
  GB_ICM20649_Read_Reg_Bits (ICM20649_INT_STATUS_1,BANK_0, START_MSB_BIT_AT_0, BIT_LENGTH_1, &ICM20649->DATA_STATUS); 
	return ICM20649->DATA_STATUS;
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE accelerometer to be written on FIFO
 * @param     ICM20649  ICM20649 struct ACCEL_TO_FIFO  variable  
 * @param     accel_fifo Determines accelerometer write on fifo
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Write_ACCEL_FIFO(GebraBit_ICM20649 * ICM20649 , ICM20649_Ability accel_fifo )
{
   GB_ICM20649_Write_Reg_Bits (ICM20649_FIFO_EN_2,BANK_0, START_MSB_BIT_AT_4, BIT_LENGTH_1,accel_fifo); 
	 ICM20649->ACCEL_TO_FIFO = accel_fifo ;
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Gyroscope to be written on FIFO
 * @param     ICM20649  ICM20649 struct GYRO_TO_FIFO  variable  
 * @param     gyro_fifo  Determines Gyroscope write on fifo
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Write_GYRO_FIFO(GebraBit_ICM20649 * ICM20649 , ICM20649_Ability gyro_fifo )
{
   GB_ICM20649_Write_Reg_Bits (ICM20649_FIFO_EN_2,BANK_0, START_MSB_BIT_AT_3, BIT_LENGTH_3,(uint8_t)(gyro_fifo*7)); 
	 ICM20649->GYRO_TO_FIFO = gyro_fifo ;
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Temperature to be written on FIFO
 * @param     ICM20649  ICM20649 struct TEMP_TO_FIFO  variable 
 * @param     temp_fifo  Determines Temperature write on fifo
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_ICM20649_Write_TEMP_FIFO(GebraBit_ICM20649 * ICM20649 , ICM20649_Ability temp_fifo )
{
   GB_ICM20649_Write_Reg_Bits (ICM20649_FIFO_EN_2,BANK_0, START_MSB_BIT_AT_0, BIT_LENGTH_1,temp_fifo); 
	 ICM20649->TEMP_TO_FIFO = temp_fifo ;
}
/*=========================================================================================================================================
 * @brief     Set FIFO MODE
 * @param     ICM20649  ICM20649 struct FIFO_MODE  variable 
 * @param     mode     Determines FIFO MODE BYPASS ,  STREAM_TO_FIFO , STOP_ON_FULL_FIFO_SNAPSHOT
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_FIFO_Mode(GebraBit_ICM20649 * ICM20649 , ICM20649_FIFO_Mode fifo_mode )
{
  GB_ICM20649_Write_Reg_Bits (ICM20649_FIFO_MODE,BANK_0, START_MSB_BIT_AT_4, BIT_LENGTH_5, fifo_mode); 
  ICM20649->FIFO_MODE = fifo_mode;
}
/*=========================================================================================================================================
 * @brief     Set FIFO reset. Assert and hold to set FIFO size to 0. Assert and de-assert to reset FIFO.
 * @param     ICM20649  ICM20649 struct FIFO_RESET  variable 
 * @param     fifo_rst     Determines NOT_FIFO_REST or FIFO_REST
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_FIFO_Reset(void ) 
{
  GB_ICM20649_Write_Reg_Bits (ICM20649_FIFO_RST,BANK_0, START_MSB_BIT_AT_4, BIT_LENGTH_5, FIFO_ASSERT); 
	HAL_Delay(5);
	GB_ICM20649_Write_Reg_Bits (ICM20649_FIFO_RST,BANK_0, START_MSB_BIT_AT_4, BIT_LENGTH_5, FIFO_DE_ASSERT);
  HAL_Delay(1);	
  //ICM20649->FIFO_RESET = fifo_rst;
}
/*=========================================================================================================================================
 * @brief     Get FIFO Count  
 * @param     ICM20649   ICM20649 struct  FIFO_COUNT variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_GET_FIFO_Count (GebraBit_ICM20649 * ICM20649 ) 
{
	uint8_t count_h , count_l;
  GB_ICM20649_Read_Reg_Bits( ICM20649_FIFO_COUNTH, BANK_0,START_MSB_BIT_AT_4, BIT_LENGTH_5, &count_h); 
	GB_ICM20649_Read_Reg_Data( ICM20649_FIFO_COUNTL, BANK_0, &count_l );
	ICM20649->FIFO_COUNT = (uint16_t)((count_h << 8) | count_l);////13_Bit
}
/*=========================================================================================================================================
 * @brief     Read Data Directly from FIFO
 * @param     ICM20649  ICM20649 struct FIFO_DATA variable
 * @param     qty    Determine hoe many Data Byte to read
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Read_FIFO(GebraBit_ICM20649 * ICM20649 , uint16_t qty)  
{
  GB_ICM20649_Burst_Read( ICM20649_FIFO_R_W,BANK_0,ICM20649->FIFO_DATA, qty);
}
/*=========================================================================================================================================
 * @brief     Check if Data is Copied TO FIFO
 * @param     ICM20649    Store FIFO Copy status on ICM20649 Struct DATA_COPY_FIFO variable
 * @return    COPY_TO_FIFO or NOT_COPY_FIFO
 ========================================================================================================================================*/ 
ICM20649_Data_Copy_FIFO GB_ICM20649_Check_Data_Copy_TO_FIFO(GebraBit_ICM20649 * ICM20649)
{
  GB_ICM20649_Read_Reg_Bits (ICM20649_DATA_RDY_STATUS,BANK_0, START_MSB_BIT_AT_3, BIT_LENGTH_4, &ICM20649->DATA_COPY_FIFO); 
	return ICM20649->DATA_COPY_FIFO;
}

/*=========================================================================================================================================
 * @brief     Set Gyroscope Full Scale Range and select Gyroscope SCALE FACTOR
 * @param     ICM20649   ICM20649 Struct GYRO_FULL_SCALE and GYRO_SCALE_FACTOR variable
 * @param     fs         Determines Full Scale Range among FS_500_DPS , FS_1000_DPS , FS_2000_DPS , FS_4000_DPS
 * @return    Nothing
 ========================================================================================================================================*/
void GB_ICM20649_GYRO_Full_Scale ( GebraBit_ICM20649 * ICM20649 , ICM20649_Gyro_Fs_Sel fs ) 
{
  GB_ICM20649_Write_Reg_Bits (ICM20649_GYRO_CONFIG_1,BANK_2 , START_MSB_BIT_AT_2, BIT_LENGTH_2, fs);
	ICM20649->GYRO_FULL_SCALE = fs ; 
	switch(fs)
	 {
	  case FS_500_DPS:
		ICM20649->GYRO_SCALE_FACTOR = SCALE_FACTOR_65p5_LSB_DPS ;
		ICM20649->PRECISE_GYRO_SF   =  65.5 ;
    break;
		case FS_1000_DPS:
		ICM20649->GYRO_SCALE_FACTOR = SCALE_FACTOR_32p8_LSB_DPS ;
		ICM20649->PRECISE_GYRO_SF   =  32.8 ;
    break;	
		case FS_2000_DPS:
		ICM20649->GYRO_SCALE_FACTOR = SCALE_FACTOR_16p4_LSB_DPS ;
		ICM20649->PRECISE_GYRO_SF   =  16.4 ;
    break;	
		case FS_4000_DPS:
		ICM20649->GYRO_SCALE_FACTOR = SCALE_FACTOR_8p2_LSB_DPS ;
		ICM20649->PRECISE_GYRO_SF   =  8.2 ;
    break;			
		default:
		ICM20649->GYRO_SCALE_FACTOR = SCALE_FACTOR_65p5_LSB_DPS ;
    ICM20649->PRECISE_GYRO_SF   =  65.5 ;		
	 }
}
/*=========================================================================================================================================
 * @brief     Enable Or Bypass GYRO Low Pass Filter
 * @param     ICM20649     ICM20649 Struct GYRO_FCHOICEB variable
 * @param     bypass       Determines ENABLE_DLPF_FCHOICEB_1 or BYPASS_DLPF_FCHOICEB_0
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_ICM20649_GYRO_Low_Pass_Filter  (GebraBit_ICM20649 * ICM20649 ,  ICM20649_FCHOICEB bypass )
{
	GB_ICM20649_Write_Reg_Bits(ICM20649_GYRO_CONFIG_1,BANK_2 , START_MSB_BIT_AT_0, BIT_LENGTH_1,  bypass);
	ICM20649->GYRO_FCHOICEB =bypass ;   
}
/*=========================================================================================================================================
 * @brief     Set GYRO Low Pass Filter value
 * @param     ICM20649     ICM20649 Struct GYRO_DLPF variable
 * @param     dlpf     Low Pass Filter value 
 * @return    Nothing
 ========================================================================================================================================*/ 		
void GB_ICM20649_GYRO_Low_Pass_Filter_Value  (GebraBit_ICM20649 * ICM20649 , ICM20649_GYRO_DLPF dlpf )
{
	  GB_ICM20649_Write_Reg_Bits(ICM20649_GYRO_CONFIG_1,BANK_2 , START_MSB_BIT_AT_5, BIT_LENGTH_3,  dlpf);
	  ICM20649->GYRO_DLPF =  dlpf ;
}
/*=========================================================================================================================================
 * @brief     Set  GYRO Averaging Filter
 * @param     ICM20649  ICM20649 Struct GYRO_AVERAGING_FILTER variable
 * @param     avg       Averaging value
 * @return    Nothing 
 ========================================================================================================================================*/ 		
void GB_ICM20649_GYRO_LP_Averaging_Filter  (GebraBit_ICM20649 * ICM20649 , ICM20649_GYRO_Averaging_Filter avg )
{
	  GB_ICM20649_Write_Reg_Bits(ICM20649_GYRO_CONFIG_2,BANK_2 , START_MSB_BIT_AT_2, BIT_LENGTH_3,  avg);
	  ICM20649->GYRO_AVERAGING_FILTER =  avg ;
}
/*=========================================================================================================================================
 * @brief     Set Gyroscope Sensor Output Sample Rate that controls sensor data output rate, FIFO sample rate
 * @param     ICM20649   ICM20649 struct GYRO_SAMPLE_RATE and GYRO_SAMPLE_DEVIDE variable
 * @param     rate_hz    Sample Rate in Hz
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_GYRO_Output_Sample_Rate (GebraBit_ICM20649 * ICM20649 , uint16_t rate_hz)
{
	uint8_t gfchoice , gdlpf ;
	GB_ICM20649_Read_Reg_Bits(ICM20649_GYRO_CONFIG_1,BANK_2, START_MSB_BIT_AT_0, BIT_LENGTH_1 , &gfchoice);
  GB_ICM20649_Read_Reg_Bits(ICM20649_GYRO_CONFIG_1,BANK_2, START_MSB_BIT_AT_5, BIT_LENGTH_3 , &gdlpf);
	if((gfchoice==1)&&(0<gdlpf)&&(gdlpf<7))
	{
		ICM20649->GYRO_SAMPLE_RATE = rate_hz ;
    ICM20649->GYRO_SAMPLE_DEVIDE=(GYRO_INTERNAL_SAMPLE_RATE/rate_hz)-1;
		GB_ICM20649_Write_Reg_Data( ICM20649_GYRO_SMPLRT_DIV ,BANK_2,ICM20649->GYRO_SAMPLE_DEVIDE); 
	}
}
/*=========================================================================================================================================
 * @brief     Set Accelerometer Full Scale Range and select sensor SCALE FACTOR
 * @param     ICM20649   ICM20649 struct ACCEL_FULL_SCALE and ACCEL_SCALE_FACTOR variable
 * @param     fs         Determines Full Scale Range among  4g , 8g , 16g , 30g
 * @return    Nothing
 ========================================================================================================================================*/
void GB_ICM20649_ACCEL_Full_Scale ( GebraBit_ICM20649 * ICM20649 , ICM20649_Accel_Fs_Sel fs ) 
{
  GB_ICM20649_Write_Reg_Bits( ICM20649_ACCEL_CONFIG,BANK_2, START_MSB_BIT_AT_2, BIT_LENGTH_2 , fs);
	ICM20649->ACCEL_FULL_SCALE =  fs ;
	switch(fs)
	 {
	  case FULL_SCALE_4g:
		ICM20649->ACCEL_SCALE_FACTOR = SCALE_FACTOR_8192_LSB_g ;
    break;
		case FULL_SCALE_8g:
		ICM20649->ACCEL_SCALE_FACTOR = SCALE_FACTOR_4096_LSB_g ;
    break;	
		case FULL_SCALE_16g:
		ICM20649->ACCEL_SCALE_FACTOR = SCALE_FACTOR_2048_LSB_g ;
    break;	
		case FULL_SCALE_30g: 
		ICM20649->ACCEL_SCALE_FACTOR = SCALE_FACTOR_1024_LSB_g ;
    break;			
		default:
		ICM20649->ACCEL_SCALE_FACTOR = SCALE_FACTOR_8192_LSB_g ;		
	 }
}

/*=========================================================================================================================================
 * @brief     Enable Or Bypass Accelerometer Low Pass Filter
 * @param     ICM20649     ICM20649 Struct ACCEL_FCHOICEB variable
 * @param     bypass       Determines ENABLE_DLPF_FCHOICEB_1 or BYPASS_DLPF_FCHOICEB_0
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_ICM20649_ACCEL_Low_Pass_Filter  (GebraBit_ICM20649 * ICM20649 ,  ICM20649_FCHOICEB bypass )
{
	GB_ICM20649_Write_Reg_Bits(ICM20649_ACCEL_CONFIG,BANK_2 , START_MSB_BIT_AT_0, BIT_LENGTH_1,  bypass);
	ICM20649->ACCEL_FCHOICEB =bypass ;   
}
/*=========================================================================================================================================
 * @brief     Set Accelerometer Low Pass Filter value
 * @param     ICM20649     ICM20649 Struct ACCEL_DLPF variable
 * @param     dlpf     Low Pass Filter value 
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_ICM20649_ACCEL_Low_Pass_Filter_Value  (GebraBit_ICM20649 * ICM20649 , ICM20649_ACCEL_DLPF dlpf )
{
	  GB_ICM20649_Write_Reg_Bits(ICM20649_ACCEL_CONFIG,BANK_2 , START_MSB_BIT_AT_5, BIT_LENGTH_3,  dlpf);
	  ICM20649->ACCEL_DLPF =  dlpf ;
}
/*=========================================================================================================================================
 * @brief     Set  Accelerometer Averaging Filter
 * @param     ICM20649  ICM20649 Struct ACCEL_AVERAGING_FILTER variable
 * @param     avg       Averaging value
 * @return    Nothing 
 ========================================================================================================================================*/ 		
void GB_ICM20649_ACCEL_LP_Averaging_Filter  (GebraBit_ICM20649 * ICM20649 , ICM20649_ACCEL_Averaging_Filter avg )
{
	  GB_ICM20649_Write_Reg_Bits(ICM20649_ACCEL_CONFIG_2,BANK_2 , START_MSB_BIT_AT_1, BIT_LENGTH_2,  avg);
	  ICM20649->ACCEL_AVERAGING_FILTER =  avg ;
}
/*=========================================================================================================================================
 * @brief     Set Accelerometer Sensor Output Sample Rate that controls Accelerometer data output rate, FIFO sample rate
 * @param     ICM20649   ICM20649 struct ACCEL_SAMPLE_RATE and ACCEL_SAMPLE_DEVIDE variable
 * @param     rate_hz    Sample Rate in Hz
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_ACCEL_Output_Sample_Rate (GebraBit_ICM20649 * ICM20649 , uint16_t rate_hz)
{
	uint8_t afchoice , adlpf ;
	GB_ICM20649_Read_Reg_Bits(ICM20649_ACCEL_CONFIG,BANK_2, START_MSB_BIT_AT_0, BIT_LENGTH_1 , &afchoice);
  GB_ICM20649_Read_Reg_Bits(ICM20649_ACCEL_CONFIG,BANK_2, START_MSB_BIT_AT_5, BIT_LENGTH_3 , &adlpf);
	if((afchoice==1)&&(0<adlpf)&&(adlpf<7))
	{
		ICM20649->ACCEL_SAMPLE_RATE = rate_hz ;
    ICM20649->ACCEL_SAMPLE_DEVIDE=(GYRO_INTERNAL_SAMPLE_RATE/rate_hz)-1;
	  GB_ICM20649_Write_Reg_Data( ICM20649_ACCEL_SMPLRT_DIV_1 ,BANK_2,(ICM20649->ACCEL_SAMPLE_DEVIDE >> 8) & 0xFF);
		GB_ICM20649_Write_Reg_Data( ICM20649_ACCEL_SMPLRT_DIV_2 ,BANK_2,ICM20649->ACCEL_SAMPLE_DEVIDE & 0xFF); 
	}
}
/*=========================================================================================================================================
 * @brief     Set Temperature Low Pass Filter value
 * @param     ICM20649     ICM20649 Struct TEMP_DLPF variable
 * @param     dlpf     Low Pass Filter value 
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_ICM20649_TEMP_Low_Pass_Filter_Value  (GebraBit_ICM20649 * ICM20649 , ICM20649_TEMP_DLPF tdlpf )
{
	  GB_ICM20649_Write_Reg_Bits(ICM20649_TEMP_CONFIG,BANK_2 , START_MSB_BIT_AT_2, BIT_LENGTH_3,  tdlpf);
	  ICM20649->TEMP_DLPF =  tdlpf ;
}
/*=========================================================================================================================================
 * @brief     Configure FIFO
 * @param     ICM20649       ICM20649 Struct FIFO variable
 * @param     fifo           Configure ICM20689 FIFO according it is FIFO_DISABLE or FIFO_ENABLE 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_FIFO_Configuration ( GebraBit_ICM20649 * ICM20649 , ICM20649_FIFO_Ability fifo  )
{
	ICM20649->FIFO_PACKET_QTY = FIFO_DATA_BUFFER_SIZE / BYTE_QTY_IN_ONE_PACKET ;  
	if( fifo==Enable )  
	{
		ICM20649->FIFO = FIFO_ENABLE  ;
		GB_ICM20649_FIFO_Reset();
		GB_ICM20649_Access_Serial_Interface_To_FIFO( ICM20649 , Enable );
		GB_ICM20649_FIFO_Mode ( ICM20649 , STOP_ON_FULL_FIFO_SNAPSHOT );
		GB_ICM20649_Write_GYRO_FIFO ( ICM20649 , Enable );
	  GB_ICM20649_Write_ACCEL_FIFO( ICM20649 , Enable );
		GB_ICM20649_Write_TEMP_FIFO ( ICM20649 , Enable );
		Interrupt_Status_Enable( ICM20649 , Disable  );
	}
	else if ( fifo == Disable )
	{
		ICM20649->FIFO = FIFO_DISABLE  ;
		GB_ICM20649_Write_GYRO_FIFO ( ICM20649 , Disable );
	  GB_ICM20649_Write_ACCEL_FIFO( ICM20649 , Disable );
		GB_ICM20649_Write_TEMP_FIFO ( ICM20649 , Disable );
		GB_ICM20649_Access_Serial_Interface_To_FIFO( ICM20649 , Disable );
		GB_ICM20649_FIFO_Reset( );
	}
}
/*=========================================================================================================================================
 * @brief     Set ICM20649 Power Management
 * @param     ICM20649   ICM20649 Struct ACCEL_POWER_MODE and GYRO_POWER_MODE  variable
 * @param     pmode        Determines ICM20649 Accelerometer Power Mode in ICM20649_LOW_NOISE or ICM20689_LOW_POWER or ICM20689_SLEEP_OFF
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Set_Power_Management(GebraBit_ICM20649 * ICM20649 , ICM20649_Power_Mode pmode) 
{	
	
 GB_ICM20649_Temperature(ICM20649 , Enable );
 GB_ICM20649_Accelerometer(ICM20649 , SENSOR_ENABLE );
 GB_ICM20649_Gyroscope(ICM20649 , SENSOR_ENABLE );
 if(pmode==ICM20649_LOW_POWER)
 {
	GB_ICM20649_Sleep_Awake(ICM20649 , ICM20649_AWAKE );
  GB_ICM20649_GYRO_Power_Mode (ICM20649 , ICM20649_LOW_POWER );
  GB_ICM20649_ACCEL_Power_Mode(ICM20649 , ICM20649_LOW_POWER );		 
 }
  else if(pmode==ICM20649_LOW_NOISE)
 {
	GB_ICM20649_Sleep_Awake(ICM20649 , ICM20649_AWAKE );
  GB_ICM20649_GYRO_Power_Mode (ICM20649 , ICM20649_LOW_NOISE );
  GB_ICM20649_ACCEL_Power_Mode(ICM20649 , ICM20649_LOW_NOISE );	  
 }
 else if (pmode==ICM20649_SLEEP_OFF)
 {
	ICM20649->ACCEL_POWER_MODE = ICM20649_SLEEP_OFF ;
  ICM20649->ACCEL_POWER_MODE = ICM20649_SLEEP_OFF ;
	GB_ICM20649_Temperature(ICM20649 , Disable );
	GB_ICM20649_Accelerometer(ICM20649 , SENSOR_DISABLE );
	GB_ICM20649_Gyroscope(ICM20649 , SENSOR_DISABLE );
	GB_ICM20649_Sleep_Awake(ICM20649 , ICM20649_SLEEP );
 }
 GB_ICM20649_Write_Reg_Bits (ICM20649_PWR_MGMT_1,BANK_0, START_MSB_BIT_AT_5, BIT_LENGTH_1 , pmode);//reduce current in low power
 HAL_Delay(1);
}

/*=========================================================================================================================================
 * @brief     initialize ICM20649
 * @param     ICM20649     initialize ICM20649
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_initialize( GebraBit_ICM20649 * ICM20649 )
{
  HAL_Delay(3);
  GB_ICM20649_Who_am_I(ICM20649);
	GB_ICM20649_Soft_Reset(ICM20649);
	GB_ICM20649_DMP( ICM20649 ,Disable ,DMP_LOW_POWER );
	GB_ICM20649_Select_SPI4_Interface(ICM20649 , IS_SPI);
	GB_ICM20649_Set_Power_Management( ICM20649 , ICM20649_LOW_NOISE );
	GB_ICM20649_Set_Clock_Source( ICM20649 , AUTO_SELECT );
	GB_ICM20649_GYRO_Low_Pass_Filter (ICM20649,ENABLE_DLPF_FCHOICEB_1);
	GB_ICM20649_ACCEL_Low_Pass_Filter(ICM20649,ENABLE_DLPF_FCHOICEB_1);
	GB_ICM20649_GYRO_Low_Pass_Filter_Value (ICM20649,ICM20649_GYRO_DLPF_152);
	GB_ICM20649_ACCEL_Low_Pass_Filter_Value(ICM20649,ICM20649_ACCEL_DLPF_246);
	GB_ICM20649_TEMP_Low_Pass_Filter_Value (ICM20649,ICM20649_TEMP_DLPF_218 );
	GB_ICM20649_FIFO_Configuration ( ICM20649 ,FIFO_DISABLE ) ;
	GB_ICM20649_Set_INT1_Pin( ICM20649 , ACTIVE_LOW  , OPEN_DRAIN  ,  HELD_STATUS_CLEAR );
	GB_ICM20649_Set_INT2_Pin( ICM20649 , ACTIVE_LOW  , OPEN_DRAIN  ,  HELD_STATUS_CLEAR );
  //Interrupt_Status_Enable( ICM20649 , Disable  );
}
/*=========================================================================================================================================
 * @brief     Configure ICM20649
 * @param     ICM20649  Configure ICM20649 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Configuration(GebraBit_ICM20649 * ICM20649, ICM20649_FIFO_Ability fifo)
{
	GB_ICM20649_GYRO_Output_Sample_Rate (ICM20649,GYRO_ODR_HZ );
	GB_ICM20649_ACCEL_Output_Sample_Rate(ICM20649,ACCEL_ODR_HZ );
	GB_ICM20649_GYRO_Full_Scale ( ICM20649 ,FS_1000_DPS ) ;
	GB_ICM20649_ACCEL_Full_Scale( ICM20649 ,FULL_SCALE_4g ) ; 
	GB_ICM20649_FIFO_Configuration ( ICM20649 ,fifo ) ;
	HAL_Delay(20);	
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Temprature from Register 
 * @param     ICM20649  store Raw Data Of Temprature in GebraBit_ICM20649 Staruct REGISTER_RAW_TEMP
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Get_Temp_Register_Raw_Data(GebraBit_ICM20649 * ICM20649)
{
	uint8_t temp_msb , temp_lsb;
  GB_ICM20649_Read_Reg_Data(ICM20649_TEMP_OUT_H, BANK_0, &temp_msb);
	GB_ICM20649_Read_Reg_Data(ICM20649_TEMP_OUT_L, BANK_0, &temp_lsb);
	ICM20649->REGISTER_RAW_TEMP = (int16_t)((temp_msb << 8) | temp_lsb);
}

/*=========================================================================================================================================
 * @brief     Get Valid Data Of Temprature Base on Datasheet Formula 
 * @param     ICM20649  store Valid Data Of Temprature in GebraBit_ICM20649 Staruct VALID_TEMP_DATA
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Get_Temp_Valid_Data(GebraBit_ICM20649 * ICM20649)
{ 
  ICM20649->VALID_TEMP_DATA =(ICM20649->REGISTER_RAW_TEMP / 333.87 ) + 21-ROOM_TEMPERATURE_OFFSET;///25 - 8 PCS OFSET!!!
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of X Axis GYRO from Register 
 * @param     ICM20649  store Raw Data Of X Axis GYRO DATA in GebraBit_ICM20649 Staruct REGISTER_RAW_GYRO_X
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Get_GYRO_X_Register_Raw_DATA(GebraBit_ICM20649 * ICM20649)
{
	uint8_t gyrox_msb , gyrox_lsb;
  GB_ICM20649_Read_Reg_Data( ICM20649_GYRO_XOUT_H, BANK_0, &gyrox_msb);
	GB_ICM20649_Read_Reg_Data( ICM20649_GYRO_XOUT_L, BANK_0, &gyrox_lsb );
	ICM20649->REGISTER_RAW_GYRO_X = (int16_t)((gyrox_msb << 8) | gyrox_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Y Axis GYRO from Register 
 * @param     ICM20649  store Raw Data Of Y Axis GYRO DATA in GebraBit_ICM20649 Staruct REGISTER_RAW_GYRO_Y
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Get_GYRO_Y_Register_Raw_DATA(GebraBit_ICM20649 * ICM20649)
{
	uint8_t gyroy_msb , gyroy_lsb;
  GB_ICM20649_Read_Reg_Data( ICM20649_GYRO_YOUT_H, BANK_0, &gyroy_msb);
	GB_ICM20649_Read_Reg_Data( ICM20649_GYRO_YOUT_L, BANK_0, &gyroy_lsb );
	ICM20649->REGISTER_RAW_GYRO_Y = (int16_t)((gyroy_msb << 8) | gyroy_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Z Axis GYRO from Register 
 * @param     ICM20649  store Raw Data Of Z Axis GYRO DATA in GebraBit_ICM20649 Staruct REGISTER_RAW_GYRO_Z
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Get_GYRO_Z_Register_Raw_DATA(GebraBit_ICM20649 * ICM20649)
{
	uint8_t gyroz_msb , gyroz_lsb;
  GB_ICM20649_Read_Reg_Data( ICM20649_GYRO_ZOUT_H, BANK_0, &gyroz_msb);
	GB_ICM20649_Read_Reg_Data( ICM20649_GYRO_ZOUT_L, BANK_0, &gyroz_lsb );
	ICM20649->REGISTER_RAW_GYRO_Z = (int16_t)((gyroz_msb << 8) | gyroz_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of X Axis GYRO Base on GebraBit_ICM20649 Staruct SCALE_FACTOR 
 * @param     ICM20649  store Valid Data Of X Axis GYRO in GebraBit_ICM20649 Staruct VALID_GYRO_DATA_X
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Get_GYRO_DATA_X_Valid_Data(GebraBit_ICM20649 * ICM20649)
{
  ICM20649->VALID_GYRO_DATA_X =(ICM20649->REGISTER_RAW_GYRO_X /ICM20649->PRECISE_GYRO_SF);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of Y Axis GYRO Base on GebraBit_ICM20649 Staruct SCALE_FACTOR 
 * @param     ICM20649  store Valid Data Of Y Axis GYRO in GebraBit_ICM20649 Staruct VALID_GYRO_DATA_Y
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Get_GYRO_DATA_Y_Valid_Data(GebraBit_ICM20649 * ICM20649)
{
  ICM20649->VALID_GYRO_DATA_Y =(ICM20649->REGISTER_RAW_GYRO_Y /ICM20649->PRECISE_GYRO_SF);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of Z Axis GYRO Base on GebraBit_ICM20649 Staruct SCALE_FACTOR 
 * @param     ICM20649  store Valid Data Of Z Axis GYRO in GebraBit_ICM20649 Staruct VALID_GYRO_DATA_Z
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Get_GYRO_DATA_Z_Valid_Data(GebraBit_ICM20649 * ICM20649)
{
  ICM20649->VALID_GYRO_DATA_Z =(ICM20649->REGISTER_RAW_GYRO_Z /ICM20649->PRECISE_GYRO_SF);
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of X Axis ACCEL from Register 
 * @param     ICM20649  store Raw Data Of X Axis ACCEL DATA in GebraBit_ICM20649 Staruct REGISTER_RAW_ACCEL_X
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Get_ACCEL_X_Register_Raw_DATA(GebraBit_ICM20649 * ICM20649)
{
	uint8_t accelx_msb , acclx_lsb;
  GB_ICM20649_Read_Reg_Data( ICM20649_ACCEL_XOUT_H, BANK_0, &accelx_msb);
	GB_ICM20649_Read_Reg_Data( ICM20649_ACCEL_XOUT_L, BANK_0, &acclx_lsb );
	ICM20649->REGISTER_RAW_ACCEL_X = (int16_t)((accelx_msb << 8) | acclx_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Y Axis ACCEL from Register 
 * @param     ICM20649  store Raw Data Of Y Axis ACCEL DATA in GebraBit_ICM20649 Staruct REGISTER_RAW_ACCEL_Y
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Get_ACCEL_Y_Register_Raw_DATA(GebraBit_ICM20649 * ICM20649)
{
	uint8_t accely_msb , accly_lsb;
  GB_ICM20649_Read_Reg_Data( ICM20649_ACCEL_YOUT_H, BANK_0, &accely_msb);
	GB_ICM20649_Read_Reg_Data( ICM20649_ACCEL_YOUT_L, BANK_0, &accly_lsb );
	ICM20649->REGISTER_RAW_ACCEL_Y = (int16_t)((accely_msb << 8) | accly_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Z Axis ACCEL from Register 
 * @param     ICM20649  store Raw Data Of Z Axis ACCEL DATA in GebraBit_ICM20649 Staruct REGISTER_RAW_ACCEL_Z
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Get_ACCEL_Z_Register_Raw_DATA(GebraBit_ICM20649 * ICM20649)
{
	uint8_t accelz_msb , acclz_lsb;
  GB_ICM20649_Read_Reg_Data( ICM20649_ACCEL_ZOUT_H, BANK_0, &accelz_msb);
	GB_ICM20649_Read_Reg_Data( ICM20649_ACCEL_ZOUT_L, BANK_0, &acclz_lsb );
	ICM20649->REGISTER_RAW_ACCEL_Z = (int16_t)((accelz_msb << 8) | acclz_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of X Axis ACCEL Base on GebraBit_ICM20649 Staruct SCALE_FACTOR 
 * @param     ICM20649  store Valid Data Of X Axis ACCEL in GebraBit_ICM20649 Staruct VALID_ACCEL_DATA_X
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Get_ACCEL_DATA_X_Valid_Data(GebraBit_ICM20649 * ICM20649)
{
	float scale_factor = ICM20649->ACCEL_SCALE_FACTOR;
  ICM20649->VALID_ACCEL_DATA_X =(ICM20649->REGISTER_RAW_ACCEL_X /scale_factor);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of Y Axis ACCEL Base on GebraBit_ICM20649 Staruct SCALE_FACTOR 
 * @param     ICM20649  store Valid Data Of Y Axis ACCEL in GebraBit_ICM20649 Staruct VALID_ACCEL_DATA_Y
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Get_ACCEL_DATA_Y_Valid_Data(GebraBit_ICM20649 * ICM20649)
{
	float scale_factor = ICM20649->ACCEL_SCALE_FACTOR;
  ICM20649->VALID_ACCEL_DATA_Y =(ICM20649->REGISTER_RAW_ACCEL_Y /scale_factor);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of Z Axis ACCEL Base on GebraBit_ICM20649 Staruct SCALE_FACTOR 
 * @param     ICM20649  store Valid Data Of Z Axis ACCEL in GebraBit_ICM20649 Staruct VALID_ACCEL_DATA_Z
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Get_ACCEL_DATA_Z_Valid_Data(GebraBit_ICM20649 * ICM20649)
{
	float scale_factor = ICM20649->ACCEL_SCALE_FACTOR;
  ICM20649->VALID_ACCEL_DATA_Z =(ICM20649->REGISTER_RAW_ACCEL_Z /scale_factor);
}
/*=========================================================================================================================================
 * @brief     Get Temprature Directly 
 * @param     ICM20649       GebraBit_ICM20649 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Get_Temperature(GebraBit_ICM20649 * ICM20649)
{
  GB_ICM20649_Get_Temp_Register_Raw_Data  (ICM20649);
	GB_ICM20649_Get_Temp_Valid_Data(ICM20649);
}
/*=========================================================================================================================================
 * @brief     Get XYZ ACCELERATION Directly 
 * @param     ICM20649       GebraBit_ICM20649 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Get_XYZ_GYROSCOPE(GebraBit_ICM20649 * ICM20649)
{
	GB_ICM20649_Get_GYRO_X_Register_Raw_DATA(ICM20649);
	GB_ICM20649_Get_GYRO_DATA_X_Valid_Data(ICM20649);
	GB_ICM20649_Get_GYRO_Y_Register_Raw_DATA(ICM20649);
	GB_ICM20649_Get_GYRO_DATA_Y_Valid_Data(ICM20649);
	GB_ICM20649_Get_GYRO_Z_Register_Raw_DATA(ICM20649);
	GB_ICM20649_Get_GYRO_DATA_Z_Valid_Data(ICM20649);
}
/*=========================================================================================================================================
 * @brief     Get XYZ ACCELERATION Directly 
 * @param     ICM20649       GebraBit_ICM20649 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Get_XYZ_ACCELERATION(GebraBit_ICM20649 * ICM20649)
{
	GB_ICM20649_Get_ACCEL_X_Register_Raw_DATA(ICM20649);
	GB_ICM20649_Get_ACCEL_DATA_X_Valid_Data(ICM20649);
	GB_ICM20649_Get_ACCEL_Y_Register_Raw_DATA(ICM20649);
	GB_ICM20649_Get_ACCEL_DATA_Y_Valid_Data(ICM20649);
	GB_ICM20649_Get_ACCEL_Z_Register_Raw_DATA(ICM20649);
	GB_ICM20649_Get_ACCEL_DATA_Z_Valid_Data(ICM20649);
}
/*=========================================================================================================================================
 * @brief     Get XYZ ACCELERATION , GYRO and Temprature From FIFO
 * @param     ICM20649       GebraBit_ICM20649 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Get_ACCEL_GYRO_TEMP_From_Registers(GebraBit_ICM20649 * ICM20649)
{
  if (IS_Ready==GB_ICM20649_Check_Data_Preparation(ICM20649))
	 {
		 ICM20649->GET_DATA =  FROM_REGISTER ; 
	   GB_ICM20649_Get_Temperature( ICM20649 );
	   GB_ICM20649_Get_XYZ_ACCELERATION( ICM20649);
		 GB_ICM20649_Get_XYZ_GYROSCOPE( ICM20649);
	 }
}
/*=========================================================================================================================================
 * @brief     Separate XYZ ACCELERATION , GYROSCOPE and Temprature Data From FIFO and caculate Valid data
 * @param     ICM20649  store Valid Data Of XYZ ACCEL Axis and temp from FIFO TO GebraBit_ICM20649 Staruct VALID_FIFO_DATA_X , VALID_FIFO_DATA_Y ,VALID_FIFO_DATA_Z
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_FIFO_Data_Partition_ACCEL_GYRO_XYZ_TEMP(GebraBit_ICM20649 * ICM20649)
{
	uint16_t i,offset=0;
  float accel_scale_factor = ICM20649->ACCEL_SCALE_FACTOR;
 if ( (ICM20649->TEMP_TO_FIFO == Enable ) && ( ICM20649->ACCEL_TO_FIFO == Enable ) && ( ICM20649->GYRO_TO_FIFO == Enable ) )
	{
	 for ( i = 0 ; i < (PACKET_QTY_IN_FULL_FIFO-1) ; i++ )	 
		{
			ICM20649->VALID_FIFO_ACCEL_X[i] = ((int16_t)( (ICM20649->FIFO_DATA[offset] << 8) | ICM20649->FIFO_DATA[offset+1]))/accel_scale_factor ;
			offset += 2; 
			ICM20649->VALID_FIFO_ACCEL_Y[i] = ((int16_t)( (ICM20649->FIFO_DATA[offset] << 8) | ICM20649->FIFO_DATA[offset+1]))/accel_scale_factor ;
			offset += 2;
			ICM20649->VALID_FIFO_ACCEL_Z[i] = ((int16_t)( (ICM20649->FIFO_DATA[offset] << 8) | ICM20649->FIFO_DATA[offset+1]))/accel_scale_factor ;
			offset += 2;
			ICM20649->VALID_FIFO_GYRO_X[i]  = ((int16_t)( (ICM20649->FIFO_DATA[offset] << 8) | ICM20649->FIFO_DATA[offset+1]))/ICM20649->PRECISE_GYRO_SF ;
			offset += 2; 
			ICM20649->VALID_FIFO_GYRO_Y[i]  = ((int16_t)( (ICM20649->FIFO_DATA[offset] << 8) | ICM20649->FIFO_DATA[offset+1]))/ICM20649->PRECISE_GYRO_SF ;
			offset += 2;
			ICM20649->VALID_FIFO_GYRO_Z[i]  = ((int16_t)( (ICM20649->FIFO_DATA[offset] << 8) | ICM20649->FIFO_DATA[offset+1]))/ICM20649->PRECISE_GYRO_SF ;
			offset += 2;
			ICM20649->VALID_FIFO_TEMP[i]    = (((int16_t)( (ICM20649->FIFO_DATA[offset] << 8)| ICM20649->FIFO_DATA[offset+1]))/ 333.87) + 25-ROOM_TEMPERATURE_OFFSET ;
			offset += 2;
		} 
			ICM20649->VALID_FIFO_ACCEL_X[292] = ((int16_t)( (ICM20649->FIFO_DATA[offset] << 8) | ICM20649->FIFO_DATA[offset+1]))/accel_scale_factor ;
			offset += 2; 
			ICM20649->VALID_FIFO_ACCEL_Y[292] = ((int16_t)( (ICM20649->FIFO_DATA[offset] << 8) | ICM20649->FIFO_DATA[offset+1]))/accel_scale_factor ;
			offset += 2;
			ICM20649->VALID_FIFO_ACCEL_Z[292] = ((int16_t)( (ICM20649->FIFO_DATA[offset] << 8) | ICM20649->FIFO_DATA[offset+1]))/accel_scale_factor ;
			offset += 2;
			ICM20649->VALID_FIFO_GYRO_X[292]  = ((int16_t)( (ICM20649->FIFO_DATA[offset] << 8) | ICM20649->FIFO_DATA[offset+1]))/ICM20649->PRECISE_GYRO_SF ;
	}
}
/*=========================================================================================================================================
 * @brief     Get XYZ ACCELERATION , GYRO and Temprature From FIFO
 * @param     ICM20649       GebraBit_ICM20649 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Get_ACCEL_GYRO_TEMP_From_FIFO(GebraBit_ICM20649 * ICM20649)
{	  
	if (IS_Ready==GB_ICM20649_Check_Data_Preparation(ICM20649))
	{
		  if (FIFO_OVERFLOW == GB_ICM20649_Check_FIFO_Overflow(ICM20649))
		  {
        GB_ICM20649_GET_FIFO_Count(ICM20649);
				GB_ICM20649_Read_FIFO(ICM20649,FIFO_DATA_BUFFER_SIZE);				
				GB_ICM20649_FIFO_Data_Partition_ACCEL_GYRO_XYZ_TEMP(ICM20649); 
        //memset(ICM20649->FIFO_DATA , 0, FIFO_DATA_BUFFER_SIZE*sizeof(uint8_t));				
				GB_ICM20649_FIFO_Reset();
				ICM20649->GET_DATA =  FROM_FIFO ;
		  } 
	}	
}
/*=========================================================================================================================================
 * @brief     Get Data From ICM20649
 * @param     ICM20649       GebraBit_ICM20649 Staruct
 * @param     get_data       Determine Method of reading data from sensoe : FROM_REGISTER or FROM_FIFO
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_Get_Data(GebraBit_ICM20649 * ICM20649 , ICM20649_Get_DATA get_data)
{
 if( (get_data == FROM_REGISTER)&&(ICM20649->FIFO == Disable) )
	 GB_ICM20649_Get_ACCEL_GYRO_TEMP_From_Registers(ICM20649);
 else if ((get_data == FROM_FIFO)&&(ICM20649->FIFO == Enable)) 
	GB_ICM20649_Get_ACCEL_GYRO_TEMP_From_FIFO(ICM20649); 
}
/*----------------------------------------------------------------------------------------------------------------------------------------*
 *                                                                      End                                                               *
 *----------------------------------------------------------------------------------------------------------------------------------------*/
