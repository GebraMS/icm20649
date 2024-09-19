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
#ifndef	_ICM20649__H_
#define	_ICM20649__H_
#include "main.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdio.h"
#include "spi.h"
#include "string.h"
#include "math.h"
/* register for all banks */
#define ICM20649_BANK_SEL            0x7F
/************************************************
 *         USER BANK 0 REGISTER MAP             *
 ***********************************************/ 
#define ICM20649_WHO_AM_I            (0x00)
#define ICM20649_USER_CTRL           (0x03)
#define ICM20649_LP_CONFIG           (0x05)
#define ICM20649_PWR_MGMT_1          (0x06)
#define ICM20649_PWR_MGMT_2          (0x07)
#define ICM20649_INT_PIN_CFG         (0x0F)
#define ICM20649_INT_ENABLE          (0x10)
#define ICM20649_INT_ENABLE_1        (0x11)
#define ICM20649_INT_ENABLE_2        (0x12)
#define ICM20649_INT_ENABLE_3        (0x13)
#define ICM20649_I2C_MST_STATUS      (0x17)
#define ICM20649_INT_STATUS          (0x19)
#define ICM20649_INT_STATUS_1        (0x1A)
#define ICM20649_INT_STATUS_2        (0x1B)
#define ICM20649_INT_STATUS_3        (0x1C)
#define ICM20649_DELAY_TIMEH         (0x28)
#define ICM20649_DELAY_TIMEL         (0x29)
#define ICM20649_ACCEL_XOUT_H        (0x2D)
#define ICM20649_ACCEL_XOUT_L        (0x2E)
#define ICM20649_ACCEL_YOUT_H        (0x2F)
#define ICM20649_ACCEL_YOUT_L        (0x30)
#define ICM20649_ACCEL_ZOUT_H        (0x31)
#define ICM20649_ACCEL_ZOUT_L        (0x32)
#define ICM20649_GYRO_XOUT_H         (0x33)
#define ICM20649_GYRO_XOUT_L         (0x34)
#define ICM20649_GYRO_YOUT_H         (0x35)
#define ICM20649_GYRO_YOUT_L         (0x36)
#define ICM20649_GYRO_ZOUT_H         (0x37)
#define ICM20649_GYRO_ZOUT_L         (0x38)
#define ICM20649_TEMP_OUT_H          (0x39)
#define ICM20649_TEMP_OUT_L          (0x3A)
#define ICM20649_FIFO_EN_1           (0x66)
#define ICM20649_FIFO_EN_2           (0x67)
#define ICM20649_FIFO_RST            (0x68)
#define ICM20649_FIFO_MODE           (0x69)
#define ICM20649_FIFO_COUNTH         (0x70)
#define ICM20649_FIFO_COUNTL         (0x71)
#define ICM20649_FIFO_R_W            (0x72)
#define ICM20649_DATA_RDY_STATUS     (0x74)
#define ICM20649_FIFO_CFG            (0x76)
/*----------------------------------------------*
 *        USER BANK 0 REGISTER MAP End          *
 *----------------------------------------------*/ 
/************************************************
 *         USER BANK 1 REGISTER MAP             *
 ***********************************************/ 
#define ICM20649_TIMEBASE_CORRECTION_PLL   (0x28)
/*----------------------------------------------*
 *        USER BANK 1 REGISTER MAP End          *
 *----------------------------------------------*/ 
/************************************************
 *         USER BANK 2 REGISTER MAP             *
 ***********************************************/ 
#define ICM20649_GYRO_SMPLRT_DIV     (0x00)
#define ICM20649_GYRO_CONFIG_1       (0x01)
#define ICM20649_GYRO_CONFIG_2       (0x02)
#define ICM20649_ODR_ALIGN_EN        (0x09)
#define ICM20649_ACCEL_SMPLRT_DIV_1  (0x10)
#define ICM20649_ACCEL_SMPLRT_DIV_2  (0x11)
#define ICM20649_ACCEL_INTEL_CTRL    (0x12)
#define ICM20649_ACCEL_WOM_THR       (0x13)
#define ICM20649_ACCEL_CONFIG        (0x14)
#define ICM20649_ACCEL_CONFIG_2      (0x15)
#define ICM20649_FSYNC_CONFIG        (0x52)
#define ICM20649_TEMP_CONFIG         (0x53)
#define ICM20649_MOD_CTRL_USR        (0x54)
/*----------------------------------------------*
 *        USER BANK 2 REGISTER MAP End          *
 *----------------------------------------------*/ 

/************************************************
 *         USER BANK 3 REGISTER MAP             *
 ***********************************************/ 
#define ICM20649_I2C_MST_ODR_CONFIG  (0x0)
#define ICM20649_I2C_MST_CTRL        (0x01)
#define ICM20649_I2C_MST_DELAY_CTRL  (0x02)
#define ICM20649_I2C_SLV0_ADDR       (0x03)
#define ICM20649_I2C_SLV0_REG        (0x04)
#define ICM20649_I2C_SLV0_CTRL       (0x05)
#define ICM20649_I2C_SLV0_DO         (0x06)
#define ICM20649_I2C_SLV1_ADDR       (0x07)
#define ICM20649_I2C_SLV1_REG        (0x08)
#define ICM20649_I2C_SLV1_CTRL       (0x09)
#define ICM20649_I2C_SLV1_DO         (0x0A)
#define ICM20649_I2C_SLV2_ADDR       (0x0B)
#define ICM20649_I2C_SLV2_REG        (0x0C)
#define ICM20649_I2C_SLV2_CTRL       (0x0D)
#define ICM20649_I2C_SLV2_DO         (0x0E)
#define ICM20649_I2C_SLV3_ADDR       (0x0F)
#define ICM20649_I2C_SLV3_REG        (0x10)
#define ICM20649_I2C_SLV3_CTRL       (0x11)
#define ICM20649_I2C_SLV3_DO         (0x12)
#define ICM20649_I2C_SLV4_ADDR       (0x13)
#define ICM20649_I2C_SLV4_REG        (0x14)
#define ICM20649_I2C_SLV4_CTRL       (0x15)
#define ICM20649_I2C_SLV4_DO         (0x16)
#define ICM20649_I2C_SLV4_DI         (0x17)

/*----------------------------------------------*
 *        USER BANK 3 REGISTER MAP End          *
 *----------------------------------------------*/ 
/************************************************
 *         MSB Bit Start Location Begin         *
 ***********************************************/ 
#define START_MSB_BIT_AT_0                    0
#define START_MSB_BIT_AT_1                    1
#define START_MSB_BIT_AT_2                    2
#define START_MSB_BIT_AT_3                    3
#define START_MSB_BIT_AT_4                    4
#define START_MSB_BIT_AT_5                    5
#define START_MSB_BIT_AT_6                    6
#define START_MSB_BIT_AT_7                    7
/*----------------------------------------------*
 *        MSB Bit Start Location End            *
 *----------------------------------------------*/ 
/************************************************
 *          Bit Field Length Begin              *
 ***********************************************/ 
#define BIT_LENGTH_1                          1
#define BIT_LENGTH_2                          2
#define BIT_LENGTH_3                          3
#define BIT_LENGTH_4                          4
#define BIT_LENGTH_5                          5
#define BIT_LENGTH_6                          6
#define BIT_LENGTH_7                          7
#define BIT_LENGTH_8                          8
/*----------------------------------------------*
 *          Bit Field Length End                *
 *----------------------------------------------*/
 /************************************************
 *          Register Values Begin                *
 ***********************************************/ 
#define FIFO_DATA_BUFFER_SIZE         4096
#define BYTE_QTY_IN_ONE_PACKET        14
#define ROOM_TEMPERATURE_OFFSET       3.5
#define PACKET_QTY_IN_FULL_FIFO       293
#define ACCEL_INTERNAL_SAMPLE_RATE    1125
#define GYRO_INTERNAL_SAMPLE_RATE     1125
#define GYRO_ODR_HZ                   500
#define ACCEL_ODR_HZ                  500
/*----------------------------------------------*
 *           Register Values End                *
 *----------------------------------------------*/
/*
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/**************************************************
 * Values For BANK_SEL in REG_BANK_SEL Register   *
 **************************************************/ 
typedef enum bank_sel
{  
	BANK_0 = 0 ,                   								   				/* Register Bank 0 selection */
	BANK_1     ,                    							   				/* Register Bank 1 selection */
	BANK_2     ,                     								 				/* Register Bank 2 selection */
	BANK_3                          								        /* Register Bank 3 selection */      
}ICM20649_Bank_Sel;
/****************************************************
 *    Values For I2C_IF_DIS in USER_CTRL Register   *
 ****************************************************/ 
typedef enum interface
{  
	NOT_SPI = 0     ,                  						            
	IS_SPI                                 					        /* 1: Disable I2C Slave module and put the serial interface in SPI mode only */  
}ICM20649_Interface;
/******************************************************
 * Values For ACCEL_FS_SEL in ACCEL_CONFIG   Register *
 ******************************************************/ 
typedef enum accel_fs_sel
{  
	FULL_SCALE_4g = 0 ,                    							    /* 00: ±4g   Full scale select for accelerometer */
	FULL_SCALE_8g      ,                							      /* 01: ±8g   Full scale select for accelerometer */
	FULL_SCALE_16g      ,               								    /* 10: ±16g  Full scale select for accelerometer */
	FULL_SCALE_30g            															/* 11: ±30g  Full scale select for accelerometer */
}ICM20649_Accel_Fs_Sel;
/**************************************************
 *           Values For ACCEL Scale_Factor              *
 **************************************************/ 
typedef enum 
{  
	SCALE_FACTOR_8192_LSB_g  = 8192    ,                    /* 00: ±4g   scale factor for accelerometer */    
	SCALE_FACTOR_4096_LSB_g  = 4096    ,                    /* 01: ±8g   scale factor for accelerometer */
	SCALE_FACTOR_2048_LSB_g  = 2048    ,                    /* 10: ±16g  scale factor for accelerometer */ 
	SCALE_FACTOR_1024_LSB_g  = 1024                         /* 11: ±30g  scale factor for accelerometer */
}ICM20649_Accel_Scale_Factor;
/******************************************************
 * Values For GYRO_FS_SEL in GYRO_CONFIG_1  Register  *
 ******************************************************/ 
typedef enum gyro_fs_sel
{  
	FS_500_DPS = 0   ,                    							     /* 00: ±500dps   Full scale select for accelerometer */
	FS_1000_DPS      ,                							         /* 01: ±1000dps  Full scale select for accelerometer */
	FS_2000_DPS      ,               								         /* 10: ±2000dps  Full scale select for accelerometer */
	FS_4000_DPS            																   /* 11: ±4000dps  Full scale select for accelerometer */
}ICM20649_Gyro_Fs_Sel;
/**************************************************
 *           Values For Scale_Factor              *
 **************************************************/ 
typedef enum 
{  
	SCALE_FACTOR_65p5_LSB_DPS  = 65  ,                       /* 00: ±500dps    Full scale factor for Gyro */
	SCALE_FACTOR_32p8_LSB_DPS  = 33    ,                     /* 01: ±1000dps   Full scale factor for Gyro */
	SCALE_FACTOR_16p4_LSB_DPS  = 16    ,                     /* 10: ±2000dps   Full scale factor for Gyro */   
	SCALE_FACTOR_8p2_LSB_DPS = 8                             /* 11: ±40000dps  Full scale factor for Gyro */
}ICM20649_Gyro_Scale_Factor;
/*************************************************
 * Values For FIFO_MODE in FIFO_MODE Register    *
 **************************************************/ 
typedef enum FIFO_Mode 
{  
	STREAM_TO_FIFO = 0 ,                                      /* 00000 : When set to ‘0’, when the FIFO is full, additional writes will be written to the FIFO, replacing the oldest data */
	STOP_ON_FULL_FIFO_SNAPSHOT = 31                           /* 11111 : When set to ‘1’, when the FIFO is full, additional writes will not be written to FIFO.    */
}ICM20649_FIFO_Mode ;

/**************************************************
 *     Values For Disable And Enable Functions    *
 **************************************************/ 
typedef enum 
{  
	Disable = 0     ,                      
	Enable     
}ICM20649_Ability;                       
/**************************************************
 *     Values For ACCEL And Gyro POWER_MODE       *
 **************************************************/ 
typedef enum
{
	ICM20649_LOW_NOISE  = 0,        							 					   /* 11: Places accelerometer in Low Noise (LN) Mode */
	ICM20649_LOW_POWER  = 1, 																   /* 10: Places accelerometer in Low Power (LP) Mode */                    
	ICM20649_SLEEP_OFF  = 2
} ICM20649_Power_Mode;
/**************************************************************
 *     Values For GYRO_AVGCFG in GYRO_CONFIG_2 Register       *
 **************************************************************/ 
typedef enum
{
	 GYRO_AVERAGE_1_SAMPLES_FILTER  = 0 ,                     /* 000 Average 1   samples for Low Power Accelerometer mode 				  */
	 GYRO_AVERAGE_2_SAMPLES_FILTER  = 1 ,                     /* 001 Average 2   samples for Low Power Accelerometer mode 				  */
	 GYRO_AVERAGE_4_SAMPLES_FILTER  = 2 ,										  /* 010 Average 4   samples for Low Power Accelerometer mode 				  */
	 GYRO_AVERAGE_8_SAMPLES_FILTER  = 3 ,                     /* 011 Average 8   samples for Low Power Accelerometer mode           */
	 GYRO_AVERAGE_16_SAMPLES_FILTER = 4 ,											/* 100 Average 16  samples for Low Power Accelerometer mode           */
	 GYRO_AVERAGE_32_SAMPLES_FILTER = 5	,										  /* 101 Average 32  samples for Low Power Accelerometer mode 				  */
	 GYRO_AVERAGE_64_SAMPLES_FILTER = 6	,                     /* 110 Average 64  samples for Low Power Accelerometer mode 				  */
	 GYRO_AVERAGE_128_SAMPLES_FILTER= 7	                      /* 111 Average 128 samples for Low Power Accelerometer mode 				  */
} ICM20649_GYRO_Averaging_Filter;
/**************************************************************
 *        Values For DEC3_CFG in ACCEL_CONFIG_2 Register      *
 **************************************************************/
typedef enum
{
	 ACCEL_AVERAGE_1_4_SAMPLES_FILTER  = 0 ,                   /* 00 Average 1-4 samples for Low Power Accelerometer mode 				  */
	 ACCEL_AVERAGE_8_SAMPLES_FILTER    = 1 ,                   /* 01 Average 8   samples for Low Power Accelerometer mode 				  */
	 ACCEL_AVERAGE_16_SAMPLES_FILTER   = 2 ,								   /* 10 Average 16  samples for Low Power Accelerometer mode 				  */
	 ACCEL_AVERAGE_32_SAMPLES_FILTER   = 3                     /* 11 Average 32  samples for Low Power Accelerometer mode 				  */
} ICM20649_ACCEL_Averaging_Filter;

/*************************************************
 *         Values For Data Preparation           *
 **************************************************/ 
typedef enum 
{  
	IS_NOT_Ready = 0     ,                      
	IS_Ready     
}ICM20649_Preparation;
/*************************************************
 *           Values For Reset Process             *
 **************************************************/ 
typedef enum 
{  
	DONE     = 0     ,                      
	FAILED   = 1    
}ICM20649_Reset_Status;
/**************************************************
 *       Values For Disable And Enable FIFO       *
 **************************************************/ 
typedef enum FIFO_Ability
{  
	FIFO_DISABLE = 0     ,                      
	FIFO_ENABLE     
} ICM20649_FIFO_Ability;

/**************************************************
 * Values For Methode of getting data from sensor *
 **************************************************/ 
typedef enum Get_DATA
{  
	FROM_REGISTER = 0     ,                      
	FROM_FIFO     
} ICM20649_Get_DATA; 
/**************************************************
 *        Values For which Interrupt pin          *
 **************************************************/ 
typedef enum Interrupt_Pin
{  
	INTERRUPT_ON_PIN_1 = 0     ,                      
	INTERRUPT_ON_PIN_2     
} ICM20649_Interrupt_Pin; 
/**************************************************************
 *    Values For REG_LP_DMP_EN in MOD_CTRL_USR Register       *
 **************************************************************/ 
typedef enum DMP_LP
{  
	NOT_DMP_LOW_POWER = 0     ,                      
	DMP_LOW_POWER     
} ICM20649_DMP_LP; 
/**************************************************************
 *        Values For SLEEP in  PWR_MGMT_1 Register            *
 **************************************************************/ 
typedef enum sleep
{
	ICM20649_AWAKE   = 0 ,																		
	ICM20649_SLEEP
}ICM20649_Sleep;
/**************************************************************
 *        Values For CLKSEL in PWR_MGMT_1 Register            *
 **************************************************************/ 
typedef enum Clock_Source
{  
	INTERNAL_20MHZ_OSCILLATOR = 0  ,                        /* 00: Internal 20 MHz oscillator. */
	AUTO_SELECT = 1      ,                    							/* 01: Auto selects the best available clock source – PLL if ready, else use the Internal oscillator */
	CLOCK_STOP  = 7                                         /* 10: Stops the clock and keeps timing generator in reset */
}ICM20649_CLK ;
/**************************************************
 *       Values For power off or on a sensor      *
 **************************************************/ 
typedef enum Sensor
{  
	SENSOR_ENABLE = 0  ,                                       							
	SENSOR_DISABLE  = 7                                     
}ICM20649_Sensor ;
/*******************************************************
 *               Values For INT_LEVEL                  *
 *******************************************************/ 
typedef enum int_level
{  
	ACTIVE_HIGH = 0     ,                      
	ACTIVE_LOW     
} ICM20649_INT_Level; 
/*******************************************************
 *                Values For INT_OPEN                  *
 *******************************************************/ 
typedef enum int_type
{  
	PUSH_PULL = 0     ,                      
	OPEN_DRAIN     
}ICM20649_INT_Type; 
/*******************************************************
 *                 Values For LATCH_INT                *
 *******************************************************/ 
typedef enum latch_type
{  
	_50_US = 0     ,                      
	HELD_STATUS_CLEAR     
} ICM20649_Latch_Type; 
/*******************************************************
 *  Values For LATCH_INT_EN in INT_PIN_CFG Register    *
 *******************************************************/ 
typedef enum FIFO_Reset
{  
	FIFO_DE_ASSERT = 0     ,                               /* S/W FIFO reset. Assert and hold to set FIFO size to 0.  */     
	FIFO_ASSERT = 31                                       /* Assert and de-assert to reset fifo */
} ICM20649_FIFO_Reset; 
/*******************************************************
 *  Values For FIFO_OFLOW_INT in INT_STATUS_2 Register *
 *******************************************************/
typedef enum FIFO_Overflow
{  
	NOT_FIFO_OVERFLOW = 0     ,                      
	FIFO_OVERFLOW = 1     
} ICM20649_FIFO_Overflow; 
/*******************************************************
 *  Values For FIFO_OFLOW_INT in INT_STATUS Register   *
 *******************************************************/
typedef enum Data_Copy_FIFO
{  
	NOT_COPY_FIFO = 0     ,                               /* Data from sensors is copied to FIFO or SRAM. Set when sequence controller kicks off on a sensor data load. Only bit 0 is relevant in a single FIFO configuration. Cleared on read */
	COPY_TO_FIFO = 3     
} ICM20649_Data_Copy_FIFO; 
/*******************************************************
 *           Values For BYPASS or Enable DLPF          *
 *******************************************************/
typedef enum FCHOICEB
{  
	BYPASS_DLPF_FCHOICEB_0 = 0     ,                      
	ENABLE_DLPF_FCHOICEB_1                                /* 1 Used to bypass DLPF           */
}ICM20649_FCHOICEB;
/*******************************************************
 * Values For GYRO_DLPFCFG in  GYRO_CONFIG_1 Register  *
 *******************************************************/
typedef enum Gyro_DLPF_CFG
{
	ICM20649_GYRO_DLPF_196    = 0, 											/*!< GYRO 3-dB BW(Hz) = 196.6,  */
	ICM20649_GYRO_DLPF_152	  = 1,								 		  /*!< GYRO 3-dB BW(Hz) = 151.8   */
	ICM20649_GYRO_DLPF_119		= 2, 											/*!< GYRO 3-dB BW(Hz) = 119.5   */
	ICM20649_GYRO_DLPF_51	  	= 3, 											/*!< GYRO 3-dB BW(Hz) = 51.2    */
	ICM20649_GYRO_DLPF_24	  	= 4, 											/*!< GYRO 3-dB BW(Hz) = 23.9,   */
	ICM20649_GYRO_DLPF_12			= 5, 											/*!< GYRO 3-dB BW(Hz) = 11.6    */
	ICM20649_GYRO_DLPF_6		  = 6, 											/*!< GYRO 3-dB BW(Hz) = 5.7     */
	ICM20649_GYRO_DLPF_361		= 7 											/*!< GYRO 3-dB BW(Hz) = 361.4   */
}ICM20649_GYRO_DLPF ;
/*******************************************************
 *   Values For A_DLPF_CFG in ACCEL_CONFIG2 Register   *  
 *******************************************************/
typedef enum Accel_DLPF_CFG
{
	ICM20649_ACCEL_DLPF_246	    = 1 ,								  /*!< accel 3-dB BW(Hz) = 246      */
	ICM20649_ACCEL_DLPF_111	  	= 2, 									/*!< accel 3-dB BW(Hz) = 111.4    */
	ICM20649_ACCEL_DLPF_50	  	= 3, 									/*!< accel 3-dB BW(Hz) = 50.4     */
	ICM20649_ACCEL_DLPF_24	  	= 4, 									/*!< accel 3-dB BW(Hz) = 23.9     */
	ICM20649_ACCEL_DLPF_11			= 5, 									/*!< accel 3-dB BW(Hz) = 11.5     */
	ICM20649_ACCEL_DLPF_6		    = 6, 									/*!< accel 3-dB BW(Hz) = 5.7      */
	ICM20649_ACCEL_DLPF_473 	  = 7 									/*!< accel 3-dB BW(Hz) = 473      */
}ICM20649_ACCEL_DLPF ;
/*******************************************************
 *   Values For A_DLPF_CFG in ACCEL_CONFIG2 Register   *
 *******************************************************/
typedef enum Temp_DLPF_CFG
{
	ICM20649_TEMP_DLPF_7932    = 0, 									/*!< TEMP 3-dB BW(Hz) = 7932      */
	ICM20649_TEMP_DLPF_218     = 1,								    /*!< TEMP 3-dB BW(Hz) = 217.9     */
	ICM20649_TEMP_DLPF_123		 = 2, 									/*!< TEMP 3-dB BW(Hz) = 123.5     */
	ICM20649_TEMP_DLPF_66	   	 = 3, 									/*!< TEMP 3-dB BW(Hz) = 65.9      */
	ICM20649_TEMP_DLPF_34	  	 = 4, 									/*!< TEMP 3-dB BW(Hz) = 34.1      */
	ICM20649_TEMP_DLPF_17			 = 5, 									/*!< TEMP 3-dB BW(Hz) = 17.3      */
	ICM20649_TEMP_DLPF_9	     = 6, 									/*!< TEMP 3-dB BW(Hz) = 8.8       */
}ICM20649_TEMP_DLPF ;


 /*************************************************
 *  Defining ICM20649 Register & Data As Struct   *
 **************************************************/
typedef	struct ICM20649
{
	  uint8_t                       		Register_Cache1;
		uint8_t                       		Register_Cache2;
	  ICM20649_Get_DATA             		GET_DATA;
		ICM20649_Bank_Sel             		Bank_Sel;
	  ICM20649_Reset_Status         		RESET;
	  uint8_t                       		WHO_AM_I;
	  ICM20649_Interface            		INTERFACE;
	  ICM20649_Sleep 							  		IS_ICM20649_SLEEP;
	  ICM20649_CLK     									CLOCK_SOURCE;
	  ICM20649_Ability              		DMP;
	  ICM20649_Ability                  TEMPERATURE;
		ICM20649_TEMP_DLPF  					    TEMP_DLPF;
	  ICM20649_Sensor                   GYRO;
	  ICM20649_Gyro_Fs_Sel				      GYRO_FULL_SCALE;
		ICM20649_Gyro_Scale_Factor        GYRO_SCALE_FACTOR;
		float            							    PRECISE_GYRO_SF;
	  ICM20649_Power_Mode 				  	  GYRO_POWER_MODE;
		ICM20649_FCHOICEB              		GYRO_FCHOICEB;
    ICM20649_GYRO_DLPF  					    GYRO_DLPF;		
	  ICM20649_GYRO_Averaging_Filter    GYRO_AVERAGING_FILTER;
		uint16_t                 		     	GYRO_SAMPLE_RATE;
	  uint8_t                       		GYRO_SAMPLE_DEVIDE;
	  ICM20649_Sensor                   ACCEL;
		ICM20649_Accel_Fs_Sel				      ACCEL_FULL_SCALE;
		ICM20649_Accel_Scale_Factor       ACCEL_SCALE_FACTOR;
		ICM20649_Power_Mode 				  	  ACCEL_POWER_MODE;
		ICM20649_FCHOICEB                 ACCEL_FCHOICEB; 
		ICM20649_ACCEL_DLPF  					    ACCEL_DLPF;
		ICM20649_ACCEL_Averaging_Filter   ACCEL_AVERAGING_FILTER;	
		uint16_t                      		ACCEL_SAMPLE_RATE;
	  uint16_t                      		ACCEL_SAMPLE_DEVIDE;
	  ICM20649_Ability              		INTERRUPT;	  
		ICM20649_INT_Level                INT1_PIN_LEVEL;
  	ICM20649_INT_Type                 INT1_PIN_TYPE;
	  ICM20649_Latch_Type               INT1_PIN_LATCH;
		ICM20649_INT_Level                INT2_PIN_LEVEL;
  	ICM20649_INT_Type                 INT2_PIN_TYPE;
	  ICM20649_Latch_Type               INT2_PIN_LATCH;			
		ICM20649_Preparation            	DATA_STATUS;
	  ICM20649_FIFO_Ability        			FIFO;
		ICM20649_FIFO_Reset  							FIFO_RESET;
		ICM20649_Ability              		INTERFACE_ACCESS_FIFO;
		ICM20649_Ability              		TEMP_TO_FIFO;
		ICM20649_Ability									GYRO_TO_FIFO;
		ICM20649_Ability              		ACCEL_TO_FIFO;
		ICM20649_FIFO_Mode					  		FIFO_MODE;
		uint16_t                      		FIFO_PACKET_QTY;
		ICM20649_Data_Copy_FIFO       		DATA_COPY_FIFO;
		ICM20649_FIFO_Overflow          	FIFO_OVERFLOW;
		uint16_t                      		FIFO_COUNT ;
  	int16_t 													REGISTER_RAW_TEMP;
		int16_t 													REGISTER_RAW_GYRO_X;
		int16_t 													REGISTER_RAW_GYRO_Y;
		int16_t 													REGISTER_RAW_GYRO_Z;
		int16_t 													REGISTER_RAW_ACCEL_X;
		int16_t 													REGISTER_RAW_ACCEL_Y;
		int16_t														REGISTER_RAW_ACCEL_Z;
  	float 														VALID_TEMP_DATA;
		float 														VALID_ACCEL_DATA_X;
		float 														VALID_ACCEL_DATA_Y;
		float 														VALID_ACCEL_DATA_Z;
		float 														VALID_GYRO_DATA_X;
		float 														VALID_GYRO_DATA_Y;
		float 														VALID_GYRO_DATA_Z;
		uint8_t 													FIFO_DATA[FIFO_DATA_BUFFER_SIZE];
		float														  VALID_FIFO_ACCEL_X[PACKET_QTY_IN_FULL_FIFO];
		float														  VALID_FIFO_ACCEL_Y[PACKET_QTY_IN_FULL_FIFO];
		float 														VALID_FIFO_ACCEL_Z[PACKET_QTY_IN_FULL_FIFO];
		float														  VALID_FIFO_GYRO_X[PACKET_QTY_IN_FULL_FIFO];
		float														  VALID_FIFO_GYRO_Y[PACKET_QTY_IN_FULL_FIFO];
		float 														VALID_FIFO_GYRO_Z[PACKET_QTY_IN_FULL_FIFO];
		float 														VALID_FIFO_TEMP[PACKET_QTY_IN_FULL_FIFO];
}GebraBit_ICM20649;
/*
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/********************************************************
 *Declare Read&Write ICM20649 Register Values Functions *
 ********************************************************/
extern	uint8_t	GB_ICM20649_Read_Reg_Data ( uint8_t regAddr, ICM20649_Bank_Sel regBank, uint8_t* data);
extern	uint8_t GB_ICM20649_Read_Reg_Bits (uint8_t regAddr, ICM20649_Bank_Sel regBank, uint8_t start_bit, uint8_t len, uint8_t* data);
extern	uint8_t GB_ICM20649_Burst_Read(uint8_t regAddr, ICM20649_Bank_Sel regBank, uint8_t *data, uint16_t byteQuantity);
extern	uint8_t GB_ICM20649_Write_Reg_Data(uint8_t regAddr, ICM20649_Bank_Sel regBank, uint8_t data);
extern	uint8_t	GB_ICM20649_Write_Reg_Bits(uint8_t regAddr, ICM20649_Bank_Sel regBank, uint8_t start_bit, uint8_t len, uint8_t data);
extern	uint8_t GB_ICM20649_Burst_Write		( uint8_t regAddr, ICM20649_Bank_Sel regBank, uint8_t *data, 	uint16_t byteQuantity);
/********************************************************
 *       Declare ICM20649 Configuration Functions       *
 ********************************************************/
extern void GB_ICM20649_Soft_Reset ( GebraBit_ICM20649 * ICM20649 );
extern void GB_ICM20649_Bank_Selection( ICM20649_Bank_Sel bsel);
extern void GB_ICM20649_Who_am_I(GebraBit_ICM20649 * ICM20649);
extern void GB_ICM20649_Select_SPI4_Interface(GebraBit_ICM20649 * ICM20649 , ICM20649_Interface spisel);
extern void GB_ICM20649_DMP(GebraBit_ICM20649* ICM20649 ,ICM20649_Ability dmp,ICM20649_DMP_LP dmp_lp);
extern void GB_ICM20649_DMP_Reset(GebraBit_ICM20649* ICM20649 ,ICM20649_Ability rst);
extern void GB_ICM20649_DMP_Interrupt(ICM20649_Interrupt_Pin pin,ICM20649_Ability interrupt);
extern void GB_ICM20649_Sleep_Awake (GebraBit_ICM20649 * ICM20649, ICM20649_Sleep  working  ) ;
extern void GB_ICM20649_ACCEL_Power_Mode(GebraBit_ICM20649* ICM20649 ,ICM20649_Power_Mode pmode);
extern void GB_ICM20649_GYRO_Power_Mode(GebraBit_ICM20649* ICM20649 ,ICM20649_Power_Mode pmode);
extern void GB_ICM20649_Set_Clock_Source(GebraBit_ICM20649 * ICM20649 , ICM20649_CLK clk) ;
extern void GB_ICM20649_Temperature(GebraBit_ICM20649* ICM20649 ,ICM20649_Ability temp);
extern void GB_ICM20649_Accelerometer(GebraBit_ICM20649 * ICM20649 , ICM20649_Sensor accel);
extern void GB_ICM20649_Gyroscope(GebraBit_ICM20649 * ICM20649 , ICM20649_Sensor gyro) ;
extern void GB_ICM20649_Set_INT1_Pin(GebraBit_ICM20649 * ICM20649 , ICM20649_INT_Level level ,ICM20649_INT_Type type , ICM20649_Latch_Type latch );
extern void GB_ICM20649_Set_INT2_Pin(GebraBit_ICM20649 * ICM20649 , ICM20649_INT_Level level ,ICM20649_INT_Type type , ICM20649_Latch_Type latch );
extern void Interrupt_Status_Enable(GebraBit_ICM20649 * ICM20649 , ICM20649_Ability interrupt );
extern ICM20649_Preparation GB_ICM20649_Check_Data_Preparation(GebraBit_ICM20649 * ICM20649);
extern void GB_ICM20649_GYRO_Full_Scale ( GebraBit_ICM20649 * ICM20649 , ICM20649_Gyro_Fs_Sel fs ) ;
extern void GB_ICM20649_GYRO_Low_Pass_Filter  (GebraBit_ICM20649 * ICM20649 ,  ICM20649_FCHOICEB bypass ) ;
extern void GB_ICM20649_GYRO_Low_Pass_Filter_Value  (GebraBit_ICM20649 * ICM20649 , ICM20649_GYRO_DLPF dlpf );
extern void GB_ICM20649_GYRO_LP_Averaging_Filter  (GebraBit_ICM20649 * ICM20649 , ICM20649_GYRO_Averaging_Filter avg );
extern void GB_ICM20649_GYRO_Output_Sample_Rate (GebraBit_ICM20649 * ICM20649 , uint16_t rate_hz);
extern void GB_ICM20649_ACCEL_Full_Scale ( GebraBit_ICM20649 * ICM20649 , ICM20649_Accel_Fs_Sel fs );
extern void GB_ICM20649_ACCEL_Low_Pass_Filter  (GebraBit_ICM20649 * ICM20649 ,  ICM20649_FCHOICEB bypass );
extern void GB_ICM20649_ACCEL_Low_Pass_Filter_Value  (GebraBit_ICM20649 * ICM20649 , ICM20649_ACCEL_DLPF dlpf );
extern void GB_ICM20649_ACCEL_LP_Averaging_Filter  (GebraBit_ICM20649 * ICM20649 , ICM20649_ACCEL_Averaging_Filter avg );
extern void GB_ICM20649_ACCEL_Output_Sample_Rate (GebraBit_ICM20649 * ICM20649 , uint16_t rate_hz);
extern void GB_ICM20649_TEMP_Low_Pass_Filter_Value  (GebraBit_ICM20649 * ICM20649 , ICM20649_TEMP_DLPF tdlpf );
/********************************************************
 *          Declare ICM20649 FIFO Functions             *
 ********************************************************/
extern void GB_ICM20649_Access_Serial_Interface_To_FIFO(GebraBit_ICM20649 * ICM20649 , ICM20649_Ability interface_access_fifo);
extern ICM20649_FIFO_Overflow GB_ICM20649_Check_FIFO_Overflow(GebraBit_ICM20649 * ICM20649) ;
extern void GB_ICM20649_Write_ACCEL_FIFO(GebraBit_ICM20649 * ICM20649 , ICM20649_Ability accel_fifo ) ;
extern void GB_ICM20649_Write_GYRO_FIFO(GebraBit_ICM20649 * ICM20649 , ICM20649_Ability gyro_fifo ) ;
extern void GB_ICM20649_Write_TEMP_FIFO(GebraBit_ICM20649 * ICM20649 , ICM20649_Ability temp_fifo );
extern void GB_ICM20649_FIFO_Mode(GebraBit_ICM20649 * ICM20649 , ICM20649_FIFO_Mode fifo_mode );
extern void GB_ICM20649_FIFO_Reset(void) ;
extern void GB_ICM20649_GET_FIFO_Count (GebraBit_ICM20649 * ICM20649 ) ;
extern void GB_ICM20649_Read_FIFO(GebraBit_ICM20649 * ICM20649 , uint16_t qty);
extern ICM20649_Data_Copy_FIFO GB_ICM20649_Check_Data_Copy_TO_FIFO(GebraBit_ICM20649 * ICM20649) ;
/********************************************************
 *          Declare ICM20649 DATA Functions             *
 ********************************************************/
extern void GB_ICM20649_Get_Temp_Register_Raw_Data(GebraBit_ICM20649 * ICM20649);
extern void GB_ICM20649_Get_Temp_Valid_Data(GebraBit_ICM20649 * ICM20649);
extern void GB_ICM20649_Get_GYRO_X_Register_Raw_DATA(GebraBit_ICM20649 * ICM20649);
extern void GB_ICM20649_Get_GYRO_Y_Register_Raw_DATA(GebraBit_ICM20649 * ICM20649);
extern void GB_ICM20649_Get_GYRO_Z_Register_Raw_DATA(GebraBit_ICM20649 * ICM20649);
extern void GB_ICM20649_Get_GYRO_DATA_X_Valid_Data(GebraBit_ICM20649 * ICM20649);
extern void GB_ICM20649_Get_GYRO_DATA_Y_Valid_Data(GebraBit_ICM20649 * ICM20649);
extern void GB_ICM20649_Get_GYRO_DATA_Z_Valid_Data(GebraBit_ICM20649 * ICM20649);
extern void GB_ICM20649_Get_ACCEL_X_Register_Raw_DATA(GebraBit_ICM20649 * ICM20649);
extern void GB_ICM20649_Get_ACCEL_Y_Register_Raw_DATA(GebraBit_ICM20649 * ICM20649);
extern void GB_ICM20649_Get_ACCEL_Z_Register_Raw_DATA(GebraBit_ICM20649 * ICM20649);
extern void GB_ICM20649_Get_ACCEL_DATA_X_Valid_Data(GebraBit_ICM20649 * ICM20649);
extern void GB_ICM20649_Get_ACCEL_DATA_Y_Valid_Data(GebraBit_ICM20649 * ICM20649);
extern void GB_ICM20649_Get_ACCEL_DATA_Z_Valid_Data(GebraBit_ICM20649 * ICM20649);
extern void GB_ICM20649_Get_Temperature(GebraBit_ICM20649 * ICM20649);
extern void GB_ICM20649_Get_XYZ_GYROSCOPE(GebraBit_ICM20649 * ICM20649);
extern void GB_ICM20649_Get_XYZ_ACCELERATION(GebraBit_ICM20649 * ICM20649);
extern void GB_ICM20649_Get_ACCEL_GYRO_TEMP_From_Registers(GebraBit_ICM20649 * ICM20649);
extern void GB_ICM20649_FIFO_Data_Partition_ACCEL_GYRO_XYZ_TEMP(GebraBit_ICM20649 * ICM20649) ;
extern void GB_ICM20649_Get_ACCEL_GYRO_TEMP_From_FIFO(GebraBit_ICM20649 * ICM20649)  ;
extern void GB_ICM20649_Get_Data(GebraBit_ICM20649 * ICM20649 , ICM20649_Get_DATA get_data)  ;
/********************************************************
 *          Declare ICM20649 HIGH LEVEL Functions       *
 ********************************************************/
extern void GB_ICM20649_FIFO_Configuration ( GebraBit_ICM20649 * ICM20649 , ICM20649_FIFO_Ability fifo );
extern void GB_ICM20649_Set_Power_Management(GebraBit_ICM20649 * ICM20649 , ICM20649_Power_Mode pmode) ;
extern void GB_ICM20649_initialize( GebraBit_ICM20649 * ICM20649 );
extern void GB_ICM20649_Configuration(GebraBit_ICM20649 * ICM20649, ICM20649_FIFO_Ability fifo);

#endif
