// This file is part of MatrixPilot.
//
//    http://code.google.com/p/gentlenav/
//
// Copyright 2009-2012 MatrixPilot Team
// See the AUTHORS.TXT file for a list of authors of MatrixPilot.
//
// MatrixPilot is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// MatrixPilot is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with MatrixPilot.  If not, see <http://www.gnu.org/licenses/>.

// Internal MPU6000 axis definition
// X axis pointing to right, Y axis pointing forward and Z axis pointing up


#include "spi.h"
#include "gpio.h"
#include "dma.h"
//Sensor variables
//NOTE: has to be 8bit not 16, so 16 data position
uint8_t mpu_data[16], mpuCnt = 0;


extern SPI_HandleTypeDef hspi2;
//DMA_HandleTypeDef hdma_spi2_rx;
//DMA_HandleTypeDef hdma_spi2_tx;

/*************************************************************************************/
// This will be mpu6000.c

void MPU6000_init16(void)
{
//	callback = fptr;

// MPU-6000 maximum SPI clock is specified as 1 MHz for all registers
//    however the datasheet states that the sensor and interrupt registers
//    may be read using an SPI clock of 20 Mhz

// As these register accesses are one time only during initial setup lets be
//    conservative and only run the SPI bus at half the maximum specified speed

	HAL_StatusTypeDef err;
	// need at least 60 msec delay here
	HAL_Delay(60);
	err = writeMPUSPIreg16(MPUREG_PWR_MGMT_1, BIT_H_RESET);
	if(err != HAL_OK){
		//TODO: Do something with posible error here
		while(1);
	}
	// 10msec delay seems to be needed for AUAV3 (MW's prototype)
	HAL_Delay(10);
	// Wake up device and select GyroZ clock (better performance)
	err = writeMPUSPIreg16(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
	if(err != HAL_OK){
		//TODO: Do something with posible error here
		while(1);
	}
//	func_SPI_Write_Byte((MPUREG_PWR_MGMT_1|0x80),0x00, data);
	// Disable I2C bus (recommended on datasheet)
	err = writeMPUSPIreg16(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);
	if(err != HAL_OK){
		//TODO: Do something with posible error here
		while(1);
	}
	// SAMPLE RATE
	err = writeMPUSPIreg16(MPUREG_SMPLRT_DIV, 4); // Sample rate = 200Hz  Fsample= 1Khz/(N+1) = 200Hz
	if(err != HAL_OK){
		//TODO: Do something with posible error here
		while(1);
	}
	// scaling & DLPF
	err = writeMPUSPIreg16(MPUREG_CONFIG, BITS_DLPF_CFG_42HZ);
	if(err != HAL_OK){
		//TODO: Do something with posible error here
		while(1);
	}

//	writeMPUSPIreg16(MPUREG_GYRO_CONFIG, BITS_FS_2000DPS);  // Gyro scale 2000º/s
	err = writeMPUSPIreg16(MPUREG_GYRO_CONFIG, BITS_FS_500DPS); // Gyro scale 500º/s
	if(err != HAL_OK){
		//TODO: Do something with posible error here
		while(1);
	}

#if (ACCEL_RANGE == 2)
	err = writeMPUSPIreg16(MPUREG_ACCEL_CONFIG, BITS_FS_2G); // Accel scele +-2g, g = 8192
	if(err != HAL_OK){
		//TODO: Do something with posible error here
		while(1);
	}
#elif (ACCEL_RANGE == 4)
	writeMPUSPIreg16(MPUREG_ACCEL_CONFIG, BITS_FS_4G); // Accel scale g = 4096
#elif (ACCEL_RANGE == 8)
	writeMPUSPIreg16(MPUREG_ACCEL_CONFIG, BITS_FS_8G); // Accel scale g = 2048
#else
#error "Invalid ACCEL_RANGE"
#endif

#if 0
	// Legacy from Mark Whitehorn's testing, we might need it some day.
	// SAMPLE RATE
	writeMPUSPIreg16(MPUREG_SMPLRT_DIV, 7); // Sample rate = 1KHz  Fsample= 8Khz/(N+1)

	// no DLPF, gyro sample rate 8KHz
	writeMPUSPIreg16(MPUREG_CONFIG, BITS_DLPF_CFG_256HZ_NOLPF2);

	writeMPUSPIreg16(MPUREG_GYRO_CONFIG, BITS_FS_500DPS); // Gyro scale 500º/s

//	writeMPUSPIreg16(MPUREG_ACCEL_CONFIG, BITS_FS_2G); // Accel scale 2g, g = 16384
	writeMPUSPIreg16(MPUREG_ACCEL_CONFIG, BITS_FS_4G); // Accel scale g = 8192
//	writeMPUSPIreg16(MPUREG_ACCEL_CONFIG, BITS_FS_8G); // Accel scale g = 4096
#endif

	// INT CFG => Interrupt on Data Ready, totem-pole (push-pull) output INT: Clear on any read
	err = writeMPUSPIreg16(MPUREG_INT_PIN_CFG, BIT_INT_LEVEL | BIT_INT_RD_CLEAR);
	if(err != HAL_OK){
		//TODO: Do something with posible error here
		while(1);
	}
	// INT: Raw data ready
	err = writeMPUSPIreg16(MPUREG_INT_ENABLE, BIT_DATA_RDY_EN);
	if(err != HAL_OK){
		//TODO: Do something with posible error here
		while(1);
	}

// Bump the SPI clock up towards 10.5 MHz for ongoing sensor and interrupt register reads
// 20 MHz is the maximum specified for the MPU-6000
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	HAL_SPI_Init(&hspi2);

	//Enable MPU INT
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}


// Blocking 16 bit write to SPI
HAL_StatusTypeDef writeMPUSPIreg16(uint8_t addr, uint8_t cmd)
{
	HAL_StatusTypeDef err;
	uint16_t i;
	uint8_t dato[2] = {0,0};
//	dato[1] = addr;
//	dato[0] = cmd;
	dato[0] = addr;
	dato[1] = cmd;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	err = HAL_SPI_Transmit(&hspi2, dato, 2, 10);
//	err = HAL_SPI_Transmit(&hspi2, dato, 1, 10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    for(i=0;i<0x0F;i++);
    return err;
}
// SPI module has 8 word FIFOs
// burst read 2n bytes starting at addr;
// Since first byte is address, max of 15 data bytes may be transferred with n=7
void readMPUSPI_burst16n(uint8_t data[], int16_t n, uint16_t addr, void (*call_back)(void))
{
	HAL_StatusTypeDef err;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    //TODO: The problem here is SPI_Receive need address on data[0]. It will call
    //HAL_SPI_TransmitReceive_IT->SPI_TxISR->SPI_TxColseIRQHandler->HAL_SPI_TxCpltCallback
    //wWe should implement HAL_SPI_TxCpltCallback to do what we need to do.
    data[0] = addr;
	err = HAL_SPI_Receive_IT(&hspi2, data, 2*n);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	if(err != HAL_OK){
		//TODO: Do something with posible error here
		while(1);
	}
//	uint16_t i;
//
//	MPU_SS = 0;                 // assert chip select
//	mpu_call_back = call_back;  // store the address of the call back routine
//	SPI_data = &data[0];        // store address of data buffer
//	i = SPIBUF;                 // empty read buffer
//	addr |= 0x80;               // write address-1 in high byte + n-1 dummy words to TX FIFO
//	SPIBUF = addr << 8;         // issue read command
//	for (i = 0; i < n; i++) {
//		SPIBUF = 0;             // queue 'n' null words into the SPI transmit buffer
//	}
//	_SPIIE = 1;                 // turn on SPI interrupts
}

