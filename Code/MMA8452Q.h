//This library is a derivative work from https://github.com/RoboCore/RoboCore_MMA8452Q/tree/master
//and is under the LGPL license.
//This port is made by Constlo, with a few changes to the original library.
//My github is https://github.com/constlo
//There are a few changes that need to be made in order for this to fully work, but right now it supports my need.
//I might add full functionality in the future if needed.

#ifndef MMA8452Q_H
#define MMA8452Q_H

#include "stm32f3xx_hal.h"

//Register values for the MMA8452Q
#define	STATUS             0x00
#define	OUT_X_MSB          0x01
#define	OUT_X_LSB           0x02
#define	OUT_Y_MSB           0x03
#define	OUT_Y_LSB           0x04
#define	OUT_Z_MSB           0x05
#define	OUT_Z_LSB           0x06
#define	SYSMOD              0x0B
#define	INT_SOURCE          0x0C
#define	WHO_AM_I            0x0D
#define	XYZ_DATA_CFG        0x0E
#define	HP_FILTER_CUTOFF    0x0F
#define	PL_STATUS           0x10
#define	PL_CFG              0x11
#define	PL_COUNT            0x12
#define	PL_BF_ZCOMP         0x13
#define	P_L_THS_REG         0x14
#define	FF_MT_CFG           0x15
#define	FF_MT_SRC           0x16
#define	FF_MT_THS           0x17
#define	FF_MT_COUNT         0x18
#define	TRANSIENT_CFG       0x1D
#define	TRANSIENT_SRC       0x1E
#define	TRANSIENT_THS       0x1F
#define	TRANSIENT_COUNT     0x20
#define	PULSE_CFG           0x21
#define	PULSE_SRC           0x22
#define	PULSE_THSX          0x23
#define	PULSE_THSY          0x24
#define	PULSE_THSZ          0x25
#define	PULSE_TMLT          0x26
#define	PULSE_LTCY          0x27
#define	PULSE_WIND          0x28
#define	ASLP_COUNT          0x29
#define	CTRL_REG1           0x2A
#define	CTRL_REG2           0x2B
#define	CTRL_REG3           0x2C
#define	CTRL_REG4           0x2D
#define	CTRL_REG5           0x2E
#define	OFF_X               0x2F
#define	OFF_Y               0x30
#define	OFF_Z               0x31

#define FAST_READ_OFF  		0
#define FAST_READ_ON  		1
#define MODE_NORMAL			0x00
#define MODE_LNLP 			0x01
#define MODE_HIGH_RES 		0x02
#define MODE_LOW_POWER  	0x03
#define SYSMOD_STANDBY		0x00
#define SYSMOD_WAKE 		0x01
#define SYSMOD_SLEEP 		0x02
#define SCALE_2G 			2
#define SCALE_4G 			4
#define SCALE_8G 			8

#define ODR_800 			0
#define ODR_400 			1
#define ODR_200 			2
#define ODR_100 			3
#define ODR_50 				4
#define ODR_12 				5
#define ODR_6 				6
#define ODR_1 				7

//I2C Read and write commands.
//The MMA8452Q has a differing slave address depending on if the SA0 pin is tied low or high.
//When the SA0 pin is tied low, the read/write addresses are 0x39/0x38.
//When the SA0 pin is tied high, the read/write addresses are 0x3B/0x3A.

#define READ_REGISTER_SA0_LOW 	0x39
#define WRITE_REGISTER_SA0_LOW 	0x38
#define READ_REGISTER_SA0_HIGH 	0x3B
#define WRITE_REGISTER_SA0_HIGH	0x3A

void SingleByteRead(uint8_t reg, uint8_t *data);
void SingleByteWrite(uint8_t reg, uint8_t *data);
void MultiByteRead(uint8_t reg, uint8_t *data, uint8_t size);
void MultiByteWrite(uint8_t reg, uint8_t *data, uint8_t size);

uint8_t MMA_available(void);
uint8_t MMA_active(void);
uint8_t MMA_getFastRead(void);
uint8_t MMA_getHighPassOutput(void);
uint8_t MMA_getMode(uint8_t sleepmode);
uint8_t MMA_getODR(void);
uint8_t MMA_getScale(void);
uint8_t MMA_getState();
uint8_t MMA_init(uint8_t scale, uint8_t odr);
void MMA_read(int16_t *x, int16_t *y, int16_t *z);
void MMA_setFastRead(uint8_t fastRead);
void MMA_setHighPassOutput(uint8_t set);
void MMA_setMode(uint8_t mode, uint8_t sleep);
void MMA_setODR(uint8_t odr);
void MMA_setScale(uint8_t scale);
void MMA_standby(void);

#endif  // MMA8452Q_H
