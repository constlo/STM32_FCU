#include "MMA8452Q.h"

//Change this to suit your stm32 model.
#include "../Inc/stm32f3xx_hal.h"

//Change this to suit your design.
extern I2C_HandleTypeDef hi2c1;
void SingleByteRead(uint8_t reg, uint8_t *data)
{
	HAL_I2C_Mem_Read(&hi2c1, READ_REGISTER_SA0_HIGH, reg, 1, data, 1, HAL_MAX_DELAY);
}
void SingleByteWrite(uint8_t reg, uint8_t *data)
{
	HAL_I2C_Mem_Write(&hi2c1, WRITE_REGISTER_SA0_HIGH, reg, 1, data, 1, HAL_MAX_DELAY);
}
void MultiByteRead(uint8_t reg, uint8_t *data, uint8_t size)
{
	HAL_I2C_Mem_Read(&hi2c1, READ_REGISTER_SA0_HIGH, reg, 1, data, size, HAL_MAX_DELAY);
}
void MultiByteWrite(uint8_t reg, uint8_t *data, uint8_t size)
{
	HAL_I2C_Mem_Write(&hi2c1, WRITE_REGISTER_SA0_HIGH, reg, 1, data, size, HAL_MAX_DELAY);
}

// Set the Active mode
//  @returns the result of the write operation (0 on success) [uint8_t]
uint8_t  MMA_active(void){
	uint8_t data;
	SingleByteRead(CTRL_REG1, &data);
	data  |= 0x01;
	SingleByteWrite(CTRL_REG1, &data );
	return 0;
}

// --------------------------------------------------

// Check if new data is available
//  @returns 1 if a new set of data is avaible, 0 otherwise [uint8_t]
uint8_t  MMA_available(void){
	uint8_t data;
	SingleByteRead(STATUS, &data);
	data &= 0x08;
	return (data >> 3);
}

// --------------------------------------------------

// Get the Read Mode
//  @returns 1 if the mode is set to fast read, 0 otherwise [uint8_t]
uint8_t  MMA_getFastRead(void){
	uint8_t data;
	SingleByteRead(CTRL_REG1, &data);
	data &= 0x02;
	data >>= 1;
	return data;
}

// --------------------------------------------------

// Check if the High-Pass Output is enabled
//  @returns 1 if the high-pass filter is on, 0 otherwise [uint8_t]
uint8_t  MMA_getHighPassOutput(void){
	uint8_t data;
	SingleByteRead(XYZ_DATA_CFG, &data);
	data &= 0x10;
	return (data >> 4);
}

// --------------------------------------------------

// Get the current Oversampling Mode
//  @param (sleep_mode) : true for sleep mode [bool]
//  @returns the oversampling mode [uint8_t]
uint8_t  MMA_getMode(uint8_t sleepmode){
	uint8_t data;
	SingleByteRead(CTRL_REG2, &data);
	if(sleepmode){
	data &= 0x18;
	return (data >> 3);
	} else {
	return (data & 0x03);
	}
}

// --------------------------------------------------

// Get the current Output Data Rate
//  @returns the ODR [uint8_t]
uint8_t  MMA_getODR(void){
	uint8_t data;
	SingleByteRead(CTRL_REG1, &data);
	data &= 0x38;
	return (data >> 3);
}

// --------------------------------------------------

// Get the current Full Scale Range
//  @returns the scale [uint8_t]
uint8_t  MMA_getScale(void){
	uint8_t data;
	SingleByteRead(XYZ_DATA_CFG, &data);
	data &= 0x03;
	return data;
}

// --------------------------------------------------

// Get the current System Mode
//  @returns the system mode [MMA8452Q_SYSMOD]
uint8_t MMA_getState(void){
	uint8_t data;
	SingleByteRead(SYSMOD, &data);
	return data;
}

// --------------------------------------------------

// Initialize the module
//  @param (scale) : the scale to configure [MMA8452Q_Scale]
//         (odr) : the output data rate to configure [MMA8452Q_ODR]
//  @returns 0 on invalid signature, 1 otherwise [uint8_t]
uint8_t MMA_init(uint8_t scale, uint8_t odr){
	uint8_t mydata;
	// check the device identification (should always be 0x2A)
	SingleByteRead(WHO_AM_I, &mydata);
	if (mydata != 0x2A) {
	return 0;
	}

	MMA_standby(); // must be in standby to change registers

	MMA_setScale(scale); // set up accelerometer scale
	MMA_setODR(odr); // set up output data rate

	MMA_active(); // set to active to start reading
	return 1;
}

// --------------------------------------------------

// Read the acceleration data
void  MMA_read(int16_t *x, int16_t *y, int16_t *z){
	uint8_t data;
	float factor = 1.0 ;

	if(MMA_getFastRead() == 0){
	factor *= 2048.0; // 12 bits resolution

	uint8_t rawData[6]; // x/y/z acceleration register data stored here
	// read the six raw data registers into data array
	MultiByteRead(OUT_X_MSB, rawData, 6);
	*x = ((int16_t)((rawData[0] << 8) | rawData[1])) >> 4;
	*y = ((int16_t)((rawData[2] << 8) | rawData[3])) >> 4;
	*z = ((int16_t)((rawData[4] << 8) | rawData[5])) >> 4;

	} else {
	factor *= 128.0; // 8 bits resolution

	uint8_t rawData[3]; // x/y/z acceleration register data stored here
	// read the three raw data registers into data array
	MultiByteRead(OUT_X_MSB, rawData, 3);
	// careful with negative values in 8-bit mode (shift operations handle the signal bit)
	*x = (int16_t)(rawData[0]<<8)>>8;
	*y = (int16_t)(rawData[1]<<8)>>8;
	*z = (int16_t)(rawData[2]<<8)>>8;
	}
}

// --------------------------------------------------

// Set the Fast Read mode
//  @param (fs) : the fast read mode to set [MMA8452Q_FastRead]
//  @returns 0xFF if not in standby or the result of the write operation (0 on success) [uint8_t]
void MMA_setFastRead(uint8_t fastRead){
	uint8_t data;
	// check if in standby mode
	if(MMA_getState() != SYSMOD_STANDBY){
		return 0xFF;
	}

	SingleByteRead(WHO_AM_I, &data);
	if(fastRead){
		data |= 0x02;
	} else {
		data &= 0xFD;
	}
	SingleByteWrite(CTRL_REG1, &data);

	return 1;
}

// --------------------------------------------------

// Set the High-Pass Output
//  @param (set) : true to enable the high-pass filter [bool]
//  @returns 0xFF if not in standby or the result of the write operation (0 on success) [uint8_t]
void  MMA_setHighPassOutput(uint8_t set){
	uint8_t data;
	// check if in standby mode
	if(MMA_getState() != SYSMOD_STANDBY){
	return 0xFF;
	}

	// check if sleep mode
	uint8_t cfg;
	SingleByteRead(XYZ_DATA_CFG, &cfg);
	if(set){
	cfg |= 0x10;
	} else {
	cfg &= 0xEF;
	}
	SingleByteWrite(XYZ_DATA_CFG, &cfg);
}

// --------------------------------------------------

// Set the Oversampling Mode
//  @param (mode) : the oversampling mode to set [MMA8452Q_Mode]
//         (sleep) : true to select the sleep mode [bool]
//  @returns 0xFF if not in standby or the result of the write operation (0 on success) [uint8_t]
void  MMA_setMode(uint8_t mode, uint8_t sleep){
	uint8_t data;
	// check if in standby mode
	if(MMA_getState() != SYSMOD_STANDBY){
		return 0xFF;
	}

	// check if sleep mode
	uint8_t toset = mode;
	SingleByteRead(CTRL_REG2, &data);
	if(sleep){
		data &= 0xE7; // mask out SMOD bits
		data |= (toset << 3);
	} else {
		data &= 0xFC; // mask out MOD bits
		data |= toset;
	}
	SingleByteWrite(CTRL_REG2, &data);
}

// --------------------------------------------------

// Set the Outuput Data Rate
//  @param (odr) : the oversampling mode to set [MMA8452Q_ODR]
//         (sleep) : true to select the sleep mode [bool]
//  @returns 0xFF if not in standby or the result of the write operation (0 on success) [uint8_t]
void  MMA_setODR(uint8_t odr){
	uint8_t data;
	// check if in standby mode
	if(MMA_getState() != SYSMOD_STANDBY){

	}

	// check if sleep mode
	uint8_t toset = odr;
	SingleByteRead(CTRL_REG1, &data);
	data &= 0xC7; // mask out data rate bits
	data |= (toset << 3);
	SingleByteWrite(CTRL_REG2, &data);
}

// --------------------------------------------------

// Set the Full Scale Range
//  @param (fsr) : the full scale range to set [MMA8452Q_Scale]
//  @returns 0xFF if not in standby or the result of the write operation (0 on success) [uint8_t]
void  MMA_setScale(uint8_t scale){
	uint8_t data;
  // check if in standby mode
	if(MMA_getState() != SYSMOD_STANDBY){
	return 0xFF;
	}

	SingleByteRead(XYZ_DATA_CFG, &data);
	data &= 0xFC; // mask out scale bits
	data |= (scale >> 2);  // neat trick, see page 22 : 00 = 2G, 01 = 4A, 10 = 8G
	SingleByteWrite(XYZ_DATA_CFG, &data);
}

// --------------------------------------------------

// Set the Standby mode
//  @returns the result of the write operation (0 on success) [uint8_t]
void  MMA_standby(void){
	uint8_t data;
	SingleByteRead(CTRL_REG1, &data);
	data &= 0xFE;
	SingleByteWrite(CTRL_REG1, &data);
}
