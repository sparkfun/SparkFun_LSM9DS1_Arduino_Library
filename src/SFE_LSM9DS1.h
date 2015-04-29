/******************************************************************************
SFE_LSM9DS1.h
SFE_LSM9DS1 Library Header File
Jim Lindblom @ SparkFun Electronics
Original Creation Date: February 27, 2015
https://github.com/sparkfun/LSM9DS1_Breakout

This file prototypes the LSM9DS1 class, implemented in SFE_LSM9DS1.cpp. In
addition, it defines every register in the LSM9DS1 (both the Gyro and Accel/
Magnetometer registers).

Development environment specifics:
	IDE: Arduino 1.6.0
	Hardware Platform: Arduino Uno
	LSM9DS1 Breakout Version: 1.0

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/
#ifndef __SFE_LSM9DS1_H__
#define __SFE_LSM9DS1_H__

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include "pins_arduino.h"
#endif

#include "LSM9DS1_Registers.h"
#include "LSM9DS1_Types.h"

#define LSM9DS1_AG_ADDR(sa0)	((sa0) == 0 ? 0x6A : 0x6B)
#define LSM9DS1_M_ADDR(sa1)		((sa1) == 0 ? 0x1C : 0x1E)

enum lsm9ds1_axis {
	X_AXIS,
	Y_AXIS,
	Z_AXIS,
	ALL_AXIS
};

class LSM9DS1
{
public:
	IMUSettings settings;
	
	// We'll store the gyro, accel, and magnetometer readings in a series of
	// public class variables. Each sensor gets three variables -- one for each
	// axis. Call readGyro(), readAccel(), and readMag() first, before using
	// these variables!
	// These values are the RAW signed 16-bit readings from the sensors.
	int16_t gx, gy, gz; // x, y, and z axis readings of the gyroscope
	int16_t ax, ay, az; // x, y, and z axis readings of the accelerometer
	int16_t mx, my, mz; // x, y, and z axis readings of the magnetometer
    int16_t temperature; // Chip temperature
	
	//! TODO: Description
	float abias[3];
	float gbias[3];

	// LSM9DS1 -- LSM9DS1 class constructor
	// The constructor will set up a handful of private variables, and set the
	// communication mode as well.
	// Input:
	//	- interface = Either IMU_MODE_SPI or IMU_MODE_I2C, whichever you're using
	//				to talk to the IC.
	//	- xgAddr = If IMU_MODE_I2C, this is the I2C address of the accel/gyroscope.
	// 				If IMU_MODE_SPI, this is the chip select pin of the gyro (CS_AG)
	//	- mAddr = If IMU_MODE_I2C, this is the I2C address of the magnetometer.
	//				If IMU_MODE_SPI, this is the cs pin of the magnetometer (CS_M)
	LSM9DS1(interface_mode interface, uint8_t xgAddr, uint8_t mAddr);
	LSM9DS1();
	void init(interface_mode interface, uint8_t xgAddr, uint8_t mAddr);
	
	
	// begin() -- Initialize the gyro, accelerometer, and magnetometer.
	// This will set up the scale and output rate of each sensor. The values set
	// in the IMUSettings struct will take effect after calling this function.
	uint16_t begin();
	
	// begin() -- Initialize the gyro, accelerometer, and magnetometer.
	// This will set up the scale and output rate of each sensor. It'll also
	// "turn on" every sensor and every axis of every sensor.
	// Input:
	//	- gScl = The scale of the gyroscope. This should be a gyro_scale value.
	//	- aScl = The scale of the accelerometer. Should be a accel_scale value.
	//	- mScl = The scale of the magnetometer. Should be a mag_scale value.
	//	- gODR = Output data rate of the gyroscope. gyro_odr value.
	//	- aODR = Output data rate of the accelerometer. accel_odr value.
	//	- mODR = Output data rate of the magnetometer. mag_odr value.
	// Output: The function will return an unsigned 16-bit value. The most-sig
	//		bytes of the output are the WHO_AM_I reading of the accel. The
	//		least significant two bytes are the WHO_AM_I reading of the gyro.
	// All parameters have a defaulted value, so you can call just "begin()".
	// Default values are FSR's of:  245DPS, 2g, 2Gs; ODRs of 95 Hz for 
	// gyro, 100 Hz for accelerometer, 100 Hz for magnetometer.
	// Use the return value of this function to verify communication.
	/*uint16_t begin(gyro_scale gScl = G_SCALE_245DPS, 
				accel_scale aScl = A_SCALE_2G, mag_scale mScl = M_SCALE_4GS,
				gyro_odr gODR = G_ODR_952, accel_odr aODR = XL_ODR_50, 
				mag_odr mODR = M_ODR_80);*/
	
	// accelAvailable() -- Polls the accelerometer status register to check
	// if new data is available.
	// Output:	1 - New data available
	//			0 - No new data available
	uint8_t accelAvailable();
	
	// gyroAvailable() -- Polls the gyroscope status register to check
	// if new data is available.
	// Output:	1 - New data available
	//			0 - No new data available
	uint8_t gyroAvailable();
	
	// gyroAvailable() -- Polls the temperature status register to check
	// if new data is available.
	// Output:	1 - New data available
	//			0 - No new data available
	uint8_t tempAvailable();
	
	// magAvailable() -- Polls the accelerometer status register to check
	// if new data is available.
	// Input:
	//	- axis can be either X_AXIS, Y_AXIS, Z_AXIS, to check for new data
	//	  on one specific axis. Or ALL_AXIS (default) to check for new data
	//	  on all axes.
	// Output:	1 - New data available
	//			0 - No new data available
	uint8_t magAvailable(lsm9ds1_axis axis = ALL_AXIS);
	
	// readGyro() -- Read the gyroscope output registers.
	// This function will read all six gyroscope output registers.
	// The readings are stored in the class' gx, gy, and gz variables. Read
	// those _after_ calling readGyro().
	void readGyro();
	
	// int16_t readGyro(axis) -- Read a specific axis of the gyroscope.
	// [axis] can be any of X_AXIS, Y_AXIS, or Z_AXIS.
	// Input:
	//	- axis: can be either X_AXIS, Y_AXIS, or Z_AXIS.
	// Output:
	//	A 16-bit signed integer with sensor data on requested axis.
	int16_t readGyro(lsm9ds1_axis axis);
	
	// readAccel() -- Read the accelerometer output registers.
	// This function will read all six accelerometer output registers.
	// The readings are stored in the class' ax, ay, and az variables. Read
	// those _after_ calling readAccel().
	void readAccel();
	
	// int16_t readAccel(axis) -- Read a specific axis of the accelerometer.
	// [axis] can be any of X_AXIS, Y_AXIS, or Z_AXIS.
	// Input:
	//	- axis: can be either X_AXIS, Y_AXIS, or Z_AXIS.
	// Output:
	//	A 16-bit signed integer with sensor data on requested axis.
	int16_t readAccel(lsm9ds1_axis axis);
	
	// readMag() -- Read the magnetometer output registers.
	// This function will read all six magnetometer output registers.
	// The readings are stored in the class' mx, my, and mz variables. Read
	// those _after_ calling readMag().
	void readMag();
	
	// int16_t readMag(axis) -- Read a specific axis of the magnetometer.
	// [axis] can be any of X_AXIS, Y_AXIS, or Z_AXIS.
	// Input:
	//	- axis: can be either X_AXIS, Y_AXIS, or Z_AXIS.
	// Output:
	//	A 16-bit signed integer with sensor data on requested axis.
	int16_t readMag(lsm9ds1_axis axis);

	// readTemp() -- Read the temperature output register.
	// This function will read two temperature output registers.
	// The combined readings are stored in the class' temperature variables. Read
	// those _after_ calling readTemp().
	void readTemp();
	
	// calcGyro() -- Convert from RAW signed 16-bit value to degrees per second
	// This function reads in a signed 16-bit value and returns the scaled
	// DPS. This function relies on gScale and gRes being correct.
	// Input:
	//	- gyro = A signed 16-bit raw reading from the gyroscope.
	float calcGyro(int16_t gyro);
	
	// calcAccel() -- Convert from RAW signed 16-bit value to gravity (g's).
	// This function reads in a signed 16-bit value and returns the scaled
	// g's. This function relies on aScale and aRes being correct.
	// Input:
	//	- accel = A signed 16-bit raw reading from the accelerometer.
	float calcAccel(int16_t accel);
	
	// calcMag() -- Convert from RAW signed 16-bit value to Gauss (Gs)
	// This function reads in a signed 16-bit value and returns the scaled
	// Gs. This function relies on mScale and mRes being correct.
	// Input:
	//	- mag = A signed 16-bit raw reading from the magnetometer.
	float calcMag(int16_t mag);
	
	// setGyroScale() -- Set the full-scale range of the gyroscope.
	// This function can be called to set the scale of the gyroscope to 
	// 245, 500, or 200 degrees per second.
	// Input:
	// 	- gScl = The desired gyroscope scale. Must be one of three possible
	//		values from the gyro_scale.
	void setGyroScale(uint16_t gScl);
	
	// setAccelScale() -- Set the full-scale range of the accelerometer.
	// This function can be called to set the scale of the accelerometer to
	// 2, 4, 6, 8, or 16 g's.
	// Input:
	// 	- aScl = The desired accelerometer scale. Must be one of five possible
	//		values from the accel_scale.
	void setAccelScale(uint8_t aScl);
	
	// setMagScale() -- Set the full-scale range of the magnetometer.
	// This function can be called to set the scale of the magnetometer to
	// 2, 4, 8, or 12 Gs.
	// Input:
	// 	- mScl = The desired magnetometer scale. Must be one of four possible
	//		values from the mag_scale.
	void setMagScale(uint8_t mScl);
	
	// setGyroODR() -- Set the output data rate and bandwidth of the gyroscope
	// Input:
	//	- gRate = The desired output rate and cutoff frequency of the gyro.
	void setGyroODR(uint8_t gRate);
	
	// setAccelODR() -- Set the output data rate of the accelerometer
	// Input:
	//	- aRate = The desired output rate of the accel.
	void setAccelODR(uint8_t aRate); 	
	
	// setMagODR() -- Set the output data rate of the magnetometer
	// Input:
	//	- mRate = The desired output rate of the mag.
	void setMagODR(uint8_t mRate);
		
	// INT_GEN_CFG_XL
	void configAccelInt(uint8_t generator, bool andInterrupts = false);
	// INT_GEN_THS_X_XL, INT_GEN_THS_Y_XL, INT_GEN_THS_Z_XL, INT_GEN_DUR_XL
	void configAccelThs(uint8_t threshold, lsm9ds1_axis axis, uint8_t duration = 0, bool wait = 0);
	
	// INT_GEN_CFG_G
	void configGyroInt(uint8_t generator, bool aoi, bool latch);
	// INT_GEN_THS_X_G, INT_GEN_THS_Y_G, INT_GEN_THS_Z_G, INT_GEN_DUR_G
	void configGyroThs(int16_t threshold, lsm9ds1_axis axis, uint8_t duration, bool wait);
	
	// INT_CFG_M
	void configMagInt(uint8_t generator, h_lactive activeLow, bool latch = true);
	// INT_THS_L
	void configMagThs(uint16_t threshold);
		
	// INT1_CTRL, INT2_CTRL, CTRL_REG8
	void configInt(interrupt_select interupt, uint8_t generator,
				   h_lactive activeLow = INT_ACTIVE_LOW, pp_od pushPull = INT_PUSH_PULL);
		
	// INT_GEN_SRC_G
	uint8_t getGyroIntSrc();
	// INT_GEN_SRC_XL
	uint8_t getAccelIntSrc();
	// INT_SRC_M
	uint8_t getMagIntSrc();
		
		// CTRL_REG9
		void sleepGyro(bool enable);
		

protected:	
	// x_mAddress and gAddress store the I2C address or SPI chip select pin
	// for each sensor.
	uint8_t _mAddress, _xgAddress;
	// _interfaceMode keeps track of whether we're using SPI or I2C to talk
	//interface_mode _interfaceMode;
	
	// gScale, aScale, and mScale store the current scale range for each 
	// sensor. Should be updated whenever that value changes.
	/*gyro_scale gScale;
	accel_scale aScale;
	mag_scale mScale;*/
	
	// gRes, aRes, and mRes store the current resolution for each sensor. 
	// Units of these values would be DPS (or g's or Gs's) per ADC tick.
	// This value is calculated as (sensor scale) / (2^15).
	float gRes, aRes, mRes;
	
	// initGyro() -- Sets up the gyroscope to begin reading.
	// This function steps through all five gyroscope control registers.
	// Upon exit, the following parameters will be set:
	//	- CTRL_REG1_G = 0x0F: Normal operation mode, all axes enabled. 
	//		95 Hz ODR, 12.5 Hz cutoff frequency.
	//	- CTRL_REG2_G = 0x00: HPF set to normal mode, cutoff frequency
	//		set to 7.2 Hz (depends on ODR).
	//	- CTRL_REG3_G = 0x88: Interrupt enabled on INT_G (set to push-pull and
	//		active high). Data-ready output enabled on DRDY_G.
	//	- CTRL_REG4_G = 0x00: Continuous update mode. Data LSB stored in lower
	//		address. Scale set to 245 DPS. SPI mode set to 4-wire.
	//	- CTRL_REG5_G = 0x00: FIFO disabled. HPF disabled.
	void initGyro();
	
	// initAccel() -- Sets up the accelerometer to begin reading.
	// This function steps through all accelerometer related control registers.
	// Upon exit these registers will be set as:
	//	- CTRL_REG0_XM = 0x00: FIFO disabled. HPF bypassed. Normal mode.
	//	- CTRL_REG1_XM = 0x57: 100 Hz data rate. Continuous update.
	//		all axes enabled.
	//	- CTRL_REG2_XM = 0x00:  2g scale. 773 Hz anti-alias filter BW.
	//	- CTRL_REG3_XM = 0x04: Accel data ready signal on INT1_XM pin.
	void initAccel();
	
	// initMag() -- Sets up the magnetometer to begin reading.
	// This function steps through all magnetometer-related control registers.
	// Upon exit these registers will be set as:
	//	- CTRL_REG4_XM = 0x04: Mag data ready signal on INT2_XM pin.
	//	- CTRL_REG5_XM = 0x14: 100 Hz update rate. Low resolution. Interrupt
	//		requests don't latch. Temperature sensor disabled.
	//	- CTRL_REG6_XM = 0x00:  2 Gs scale.
	//	- CTRL_REG7_XM = 0x00: Continuous conversion mode. Normal HPF mode.
	//	- INT_CTRL_REG_M = 0x09: Interrupt active-high. Enable interrupts.
	void initMag();
	
	// gReadByte() -- Reads a byte from a specified gyroscope register.
	// Input:
	// 	- subAddress = Register to be read from.
	// Output:
	// 	- An 8-bit value read from the requested address.
	uint8_t mReadByte(uint8_t subAddress);
	
	// gReadBytes() -- Reads a number of bytes -- beginning at an address
	// and incrementing from there -- from the gyroscope.
	// Input:
	// 	- subAddress = Register to be read from.
	// 	- * dest = A pointer to an array of uint8_t's. Values read will be
	//		stored in here on return.
	//	- count = The number of bytes to be read.
	// Output: No value is returned, but the `dest` array will store
	// 	the data read upon exit.
	void mReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);
	
	// gWriteByte() -- Write a byte to a register in the gyroscope.
	// Input:
	//	- subAddress = Register to be written to.
	//	- data = data to be written to the register.
	void mWriteByte(uint8_t subAddress, uint8_t data);
	
	// xmReadByte() -- Read a byte from a register in the accel/mag sensor
	// Input:
	//	- subAddress = Register to be read from.
	// Output:
	//	- An 8-bit value read from the requested register.
	uint8_t xgReadByte(uint8_t subAddress);
	
	// xmReadBytes() -- Reads a number of bytes -- beginning at an address
	// and incrementing from there -- from the accelerometer/magnetometer.
	// Input:
	// 	- subAddress = Register to be read from.
	// 	- * dest = A pointer to an array of uint8_t's. Values read will be
	//		stored in here on return.
	//	- count = The number of bytes to be read.
	// Output: No value is returned, but the `dest` array will store
	// 	the data read upon exit.
	void xgReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);
	
	// xmWriteByte() -- Write a byte to a register in the accel/mag sensor.
	// Input:
	//	- subAddress = Register to be written to.
	//	- data = data to be written to the register.
	void xgWriteByte(uint8_t subAddress, uint8_t data);
	
	// calcgRes() -- Calculate the resolution of the gyroscope.
	// This function will set the value of the gRes variable. gScale must
	// be set prior to calling this function.
	void calcgRes();
	
	// calcmRes() -- Calculate the resolution of the magnetometer.
	// This function will set the value of the mRes variable. mScale must
	// be set prior to calling this function.
	void calcmRes();
	
	// calcaRes() -- Calculate the resolution of the accelerometer.
	// This function will set the value of the aRes variable. aScale must
	// be set prior to calling this function.
	void calcaRes();
	
	//////////////////////
	// Helper Functions //
	//////////////////////
	void constrainScales();
	
	///////////////////
	// SPI Functions //
	///////////////////
	// initSPI() -- Initialize the SPI hardware.
	// This function will setup all SPI pins and related hardware.
	void initSPI();
	
	// SPIwriteByte() -- Write a byte out of SPI to a register in the device
	// Input:
	//	- csPin = The chip select pin of the slave device.
	//	- subAddress = The register to be written to.
	//	- data = Byte to be written to the register.
	void SPIwriteByte(uint8_t csPin, uint8_t subAddress, uint8_t data);
	
	// SPIreadByte() -- Read a single byte from a register over SPI.
	// Input:
	//	- csPin = The chip select pin of the slave device.
	//	- subAddress = The register to be read from.
	// Output:
	//	- The byte read from the requested address.
	uint8_t SPIreadByte(uint8_t csPin, uint8_t subAddress);
	
	// SPIreadBytes() -- Read a series of bytes, starting at a register via SPI
	// Input:
	//	- csPin = The chip select pin of a slave device.
	//	- subAddress = The register to begin reading.
	// 	- * dest = Pointer to an array where we'll store the readings.
	//	- count = Number of registers to be read.
	// Output: No value is returned by the function, but the registers read are
	// 		all stored in the *dest array given.
	void SPIreadBytes(uint8_t csPin, uint8_t subAddress, 
							uint8_t * dest, uint8_t count);
	
	///////////////////
	// I2C Functions //
	///////////////////
	// initI2C() -- Initialize the I2C hardware.
	// This function will setup all I2C pins and related hardware.
	void initI2C();
	
	// I2CwriteByte() -- Write a byte out of I2C to a register in the device
	// Input:
	//	- address = The 7-bit I2C address of the slave device.
	//	- subAddress = The register to be written to.
	//	- data = Byte to be written to the register.
	void I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data);
	
	// I2CreadByte() -- Read a single byte from a register over I2C.
	// Input:
	//	- address = The 7-bit I2C address of the slave device.
	//	- subAddress = The register to be read from.
	// Output:
	//	- The byte read from the requested address.
	uint8_t I2CreadByte(uint8_t address, uint8_t subAddress);
	
	// I2CreadBytes() -- Read a series of bytes, starting at a register via SPI
	// Input:
	//	- address = The 7-bit I2C address of the slave device.
	//	- subAddress = The register to begin reading.
	// 	- * dest = Pointer to an array where we'll store the readings.
	//	- count = Number of registers to be read.
	// Output: No value is returned by the function, but the registers read are
	// 		all stored in the *dest array given.
	void I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count);
};

#endif // SFE_LSM9DS1_H //
