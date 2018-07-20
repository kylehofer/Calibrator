/*
 * Copyright (c) 2018 Kyle Hofer
 * Email: kylehofer@neurak.com.au
 * Web: https://neurak.com.au/
 *
 * This file is part of Calibrator.
 *
 * Calibrator is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Calibrator is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Arduino sketch to quickly calculate and calibrate your MPU6050 offsets
 */

#include <Wire.h>
#include <stdarg.h>

// Text Data
#define START_MSG F("Starting up I2C Interface...\r\nConnecting to MPU6050...")
#define CONNECTION_FAILED_MSG F("Connection Failed, Please connect the device try again.")
#define CONNECTION_SUCCESSFUL_MSG F("Connection successful.\r\nWaking up the MPU6050")
#define SLAP_MSG F(" *SLAP*")
#define DEVICE_AWAKE_MSG F("Device is awake, Please configure the device for tests.")
#define DLPF_MSG F("Choose a setting for the Digital Low Pass Filter [0-6]: ")
#define GYRO_SENSITIVITY_MSG F("Choose a setting for the Gyro Sensitivity [0-3]: ")
#define ACCEL_SENSITIVITY_MSG F("Choose a setting for the Accelerometer Sensitivity [0-3]: ")
#define PRE_CALCULATE_MSG F("\r\nPlace the device on a flat surface, and make sure it won't move while calculating the average.\r\nThis will take roughly 12.5 seconds.\r\nSend any data to begin calculations...\r\n")
#define CALCULATE_MSG F("Average data is being calculated, please don't move the device.\r\n\r\n	[")
#define POST_CALCULATE_MSG F("]\r\n\r\nReadings Complete, Data shown below:\r\n")
#define ACCEL_TITLE_MSG F("--------  Accelerometer Data  --------")
#define GYRO_TITLE_MSG F("--------  Gyro Data  --------")
#define TEMP_TITLE_MSG F("--------  Temperature Data  --------")
#define MIN_MAX_MSG F("Min/Max %c:	%d/%d\r\n")
#define AVG_MSG F("Average %c:	%d\r\n")
#define EXPECTED_MSG F("Expected %c:	%d\r\n\r\n")
#define CALIBRATED_MSG F("Calibration %c Offset:	%d\r\n")
//F(



// MPU6050 I2C Address
#define MPU6050_ADDRESS			0x68

/*
 * MPU6050 Data Register Macros
 */
#define MPU6050_CONFIG			0x1A
#define GYRO_CONFIG				0x1B
#define ACCEL_CONFIG			0x1C
#define PWR_MGMT_1				0x6B
#define ACCEL_XOUT_H			0x3B
#define WHO_AM_I				0x75

/*
 * MPU6050 Data Bit Settings Macro
 */

// Digital Low Pass Filter (DLPF) settings
#define DLPF_CFG_0		0
#define DLPF_CFG_1		1
#define DLPF_CFG_2		2

// Full Scale Range Gyroscope Settings
#define FS_SEL_0		3
#define FS_SEL_1		4

// Full Scale Range Accelerometer Settings
#define AFS_SEL_0		3
#define AFS_SEL_1		4

//MPU6050 Data Packet Size
#define MPU6050_DATA_SIZE 		14

#define M_PI 3.14159265358979323846
#define TIME_CONST 0.001
#define GYRO_WEIGHT 20
#define AVERAGE_MAX 255
#define ACCEL_BASE_SENSITIVITY 16384

/*
 * Frequency of Data Probes
 * With a time of 50ms, 20 probes will occur per second.
 * At 20 probes per second the calculation will finish in 12.5 seconds.
 */
#define TIME_DELAY 50

//Used to both access and set raw data array sizes
#define SMALL_SENSOR_DATA_FIELDS_LEN (sizeof(Small_Sensor_Data_Fields) / sizeof(int16_t))
#define LARGE_SENSOR_DATA_FIELDS_LEN (sizeof(Large_Sensor_Data_Fields) / sizeof(int32_t))

/*
 * Data storage structs used for calculating average/min/max
 * Can access both raw data arrays and individual fields
 */

typedef struct {
	int16_t AccX, AccY, AccZ, Temp, GyrX, GyrY, GyrZ;
} Small_Sensor_Data_Fields;

typedef union {
	Small_Sensor_Data_Fields fields;
	int16_t raw[SMALL_SENSOR_DATA_FIELDS_LEN];
} Small_Sensor_Data_t;

typedef struct {
	int32_t AccX, AccY, AccZ, Temp, GyrX, GyrY, GyrZ;
} Large_Sensor_Data_Fields;

typedef union {
	Large_Sensor_Data_Fields fields;
	int32_t raw[LARGE_SENSOR_DATA_FIELDS_LEN];
} Large_Sensor_Data_t;

/*
 * Storage for persistant Sensor Data
 */

static Small_Sensor_Data_t SENSOR_DATA, SENSOR_DATA_MIN, SENSOR_DATA_MAX, SENSOR_DATA_EXPECTED;
static Large_Sensor_Data_t SENSOR_DATA_TOTAL;

//Uses vsnprintf to formulate a formated string, then sends that string to the Serial connection
void printf(const __FlashStringHelper *fmt, ... ){
	char buf[128];
	va_list args;
	va_start (args, fmt);
	vsnprintf_P(buf, sizeof(buf), (const char *)fmt, args);
	va_end(args);
	Serial.print(buf);
}

/*
 * Sensor Data Modifier Methods
 * Used to store and modify Sensor Data structs
 */

void SENSORDATA_min(Small_Sensor_Data_t *to, Small_Sensor_Data_t *from) {
	for (uint8_t i = 0; i < SMALL_SENSOR_DATA_FIELDS_LEN; i++)
		if (to->raw[i] > from->raw[i])
			to->raw[i] = from->raw[i];
}

void SENSORDATA_max(Small_Sensor_Data_t *to, Small_Sensor_Data_t *from) {
	for (uint8_t i = 0; i < SMALL_SENSOR_DATA_FIELDS_LEN; i++)
		if (to->raw[i] < from->raw[i])
			to->raw[i] = from->raw[i];
}

void SENSORDATA_copy(Small_Sensor_Data_t *to, Small_Sensor_Data_t *from) {
	for (uint8_t i = 0; i < SMALL_SENSOR_DATA_FIELDS_LEN; i++)
		to->raw[i] = from->raw[i];
}

void SENSORDATA_copy(Large_Sensor_Data_t *to, Large_Sensor_Data_t *from) {
	for (uint8_t i = 0; i < LARGE_SENSOR_DATA_FIELDS_LEN; i++)
		to->raw[i] = from->raw[i];
}

void SENSORDATA_addToTotal(Large_Sensor_Data_t *to, Small_Sensor_Data_t *from) {
	for (uint8_t i = 0; i < SMALL_SENSOR_DATA_FIELDS_LEN; i++)
		to->raw[i] += from->raw[i];
}

void SENSORDATA_calculateAverage(Small_Sensor_Data_t *to, Large_Sensor_Data_t *from) {
	for (uint8_t i = 0; i < SMALL_SENSOR_DATA_FIELDS_LEN; i++)
		to->raw[i] = from->raw[i] / AVERAGE_MAX;
}

/*
 * MPU6050 Methods
 * Used to communicate with the MPU6050 over I2C
 */

void MPU6050_sensor_read() {
	Wire.beginTransmission(MPU6050_ADDRESS);
	Wire.write(ACCEL_XOUT_H);							// Requesting access to 0x3B
	Wire.endTransmission(false);						// Keeping Transmission open
	Wire.requestFrom(MPU6050_ADDRESS, MPU6050_DATA_SIZE, true);	// Request a total of 14 registers and closing transmission

	SENSOR_DATA.fields = {
		Wire.read() << 8 | Wire.read(),		// 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
		Wire.read() << 8 | Wire.read(),		// 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
		Wire.read() << 8 | Wire.read(),		// 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
		Wire.read() << 8 | Wire.read(),		// 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
		Wire.read() << 8 | Wire.read(),		// 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
		Wire.read() << 8 | Wire.read(),		// 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
		Wire.read() << 8 | Wire.read()		// 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
	};
}

void MPU6050_write_byte(uint16_t start, uint8_t data) {
	Wire.beginTransmission(MPU6050_ADDRESS);
	Wire.write(start);						// Requesting access to address in start
	Wire.write(&data, 1);					// Write data byte
	Wire.endTransmission(true); 			// Closing Transmission
}

void MPU6050_read_byte(int start, uint8_t *buffer) {	 
	Wire.beginTransmission(MPU6050_ADDRESS);
	Wire.write(start);							// Requesting access to address in start
	Wire.endTransmission(false);				// Keeping Transmission open
	Wire.requestFrom(MPU6050_ADDRESS, 1, true);	// Request 1 byte and closing transmission
	buffer[0] = Wire.read();
}

bool MPU6050_init() {
	Wire.begin();

	Serial.println(START_MSG);

	uint8_t data;								//Used to both read and write data to the MPU6050

	MPU6050_read_byte(WHO_AM_I, &data);			// Reading WHO_AM_I Register to test connection

	if (data != 0x68) {							// MPU6050 not found
		Serial.println(CONNECTION_FAILED_MSG);
		Wire.end();
		return false;							// No connection
	}

	Serial.print(CONNECTION_SUCCESSFUL_MSG);

	for (int i = 0; i < 5; ++i) {
		delay(225);
		Serial.print('.');
	}

	Serial.println(SLAP_MSG);
	Serial.println();
	delay(550);
	

	Serial.println(DEVICE_AWAKE_MSG);

	MPU6050_write_byte(PWR_MGMT_1, 0);		// Wake up the MPU-6050

	/*
	 * User Inputs for setting up the MPU6050
	 */

	char reply;

	for (;;) {
		Serial.print(DLPF_MSG);
		reply = CALIBRATOR_waitForInput();
		Serial.println(reply);
		if (reply > 47 && reply < 55)	//Ascii 0-6
			break;		
	}

	switch(reply) {
		case 49:	// 1
			data = _BV(DLPF_CFG_0);
			break;
		case 50:	// 2
			data = _BV(DLPF_CFG_1);
			break;
		case 51:	// 3
			data = _BV(DLPF_CFG_1) | _BV(DLPF_CFG_0);
			break;
		case 52:	// 4
			data = _BV(DLPF_CFG_2);
			break;
		case 53:	// 5
			data = _BV(DLPF_CFG_2) | _BV(DLPF_CFG_0);
			break;
		case 54:	// 6
			data = _BV(DLPF_CFG_2) | _BV(DLPF_CFG_1);
			break;
		default:
			data = 0;
	}

	MPU6050_write_byte(MPU6050_CONFIG, data); 	// Setting Digital Low Pass Filter

	for (;;) {
		Serial.print(GYRO_SENSITIVITY_MSG);
		reply = CALIBRATOR_waitForInput();
		Serial.println(reply);
		if (reply > 47 && reply < 52)	//Ascii 0-3
			break;		
	}

	switch(reply) {
		case 49:	// 1
			data = _BV(FS_SEL_0);
			break;
		case 50:	// 2
			data = _BV(FS_SEL_1);
			break;
		case 51:	// 3
			data = _BV(FS_SEL_1) | _BV(FS_SEL_0);
			break;
		default:
			data = 0;
	}

	MPU6050_write_byte(GYRO_CONFIG, data);		// Setting Gyro Sensitivity

	for (;;) {
		Serial.print(ACCEL_SENSITIVITY_MSG);
		reply = CALIBRATOR_waitForInput();
		Serial.println(reply);
		if (reply > 47 && reply < 52)	//Ascii 0-3
			break;	
	}

	switch(reply) {
		case 49:	// 1
			data = _BV(AFS_SEL_0);
			SENSOR_DATA_EXPECTED.fields.AccZ >>= 1;	//Setting the Expected AccZ to match sensitivity
			break;
		case 50:	// 2
			data = _BV(AFS_SEL_1);
			SENSOR_DATA_EXPECTED.fields.AccZ >>= 2;
			break;
		case 51:	// 3
			data = _BV(AFS_SEL_1) | _BV(AFS_SEL_0);
			SENSOR_DATA_EXPECTED.fields.AccZ >>= 3;
			break;
		default:
			data = 0;
	}

	MPU6050_write_byte(ACCEL_CONFIG, data);		// Setting Accelerometer Sensitivity

	Serial.println(PRE_CALCULATE_MSG);
	CALIBRATOR_waitForInput();
	Serial.print(CALCULATE_MSG);
	return true;							// Connection Successful
}

/*
 * CALIBRATOR Methods
 * Acts as an interface between the user and the MPU6050
 */

void CALIBRATOR_calculate() {
	uint8_t avgCount = 0;

	while(avgCount < AVERAGE_MAX) {
		delay(TIME_DELAY);
		MPU6050_sensor_read();				// Reading data from MPU6050

		if (avgCount % 15 == 0)
			Serial.print('#');			// Printing Progress Bar

		SENSORDATA_min(&SENSOR_DATA_MIN, &SENSOR_DATA);	//Storing Minimum Readings
		SENSORDATA_max(&SENSOR_DATA_MAX, &SENSOR_DATA);	//Storing Maximum Readings

		SENSORDATA_addToTotal(&SENSOR_DATA_TOTAL, &SENSOR_DATA);	//Storing Average Readings

		avgCount++;
	}
}

void CALIBRATOR_display() {
	Serial.println(POST_CALCULATE_MSG);

	SENSORDATA_calculateAverage(&SENSOR_DATA, &SENSOR_DATA_TOTAL);

	Serial.println(ACCEL_TITLE_MSG);

	char x = 'X', y = 'Y', z = 'Z', t = 't';

	printf(MIN_MAX_MSG, x, SENSOR_DATA_MIN.fields.AccX, SENSOR_DATA_MAX.fields.AccX);
	printf(AVG_MSG, x, SENSOR_DATA.fields.AccX);
	printf(EXPECTED_MSG, x, SENSOR_DATA_EXPECTED.fields.AccX);

	printf(MIN_MAX_MSG, y, SENSOR_DATA_MIN.fields.AccY, SENSOR_DATA_MAX.fields.AccY);
	printf(AVG_MSG, y, SENSOR_DATA.fields.AccY);
	printf(EXPECTED_MSG, y, SENSOR_DATA_EXPECTED.fields.AccY);

	printf(MIN_MAX_MSG, z, SENSOR_DATA_MIN.fields.AccZ, SENSOR_DATA_MAX.fields.AccZ);
	printf(AVG_MSG, z, SENSOR_DATA.fields.AccZ);
	printf(EXPECTED_MSG, z, SENSOR_DATA_EXPECTED.fields.AccZ);

	printf(CALIBRATED_MSG, x, SENSOR_DATA_EXPECTED.fields.AccX - SENSOR_DATA.fields.AccX);
	printf(CALIBRATED_MSG, y, SENSOR_DATA_EXPECTED.fields.AccY - SENSOR_DATA.fields.AccY);
	printf(CALIBRATED_MSG, z, SENSOR_DATA_EXPECTED.fields.AccZ - SENSOR_DATA.fields.AccZ);

	Serial.println();

	Serial.println(GYRO_TITLE_MSG);

	printf(MIN_MAX_MSG, x, SENSOR_DATA_MIN.fields.GyrX, SENSOR_DATA_MAX.fields.GyrX);
	printf(AVG_MSG, x, SENSOR_DATA.fields.GyrX);
	printf(EXPECTED_MSG, x, SENSOR_DATA_EXPECTED.fields.GyrX);

	printf(MIN_MAX_MSG, y, SENSOR_DATA_MIN.fields.GyrY, SENSOR_DATA_MAX.fields.GyrY);
	printf(AVG_MSG, y, SENSOR_DATA.fields.GyrY);
	printf(EXPECTED_MSG, y, SENSOR_DATA_EXPECTED.fields.GyrY);

	printf(MIN_MAX_MSG, z, SENSOR_DATA_MIN.fields.GyrZ, SENSOR_DATA_MAX.fields.GyrZ);
	printf(AVG_MSG, z, SENSOR_DATA.fields.GyrZ);
	printf(EXPECTED_MSG, z, SENSOR_DATA_EXPECTED.fields.GyrZ);

	printf(CALIBRATED_MSG, x, SENSOR_DATA_EXPECTED.fields.GyrX - SENSOR_DATA.fields.GyrX);
	printf(CALIBRATED_MSG, y, SENSOR_DATA_EXPECTED.fields.GyrY - SENSOR_DATA.fields.GyrY);
	printf(CALIBRATED_MSG, z, SENSOR_DATA_EXPECTED.fields.GyrZ - SENSOR_DATA.fields.GyrZ);

	Serial.println();

	Serial.println(TEMP_TITLE_MSG);

	printf(MIN_MAX_MSG, t, SENSOR_DATA_MIN.fields.Temp, SENSOR_DATA_MAX.fields.Temp);
	printf(AVG_MSG, t, SENSOR_DATA.fields.Temp);
}

uint8_t CALIBRATOR_waitForInput() {
	int8_t data = 0;
	while (Serial.available() && Serial.read());	// empty buffer
	while (!Serial.available());					// wait for data
	while (Serial.available() && (data = Serial.read()));	// empty buffer again	
	return data;
}

void setup() {
	Serial.begin(115200);

	for (uint8_t i = 0; i < SMALL_SENSOR_DATA_FIELDS_LEN; i++)
		SENSOR_DATA_MIN.raw[i] = 0x7FFF;

	for (uint8_t i = 0; i < SMALL_SENSOR_DATA_FIELDS_LEN; i++)
		SENSOR_DATA_MAX.raw[i] = 0x8000;

	SENSOR_DATA_EXPECTED.fields.AccZ = ACCEL_BASE_SENSITIVITY;		//Setting the Expected AccZ to 1g (laying flat)

	bool connected = false;
	while (!connected) {
		CALIBRATOR_waitForInput();
		connected = MPU6050_init();
	}

	CALIBRATOR_calculate();
	CALIBRATOR_display();
}

void loop() { }
