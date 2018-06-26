/*=============================================================================
 A simple Arduino -> MyRIO interface for the BNO080 IMU 
 By: Alexander Su
 Date: 17/5/2018

 A modified and heavily trimmed version of Nathan Seidle's code
https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library
 
 Desc: This code reads the BNO080 IMU's data via I2C and then 
 passes it along a serial communication line to the MyRIO. The
 MyRIO reads it by its default UART connection.
 */

/*=============================================================================
Wiring the BNO080

Note: Hold the BNO080 facing you with the text facing up. We only need the left
hand side interface/connectors.

GND -> Arduino Ground
3V3 -> Arduino 3.3V
SDA -> Arduino SDA (A4)
SCL -> Arduino SCL (A5)
 */


/*=============================================================================
Wiring the Arduino

*/

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

//The default I2C address for the BNO080 on the SparkX breakout is 0x4B. 0x4A is also possible.
#define BNO080_DEFAULT_ADDRESS 0x4B#include <Wire.h>

//The default I2C address for the BNO080 on the SparkX breakout is 0x4B. 0x4A is also possible.
#define BNO080_DEFAULT_ADDRESS 0x4B

//Platform specific configurations

//Define the size of the I2C buffer based on the platform the user has
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)

//I2C_BUFFER_LENGTH is defined in Wire.H
#define I2C_BUFFER_LENGTH BUFFER_LENGTH

#elif defined(__SAMD21G18A__)

//SAMD21 uses RingBuffer.h
#define I2C_BUFFER_LENGTH SERIAL_BUFFER_SIZE

#elif __MK20DX256__
//Teensy

#elif ARDUINO_ARCH_ESP32
//ESP32 based platforms

#else

//The catch-all default is 32
#define I2C_BUFFER_LENGTH 32

#endif
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Registers
const byte CHANNEL_COMMAND = 0;
const byte CHANNEL_EXECUTABLE = 1;
const byte CHANNEL_CONTROL = 2;
const byte CHANNEL_REPORTS = 3;
const byte CHANNEL_WAKE_REPORTS = 4;
const byte CHANNEL_GYRO = 5;

//All the ways we can configure or talk to the BNO080, figure 34, page 36 reference manual
//These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER 0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E

//Record IDs from figure 29, page 29 reference manual
//These are used to read the metadata for each sensor type
#define FRS_RECORDID_ACCELEROMETER 0xE302
#define FRS_RECORDID_GYROSCOPE_CALIBRATED 0xE306
#define FRS_RECORDID_MAGNETIC_FIELD_CALIBRATED 0xE309
#define FRS_RECORDID_ROTATION_VECTOR 0xE30B

//Command IDs from section 6.4, page 42
//These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5

#define MAX_PACKET_SIZE 128 //Packets can be up to 32k but we don't have that much RAM.
#define MAX_METADATA_SIZE 9 //This is in words. There can be many but we mostly only care about the first 9 (Qs, range, etc)

class BNO080 {
  public:

    boolean begin(uint8_t deviceAddress = BNO080_DEFAULT_ADDRESS, TwoWire &wirePort = Wire); //By default use the default I2C addres, and use Wire port

    void enableDebugging(Stream &debugPort = Serial); //Turn on debug printing. If user doesn't specify then Serial will be used.

	void softReset(); //Try to reset the IMU via software
	uint8_t resetReason(); //Query the IMU for the reason it last reset

	float qToFloat(int16_t fixedPointValue, uint8_t qPoint); //Given a Q value, converts fixed point floating to regular floating point number

	boolean waitForI2C(); //Delay based polling for I2C traffic
	boolean receivePacket(void);
	boolean getData(uint16_t bytesRemaining); //Given a number of bytes, send the requests in I2C_BUFFER_LENGTH chunks
	boolean sendPacket(uint8_t channelNumber, uint8_t dataLength);
	void printPacket(void); //Prints the current shtp header and data packets

	void enableRotationVector(uint16_t timeBetweenReports);
	void enableGameRotationVector(uint16_t timeBetweenReports);
	void enableAccelerometer(uint16_t timeBetweenReports);
	void enableLinearAccelerometer(uint16_t timeBetweenReports);
	void enableGyro(uint16_t timeBetweenReports);
	void enableMagnetometer(uint16_t timeBetweenReports);
	void enableStepCounter(uint16_t timeBetweenReports);
	void enableStabilityClassifier(uint16_t timeBetweenReports);
	void enableActivityClassifier(uint16_t timeBetweenReports, uint32_t activitiesToEnable, uint8_t (&activityConfidences)[9]);

	bool dataAvailable(void);
	void parseInputReport(void);
	
	float getQuatI();
	float getQuatJ();
	float getQuatK();
	float getQuatReal();
	float getQuatRadianAccuracy();
	uint8_t getQuatAccuracy();

	float getAccelX();
	float getAccelY();
	float getAccelZ();
	uint8_t getAccelAccuracy();

	float getLinAccelX();
	float getLinAccelY();
	float getLinAccelZ();
	uint8_t getLinAccelAccuracy();

	float getGyroX();
	float getGyroY();
	float getGyroZ();
	uint8_t getGyroAccuracy();

	float getMagX();
	float getMagY();
	float getMagZ();
	uint8_t getMagAccuracy();

	void calibrateAccelerometer();
	void calibrateGyro();
	void calibrateMagnetometer();
	void calibratePlanarAccelerometer();
	void calibrateAll();
	void endCalibration();
	void saveCalibration();
	
	uint16_t getStepCount();
	uint8_t getStabilityClassifier();
	uint8_t getActivityClassifier();

	void setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports);
	void setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig);
	void sendCommand(uint8_t command);
	void sendCalibrateCommand(uint8_t thingToCalibrate);	
	
	//Metadata functions
	int16_t getQ1(uint16_t recordID);
	int16_t getQ2(uint16_t recordID);
	int16_t getQ3(uint16_t recordID);
	float getResolution(uint16_t recordID);
	float getRange(uint16_t recordID);
	uint32_t readFRSword(uint16_t recordID, uint8_t wordNumber);
	void frsReadRequest(uint16_t recordID, uint16_t readOffset, uint16_t blockSize);
	bool readFRSdata(uint16_t recordID, uint8_t startLocation, uint8_t wordsToRead);

	//Global Variables
	uint8_t shtpHeader[4]; //Each packet has a header of 4 bytes
	uint8_t shtpData[MAX_PACKET_SIZE]; 
	uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; //There are 6 com channels. Each channel has its own seqnum
	uint8_t commandSequenceNumber = 0; //Commands have a seqNum as well. These are inside command packet, the header uses its own seqNum per channel
	uint32_t metaData[MAX_METADATA_SIZE]; //There is more than 10 words in a metadata record but we'll stop at Q point 3

  private:

    //Variables
    TwoWire *_i2cPort; //The generic connection to user's chosen I2C hardware
    uint8_t _deviceAddress; //Keeps track of I2C address. setI2CAddress changes this.

    Stream *_debugPort; //The stream to send debug messages to if enabled. Usually Serial.
    boolean _printDebug = false; //Flag to print debugging variables

	//These are the raw sensor values pulled from the user requested Input Report
	uint16_t rawAccelX, rawAccelY, rawAccelZ, accelAccuracy;
	uint16_t rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy;
	uint16_t rawGyroX, rawGyroY, rawGyroZ, gyroAccuracy;
	uint16_t rawMagX, rawMagY, rawMagZ, magAccuracy;
	uint16_t rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy;
	uint16_t stepCount;
	uint8_t stabilityClassifier;
	uint8_t activityClassifier;
	uint8_t *_activityConfidences; //Array that store the confidences of the 9 possible activities
	
	//These Q values are defined in the datasheet but can also be obtained by querying the meta data records
	//See the read metadata example for more info
	int16_t rotationVector_Q1 = 14;
	int16_t accelerometer_Q1 = 8;
	int16_t linear_accelerometer_Q1 = 8;
	int16_t gyro_Q1 = 9;
	int16_t magnetometer_Q1 = 4;
};


#include <Wire.h>
BNO080 gyro;

void setup()
{
  // Serial communication with the MyRIO + status info
  Serial.begin(115200);
  
  // I2C communication with the IMU
  Wire.begin();
  //Wire.setClock(40000L);
  
  // Initialise the gyro
  gyro.begin();
  gyro.enableRotationVector(50);
}

unsigned long timestamp = 10000;

void loop(){
 
  if(gyro.dataAvailable() == true)
  {
    timestamp = millis();
    
    float quatI = gyro.getQuatI();
    float quatJ = gyro.getQuatJ();
    float quatK = gyro.getQuatK();
    float quatReal = gyro.getQuatReal();
    float quatRadianAccuracy = gyro.getQuatRadianAccuracy();
    
    if(quatI > 0) Serial.print('+');
    Serial.print(quatI, 2);

    Serial.print(F(","));

    if(quatJ > 0) Serial.print('+');
    Serial.print(quatJ, 2);

    Serial.print(F(","));

    if(quatK > 0) Serial.print('+');
    Serial.print(quatK, 2);

    Serial.print(F(","));

    if(quatReal > 0) Serial.print('+');
    Serial.print(quatReal, 2);

    Serial.println();
  }

}

//Attempt communication with the device
//Return true if we got a 'Polo' back from Marco
boolean BNO080::begin(uint8_t deviceAddress, TwoWire &wirePort)
{
	_deviceAddress = deviceAddress; //If provided, store the I2C address from user
	_i2cPort = &wirePort; //Grab which port the user wants us to use

	//We expect caller to begin their I2C port, with the speed of their choice external to the library
	//But if they forget, we start the hardware here.
	_i2cPort->begin();

	//Begin by resetting the IMU
	softReset();

	//Check communication with device
	shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
	shtpData[1] = 0; //Reserved
	
	//Transmit packet on channel 2, 2 bytes
	sendPacket(CHANNEL_CONTROL, 2);
	
	//Now we wait for response
	if (receivePacket() == true)
	{
		if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
		{
			return(true);
		}
	}

	return(false); //Something went wrong
}


//Calling this function with nothing sets the debug port to Serial
//You can also call it with other streams like Serial1, SerialUSB, etc.
void BNO080::enableDebugging(Stream &debugPort)
{
	_debugPort = &debugPort;
	_printDebug = true;
}

//Updates the latest variables if possible
//Returns false if new readings are not available
bool BNO080::dataAvailable(void)
{
	if (receivePacket() == true)
	{
		//Check to see if this packet is a sensor reporting its data to us
		if (shtpHeader[2] == CHANNEL_REPORTS && shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP)
		{
			parseInputReport(); //This will update the rawAccelX, etc variables depending on which feature report is found
			return(true);
		}
	}
	return(false);
}

//This function pulls the data from the input report
//The input reports vary in length so this function stores the various 16-bit values as globals

//Unit responds with packet that contains the following:
//shtpHeader[0:3]: First, a 4 byte header
//shtpData[0:4]: Then a 5 byte timestamp of microsecond clicks since reading was taken
//shtpData[5 + 0]: Then a feature report ID (0x01 for Accel, 0x05 for Rotation Vector)
//shtpData[5 + 1]: Sequence number (See 6.5.18.2)
//shtpData[5 + 2]: Status
//shtpData[3]: Delay
//shtpData[4:5]: i/accel x/gyro x/etc
//shtpData[6:7]: j/accel y/gyro y/etc
//shtpData[8:9]: k/accel z/gyro z/etc
//shtpData[10:11]: real/gyro temp/etc
//shtpData[12:13]: Accuracy estimate
void BNO080::parseInputReport(void)
{
	//Calculate the number of data bytes in this packet
	int16_t dataLength = ((uint16_t)shtpHeader[1] << 8 | shtpHeader[0]);
	dataLength &= ~(1 << 15); //Clear the MSbit. This bit indicates if this package is a continuation of the last. 
	//Ignore it for now. TODO catch this as an error and exit

	dataLength -= 4; //Remove the header bytes from the data count

    uint8_t status = shtpData[5 + 2] & 0x03; //Get status bits
	uint16_t data1 = (uint16_t)shtpData[5 + 5] << 8 | shtpData[5 + 4];
    uint16_t data2 = (uint16_t)shtpData[5 + 7] << 8 | shtpData[5 + 6];
    uint16_t data3 = (uint16_t)shtpData[5 + 9] << 8 | shtpData[5 + 8];
	uint16_t data4 = 0;
	uint16_t data5 = 0;

	if(dataLength - 5 > 9)
	{
		data4= (uint16_t)shtpData[5 + 11] << 8 | shtpData[5 + 10];
	}
	if(dataLength - 5 > 11)
	{
		data5 = (uint16_t)shtpData[5 + 13] << 8 | shtpData[5 + 12];
	}
	
	//Store these generic values to their proper global variable
	if(shtpData[5] == SENSOR_REPORTID_ACCELEROMETER)
	{
		accelAccuracy = status;
		rawAccelX = data1;
		rawAccelY = data2;
		rawAccelZ = data3;
	}
	else if(shtpData[5] == SENSOR_REPORTID_LINEAR_ACCELERATION)
	{
		accelLinAccuracy = status;
		rawLinAccelX = data1;
		rawLinAccelY = data2;
		rawLinAccelZ = data3;
	}
	else if(shtpData[5] == SENSOR_REPORTID_GYROSCOPE)
	{
		gyroAccuracy = status;
		rawGyroX = data1;
		rawGyroY = data2;
		rawGyroZ = data3;
	}
	else if(shtpData[5] == SENSOR_REPORTID_MAGNETIC_FIELD)
	{
		magAccuracy = status;
		rawMagX = data1;
		rawMagY = data2;
		rawMagZ = data3;
	}
	else if(shtpData[5] == SENSOR_REPORTID_ROTATION_VECTOR || shtpData[5] == SENSOR_REPORTID_GAME_ROTATION_VECTOR)
	{
		quatAccuracy = status;
		rawQuatI = data1;
		rawQuatJ = data2;
		rawQuatK = data3;
		rawQuatReal = data4;
		rawQuatRadianAccuracy = data5; //Only available on rotation vector, not game rot vector
	}
	else if(shtpData[5] == SENSOR_REPORTID_STEP_COUNTER)
	{
		stepCount = data3; //Bytes 8/9
	}
	else if(shtpData[5] == SENSOR_REPORTID_STABILITY_CLASSIFIER)
	{
		stabilityClassifier = shtpData[5 + 4]; //Byte 4 only
	}
	else if(shtpData[5] == SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER)
	{
		activityClassifier = shtpData[5 + 5]; //Most likely state
		
		//Load activity classification confidences into the array
		for(uint8_t x = 0 ; x < 9 ; x++) //Hardcoded to max of 9. TODO - bring in array size
			_activityConfidences[x] = shtpData[5 + 6 + x]; //5 bytes of timestamp, byte 6 is first confidence byte
	}
	else
	{
		//This sensor report ID is unhandled.
		//See reference manual to add additional feature reports as needed
	}
	
	//TODO additional feature reports may be strung together. Parse them all.
}

//Return the rotation vector quaternion I
float BNO080::getQuatI()
{
    float quat = qToFloat(rawQuatI, rotationVector_Q1);
	return(quat);
}

//Return the rotation vector quaternion J
float BNO080::getQuatJ()
{
    float quat = qToFloat(rawQuatJ, rotationVector_Q1);
	return(quat);
}

//Return the rotation vector quaternion K
float BNO080::getQuatK()
{
    float quat = qToFloat(rawQuatK, rotationVector_Q1);
	return(quat);
}

//Return the rotation vector quaternion Real
float BNO080::getQuatReal()
{
    float quat = qToFloat(rawQuatReal, rotationVector_Q1);
	return(quat);
}

//Return the rotation vector accuracy
float BNO080::getQuatRadianAccuracy()
{
    float quat = qToFloat(rawQuatRadianAccuracy, rotationVector_Q1);
	return(quat);
}

//Return the acceleration component
uint8_t BNO080::getQuatAccuracy()
{
	return(quatAccuracy);
}

//Return the acceleration component
float BNO080::getAccelX()
{
    float accel = qToFloat(rawAccelX, accelerometer_Q1);
	return(accel);
}

//Return the acceleration component
float BNO080::getAccelY()
{
    float accel = qToFloat(rawAccelY, accelerometer_Q1);
	return(accel);
}

//Return the acceleration component
float BNO080::getAccelZ()
{
    float accel = qToFloat(rawAccelZ, accelerometer_Q1);
	return(accel);
}

//Return the acceleration component
uint8_t BNO080::getAccelAccuracy()
{
	return(accelAccuracy);
}

// linear acceleration, i.e. minus gravity

//Return the acceleration component
float BNO080::getLinAccelX()
{
    float accel = qToFloat(rawLinAccelX, linear_accelerometer_Q1);
	return(accel);
}

//Return the acceleration component
float BNO080::getLinAccelY()
{
    float accel = qToFloat(rawLinAccelY, linear_accelerometer_Q1);
	return(accel);
}

//Return the acceleration component
float BNO080::getLinAccelZ()
{
    float accel = qToFloat(rawLinAccelZ, linear_accelerometer_Q1);
	return(accel);
}

//Return the acceleration component
uint8_t BNO080::getLinAccelAccuracy()
{
	return(accelLinAccuracy);
}

//Return the gyro component
float BNO080::getGyroX()
{
    float gyro = qToFloat(rawGyroX, gyro_Q1);
	return(gyro);
}

//Return the gyro component
float BNO080::getGyroY()
{
    float gyro = qToFloat(rawGyroY, gyro_Q1);
	return(gyro);
}

//Return the gyro component
float BNO080::getGyroZ()
{
    float gyro = qToFloat(rawGyroZ, gyro_Q1);
	return(gyro);
}

//Return the gyro component
uint8_t BNO080::getGyroAccuracy()
{
	return(gyroAccuracy);
}

//Return the magnetometer component
float BNO080::getMagX()
{
    float mag = qToFloat(rawMagX, magnetometer_Q1);
	return(mag);
}

//Return the magnetometer component
float BNO080::getMagY()
{
    float mag = qToFloat(rawMagY, magnetometer_Q1);
	return(mag);
}

//Return the magnetometer component
float BNO080::getMagZ()
{
    float mag = qToFloat(rawMagZ, magnetometer_Q1);
	return(mag);
}

//Return the mag component
uint8_t BNO080::getMagAccuracy()
{
	return(magAccuracy);
}

//Return the step count
uint16_t BNO080::getStepCount()
{
    return(stepCount);
}

//Return the stability classifier
uint8_t BNO080::getStabilityClassifier()
{
    return(stabilityClassifier);
}

//Return the activity classifier
uint8_t BNO080::getActivityClassifier()
{
    return(activityClassifier);
}

//Given a record ID, read the Q1 value from the metaData record in the FRS (ya, it's complicated)
//Q1 is used for all sensor data calculations
int16_t BNO080::getQ1(uint16_t recordID)
{
  //Q1 is always the lower 16 bits of word 7
  uint16_t q = readFRSword(recordID, 7) & 0xFFFF; //Get word 7, lower 16 bits
  return(q);
}

//Given a record ID, read the Q2 value from the metaData record in the FRS
//Q2 is used in sensor bias
int16_t BNO080::getQ2(uint16_t recordID)
{
  //Q2 is always the upper 16 bits of word 7
  uint16_t q = readFRSword(recordID, 7) >> 16; //Get word 7, upper 16 bits
  return(q);
}

//Given a record ID, read the Q3 value from the metaData record in the FRS
//Q3 is used in sensor change sensitivity
int16_t BNO080::getQ3(uint16_t recordID)
{
  //Q3 is always the upper 16 bits of word 8
  uint16_t q = readFRSword(recordID, 8) >> 16; //Get word 8, upper 16 bits
  return(q);
}

//Given a record ID, read the resolution value from the metaData record in the FRS for a given sensor
float BNO080::getResolution(uint16_t recordID)
{
  //The resolution Q value are 'the same as those used in the sensor's input report'
  //This should be Q1.
  int16_t Q = getQ1(recordID);
  
  //Resolution is always word 2
  uint32_t value = readFRSword(recordID, 2); //Get word 2

  float resolution = qToFloat(value, Q);

  return(resolution);
}

//Given a record ID, read the range value from the metaData record in the FRS for a given sensor
float BNO080::getRange(uint16_t recordID)
{
  //The resolution Q value are 'the same as those used in the sensor's input report'
  //This should be Q1.
  int16_t Q = getQ1(recordID);
  
  //Range is always word 1
  uint32_t value = readFRSword(recordID, 1); //Get word 1

  float range = qToFloat(value, Q);

  return(range);
}

//Given a record ID and a word number, look up the word data
//Helpful for pulling out a Q value, range, etc.
//Use readFRSdata for pulling out multi-word objects for a sensor (Vendor data for example)
uint32_t BNO080::readFRSword(uint16_t recordID, uint8_t wordNumber)
{
	if(readFRSdata(recordID, wordNumber, 1) == true) //Get word number, just one word in length from FRS
		return(metaData[0]); //Return this one word

	return(0); //Error
}

//Ask the sensor for data from the Flash Record System
//See 6.3.6 page 40, FRS Read Request
void BNO080::frsReadRequest(uint16_t recordID, uint16_t readOffset, uint16_t blockSize)
{
  shtpData[0] = SHTP_REPORT_FRS_READ_REQUEST; //FRS Read Request
  shtpData[1] = 0; //Reserved
  shtpData[2] = (readOffset >> 0) & 0xFF; //Read Offset LSB
  shtpData[3] = (readOffset >> 8) & 0xFF; //Read Offset MSB
  shtpData[4] = (recordID >> 0) & 0xFF; //FRS Type LSB
  shtpData[5] = (recordID >> 8) & 0xFF; //FRS Type MSB
  shtpData[6] = (blockSize >> 0) & 0xFF; //Block size LSB
  shtpData[7] = (blockSize >> 8) & 0xFF; //Block size MSB

  //Transmit packet on channel 2, 8 bytes
  sendPacket(CHANNEL_CONTROL, 8);
}

//Given a sensor or record ID, and a given start/stop bytes, read the data from the Flash Record System (FRS) for this sensor
//Returns true if metaData array is loaded successfully
//Returns false if failure
bool BNO080::readFRSdata(uint16_t recordID, uint8_t startLocation, uint8_t wordsToRead)
{
  uint8_t spot = 0;

  //First we send a Flash Record System (FRS) request
  frsReadRequest(recordID, startLocation, wordsToRead); //From startLocation of record, read a # of words

  //Read bytes until FRS reports that the read is complete
  while (1)
  {
    //Now we wait for response
    while (1)
    {
		uint8_t counter = 0;
		while(receivePacket() == false)
		{
			if(counter++ > 100) return(false); //Give up
			delay(1);
		}

		//We have the packet, inspect it for the right contents
		//See page 40. Report ID should be 0xF3 and the FRS types should match the thing we requested
		if (shtpData[0] == SHTP_REPORT_FRS_READ_RESPONSE)
		if ( ( (uint16_t)shtpData[13] << 8 | shtpData[12]) == recordID)
			break; //This packet is one we are looking for
    }

    uint8_t dataLength = shtpData[1] >> 4;
    uint8_t frsStatus = shtpData[1] & 0x0F;

    uint32_t data0 = (uint32_t)shtpData[7] << 24 | (uint32_t)shtpData[6] << 16 | (uint32_t)shtpData[5] << 8 | (uint32_t)shtpData[4];
    uint32_t data1 = (uint32_t)shtpData[11] << 24 | (uint32_t)shtpData[10] << 16 | (uint32_t)shtpData[9] << 8 | (uint32_t)shtpData[8];

    //Record these words to the metaData array
    if (dataLength > 0)
    {
      metaData[spot++] = data0;
    }
    if (dataLength > 1)
    {
      metaData[spot++] = data1;
    }

    if (spot >= MAX_METADATA_SIZE)
    {
		if(_printDebug == true) _debugPort->println(F("metaData array over run. Returning."));
		return(true); //We have run out of space in our array. Bail.
    }

    if (frsStatus == 3 || frsStatus == 6 || frsStatus == 7)
    {
      return(true); //FRS status is read completed! We're done!
    }
  }
}

//Send command to reset IC
//Read all advertisement packets from sensor
//The sensor has been seen to reset twice if we attempt too much too quickly.
//This seems to work reliably.
void BNO080::softReset(void)
{
  shtpData[0] = 1; //Reset

	//Attempt to start communication with sensor
	sendPacket(CHANNEL_EXECUTABLE, 1); //Transmit packet on channel 1, 1 byte

  //Read all incoming data and flush it
  delay(50);
  while (receivePacket() == true) ;
  delay(50);
  while (receivePacket() == true) ;
}


//Get the reason for the last reset
//1 = POR, 2 = Internal reset, 3 = Watchdog, 4 = External reset, 5 = Other
uint8_t BNO080::resetReason()
{
	shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
	shtpData[1] = 0; //Reserved
	
	//Transmit packet on channel 2, 2 bytes
	sendPacket(CHANNEL_CONTROL, 2);
	
	//Now we wait for response
	if (receivePacket() == true)
	{
		if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
		{
			return(shtpData[1]);
		}
	}
	
	return(0);
}

//Given a register value and a Q point, convert to float
//See https://en.wikipedia.org/wiki/Q_(number_format)
float BNO080::qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{
  float qFloat = fixedPointValue;
  qFloat *= pow(2, qPoint * -1);
  return (qFloat);
}

//Sends the packet to enable the rotation vector
void BNO080::enableRotationVector(uint16_t timeBetweenReports)
{
  setFeatureCommand(SENSOR_REPORTID_ROTATION_VECTOR, timeBetweenReports);
}


//Sends the packet to enable the rotation vector
void BNO080::enableGameRotationVector(uint16_t timeBetweenReports)
{
  setFeatureCommand(SENSOR_REPORTID_GAME_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the accelerometer
void BNO080::enableAccelerometer(uint16_t timeBetweenReports)
{
  setFeatureCommand(SENSOR_REPORTID_ACCELEROMETER, timeBetweenReports);
}

//Sends the packet to enable the accelerometer
void BNO080::enableLinearAccelerometer(uint16_t timeBetweenReports)
{
  setFeatureCommand(SENSOR_REPORTID_LINEAR_ACCELERATION, timeBetweenReports);
}

//Sends the packet to enable the gyro
void BNO080::enableGyro(uint16_t timeBetweenReports)
{
  setFeatureCommand(SENSOR_REPORTID_GYROSCOPE, timeBetweenReports);
}

//Sends the packet to enable the magnetometer
void BNO080::enableMagnetometer(uint16_t timeBetweenReports)
{
  setFeatureCommand(SENSOR_REPORTID_MAGNETIC_FIELD, timeBetweenReports);
}

//Sends the packet to enable the step counter
void BNO080::enableStepCounter(uint16_t timeBetweenReports)
{
  setFeatureCommand(SENSOR_REPORTID_STEP_COUNTER, timeBetweenReports);
}

//Sends the packet to enable the Stability Classifier
void BNO080::enableStabilityClassifier(uint16_t timeBetweenReports)
{
  setFeatureCommand(SENSOR_REPORTID_STABILITY_CLASSIFIER, timeBetweenReports);
}

//Sends the packet to enable the various activity classifiers
void BNO080::enableActivityClassifier(uint16_t timeBetweenReports, uint32_t activitiesToEnable, uint8_t (&activityConfidences)[9])
{
	_activityConfidences = activityConfidences; //Store pointer to array	

	setFeatureCommand(SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER, timeBetweenReports, activitiesToEnable);
}

//Sends the commands to begin calibration of the accelerometer
void BNO080::calibrateAccelerometer()
{
	sendCalibrateCommand(CALIBRATE_ACCEL);
}

//Sends the commands to begin calibration of the gyro
void BNO080::calibrateGyro()
{
	sendCalibrateCommand(CALIBRATE_GYRO);
}

//Sends the commands to begin calibration of the magnetometer
void BNO080::calibrateMagnetometer()
{
	sendCalibrateCommand(CALIBRATE_MAG);
}

//Sends the commands to begin calibration of the planar accelerometer
void BNO080::calibratePlanarAccelerometer()
{
	sendCalibrateCommand(CALIBRATE_PLANAR_ACCEL);
}

//See 2.2 of the Calibration Procedure document 1000-4044
void BNO080::calibrateAll()
{
	sendCalibrateCommand(CALIBRATE_ACCEL_GYRO_MAG);
}

void BNO080::endCalibration()
{
	sendCalibrateCommand(CALIBRATE_STOP); //Disables all calibrations
}


//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
void BNO080::setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports)
{
	setFeatureCommand(reportID, timeBetweenReports, 0); //No specific config
}

//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
//Also sets the specific config word. Useful for personal activity classifier
void BNO080::setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig)
{
  long microsBetweenReports = (long)timeBetweenReports * 1000L;

  shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND; //Set feature command. Reference page 55
  shtpData[1] = reportID; //Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
  shtpData[2] = 0; //Feature flags
  shtpData[3] = 0; //Change sensitivity (LSB)
  shtpData[4] = 0; //Change sensitivity (MSB)
  shtpData[5] = (microsBetweenReports >> 0) & 0xFF; //Report interval (LSB) in microseconds. 0x7A120 = 500ms
  shtpData[6] = (microsBetweenReports >> 8) & 0xFF; //Report interval
  shtpData[7] = (microsBetweenReports >> 16) & 0xFF; //Report interval
  shtpData[8] = (microsBetweenReports >> 24) & 0xFF; //Report interval (MSB)
  shtpData[9] = 0; //Batch Interval (LSB)
  shtpData[10] = 0; //Batch Interval
  shtpData[11] = 0; //Batch Interval
  shtpData[12] = 0; //Batch Interval (MSB)
  shtpData[13] = (specificConfig >> 0) & 0xFF; //Sensor-specific config (LSB)
  shtpData[14] = (specificConfig >> 8) & 0xFF; //Sensor-specific config
  shtpData[15] = (specificConfig >> 16) & 0xFF; //Sensor-specific config
  shtpData[16] = (specificConfig >> 24) & 0xFF; //Sensor-specific config (MSB)

  //Transmit packet on channel 2, 17 bytes
  sendPacket(CHANNEL_CONTROL, 17);
}

//Tell the sensor to do a command
//See 6.3.8 page 41, Command request
//The caller is expected to set P0 through P8 prior to calling
void BNO080::sendCommand(uint8_t command)
{
  shtpData[0] = SHTP_REPORT_COMMAND_REQUEST; //Command Request
  shtpData[1] = commandSequenceNumber++; //Increments automatically each function call
  shtpData[2] = command; //Command
  
  //Caller must set these
  /*shtpData[3] = 0; //P0
  shtpData[4] = 0; //P1
  shtpData[5] = 0; //P2
  shtpData[6] = 0;
  shtpData[7] = 0;
  shtpData[8] = 0;
  shtpData[9] = 0;
  shtpData[10] = 0;
  shtpData[11] = 0;*/

  //Transmit packet on channel 2, 12 bytes
  sendPacket(CHANNEL_CONTROL, 12);
}

//This tells the BNO080 to begin calibrating
//See page 50 of reference manual and the 1000-4044 calibration doc
void BNO080::sendCalibrateCommand(uint8_t thingToCalibrate)
{
	/*shtpData[3] = 0; //P0 - Accel Cal Enable
	shtpData[4] = 0; //P1 - Gyro Cal Enable
	shtpData[5] = 0; //P2 - Mag Cal Enable
	shtpData[6] = 0; //P3 - Subcommand 0x00
	shtpData[7] = 0; //P4 - Planar Accel Cal Enable
	shtpData[8] = 0; //P5 - Reserved
	shtpData[9] = 0; //P6 - Reserved
	shtpData[10] = 0; //P7 - Reserved
	shtpData[11] = 0; //P8 - Reserved*/
	
	for(uint8_t x = 3 ; x < 12 ; x++) //Clear this section of the shtpData array
		shtpData[x] = 0;
	
	if(thingToCalibrate == CALIBRATE_ACCEL) shtpData[3] = 1;
	else if(thingToCalibrate == CALIBRATE_GYRO) shtpData[4] = 1;
	else if(thingToCalibrate == CALIBRATE_MAG) shtpData[5] = 1;
	else if(thingToCalibrate == CALIBRATE_PLANAR_ACCEL) shtpData[7] = 1;
	else if(thingToCalibrate == CALIBRATE_ACCEL_GYRO_MAG)
	{
		shtpData[3] = 1;
		shtpData[4] = 1;
		shtpData[5] = 1;
	}
	else if(thingToCalibrate == CALIBRATE_STOP) ; //Do nothing, bytes are set to zero

	//Using this shtpData packet, send a command
	sendCommand(COMMAND_ME_CALIBRATE);
}

//This tells the BNO080 to save the Dynamic Calibration Data (DCD) to flash
//See page 49 of reference manual and the 1000-4044 calibration doc
void BNO080::saveCalibration()
{
	/*shtpData[3] = 0; //P0 - Reserved
	shtpData[4] = 0; //P1 - Reserved
	shtpData[5] = 0; //P2 - Reserved
	shtpData[6] = 0; //P3 - Reserved
	shtpData[7] = 0; //P4 - Reserved
	shtpData[8] = 0; //P5 - Reserved
	shtpData[9] = 0; //P6 - Reserved
	shtpData[10] = 0; //P7 - Reserved
	shtpData[11] = 0; //P8 - Reserved*/
	
	for(uint8_t x = 3 ; x < 12 ; x++) //Clear this section of the shtpData array
		shtpData[x] = 0;

	//Using this shtpData packet, send a command
	sendCommand(COMMAND_DCD); //Save DCD command
}

//Wait a certain time for incoming I2C bytes before giving up
//Returns false if failed
boolean BNO080::waitForI2C()
{
  for (uint8_t counter = 0 ; counter < 100 ; counter++) //Don't got more than 255
  {
    if (_i2cPort->available() > 0) return (true);
    delay(1);
  }

  if(_printDebug == true) _debugPort->println(F("I2C timeout"));
  return (false);
}

//Check to see if there is any new data available
//Read the contents of the incoming packet into the shtpData array
boolean BNO080::receivePacket(void)
{
  _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)4); //Ask for four bytes to find out how much data we need to read
  if (waitForI2C() == false) return (false); //Error

  //Get the first four bytes, aka the packet header
  uint8_t packetLSB = _i2cPort->read();
  uint8_t packetMSB = _i2cPort->read();
  uint8_t channelNumber = _i2cPort->read();
  uint8_t sequenceNumber = _i2cPort->read(); //Not sure if we need to store this or not

  //Store the header info.
  shtpHeader[0] = packetLSB;
  shtpHeader[1] = packetMSB;
  shtpHeader[2] = channelNumber;
  shtpHeader[3] = sequenceNumber;

  //Calculate the number of data bytes in this packet
  int16_t dataLength = ((uint16_t)packetMSB << 8 | packetLSB);
  dataLength &= ~(1 << 15); //Clear the MSbit.
  //This bit indicates if this package is a continuation of the last. Ignore it for now.
  //TODO catch this as an error and exit
  if (dataLength == 0)
  {
    //Packet is empty
    return (false); //All done
  }
  dataLength -= 4; //Remove the header bytes from the data count

  getData(dataLength);

  return (true); //We're done!
}

//Sends multiple requests to sensor until all data bytes are received from sensor
//The shtpData buffer has max capacity of MAX_PACKET_SIZE. Any bytes over this amount will be lost.
//Arduino I2C read limit is 32 bytes. Header is 4 bytes, so max data we can read per interation is 28 bytes
boolean BNO080::getData(uint16_t bytesRemaining)
{
  uint16_t dataSpot = 0; //Start at the beginning of shtpData array

  //Setup a series of chunked 32 byte reads
  while (bytesRemaining > 0)
  {
    uint16_t numberOfBytesToRead = bytesRemaining;
    if (numberOfBytesToRead > (I2C_BUFFER_LENGTH-4)) numberOfBytesToRead = (I2C_BUFFER_LENGTH-4);

    _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)(numberOfBytesToRead + 4));
    if (waitForI2C() == false) return (0); //Error

    //The first four bytes are header bytes and are throw away
    _i2cPort->read();
    _i2cPort->read();
    _i2cPort->read();
    _i2cPort->read();

    for (uint8_t x = 0 ; x < numberOfBytesToRead ; x++)
    {
      uint8_t incoming = _i2cPort->read();
      if (dataSpot < MAX_PACKET_SIZE)
      {
        shtpData[dataSpot++] = incoming; //Store data into the shtpData array
      }
      else
      {
        //Do nothing with the data
      }
    }

    bytesRemaining -= numberOfBytesToRead;
  }
  return (true); //Done!
}

//Given the data packet, send the header then the data
//Returns false if sensor does not ACK
//TODO - Arduino has a max 32 byte send. Break sending into multi packets if needed.
boolean BNO080::sendPacket(uint8_t channelNumber, uint8_t dataLength)
{
  uint8_t packetLength = dataLength + 4; //Add four bytes for the header
  //if(packetLength > I2C_BUFFER_LENGTH) return(false); //You are trying to send too much. Break into smaller packets.

  _i2cPort->beginTransmission(_deviceAddress);

  //Send the 4 byte packet header
  _i2cPort->write(packetLength & 0xFF); //Packet length LSB
  _i2cPort->write(packetLength >> 8); //Packet length MSB
  _i2cPort->write(channelNumber); //Channel number
  _i2cPort->write(sequenceNumber[channelNumber]++); //Send the sequence number, increments with each packet sent, different counter for each channel

  //Send the user's data packet
  for (uint8_t i = 0 ; i < dataLength ; i++)
  {
    _i2cPort->write(shtpData[i]);
  }
  if (_i2cPort->endTransmission() != 0)
  {
    return (false);
  }

  return (true);
}

//Pretty prints the contents of the current shtp header and data packets
void BNO080::printPacket(void)
{
	if(_printDebug == true)
	{
		uint16_t packetLength = (uint16_t)shtpHeader[1] << 8 | shtpHeader[0];

		//Print the four byte header
		_debugPort->print(F("Header:"));
		for(uint8_t x = 0 ; x < 4 ; x++)
		{
			_debugPort->print(F(" "));
			if(shtpHeader[x] < 0x10) _debugPort->print(F("0"));
			_debugPort->print(shtpHeader[x], HEX);
		}

		uint8_t printLength = packetLength - 4;
		if(printLength > 40) printLength = 40; //Artificial limit. We don't want the phone book.

		_debugPort->print(F(" Body:"));
		for(uint8_t x = 0 ; x < printLength ; x++)
		{
			_debugPort->print(F(" "));
			if(shtpData[x] < 0x10) _debugPort->print(F("0"));
			_debugPort->print(shtpData[x], HEX);
		}

		if (packetLength & 1 << 15)
		{
			_debugPort->println(F(" [Continued packet] "));
			packetLength &= ~(1 << 15);
		}

		_debugPort->print(F(" Length:"));
		_debugPort->print (packetLength);

		_debugPort->print(F(" Channel:"));
		if (shtpHeader[2] == 0) _debugPort->print(F("Command"));
		else if (shtpHeader[2] == 1) _debugPort->print(F("Executable"));
		else if (shtpHeader[2] == 2) _debugPort->print(F("Control"));
		else if (shtpHeader[2] == 3) _debugPort->print(F("Sensor-report"));
		else if (shtpHeader[2] == 4) _debugPort->print(F("Wake-report"));
		else if (shtpHeader[2] == 5) _debugPort->print(F("Gyro-vector"));
		else _debugPort->print(shtpHeader[2]);
		
		_debugPort->println();
	}

}

/* 
  yaw   =  atan2((q1 * q2 + q0 * q3), ((q0 * q0 + q1 * q1) - 0.5f));   
  pitch = -asin(2.0f * (q1 * q3 - q0 * q2));
  roll  =  atan2((q0 * q1 + q2 * q3), ((q0 * q0 + q3 * q3) - 0.5f));
*/
 
