#include "Arduino.h"
#include <Wire.h>
#include "LDC2114.h"


LDC2114::LDC2114(uint8_t i2cAddress) {
	_i2caddr = i2cAddress;
    // _i2caddr = I2C_ADDR_0;
}

// Checking for chip ID, if OK, calls chip init
bool LDC2114::begin(uint8_t gain) {
    Wire.begin();
    // Wire.setClock(400000);  // try with and without this - is there a difference in performance?
    // delay(100);  // give everything some startup time, probably unnecessary

    int devId = read8LDC(LDC2114_DEVICE_ID_LSB);
    if (devId != 0x00) {
        return false;
    }

    devId = read8LDC(LDC2114_DEVICE_ID_MSB);
    if (devId != 0x20) {
        return false;
    }

    bool settings = loadSettings(gain);
    if (settings) {
        return true;
    } else {
        return false;
    }
}

uint8_t LDC2114::chipReady() {
    int status = read8LDC(LDC2114_STATUS);
    return get_bit(status, 6);
}

// Internal routine to set configuration settings
bool LDC2114::loadSettings(uint8_t gain) {
	// setup the following:
	// 1. activate ch 0 and 1, deactivate ch 2 and 3 - optional
	// 2. set OUT polarity so increased DataN means a button press; optional, can work directly with DataN
	// 3. Gain; low priority
	// 4. set NP scan LDC2114_LP_SCAN_RATE
	// 5. config settings: SENCYC, resonance freq, Rp
	// 6. CNTSC
	// 7. LC divider
	// 8. OPOL_DPOL
	// Must set config mode, then poll ready to write, then set registers, then return config mode

    // Set config mode - reset bit 0
	write8LDC(LDC2114_RESET, 0b1);
    // Poll ready to write - status bit 5
    int status = read8LDC(LDC2114_STATUS);
    if (get_bit(status, 5)) {
        // set registers
        write8LDC(LDC2114_NP_SCAN_RATE, 0b0); // b01 - 40 SPS; b00 - 80 SPS
        write8LDC(LDC2114_LC_DIVIDER, 0b1);  // default 3, 0b11

        // sensor config - set sencyc, sensor freq, Rp
        // expect sensors to have 3.3-10 MHz resonance freq, Rp < 4kOhm
        uint8_t Rp = 0b0;
        uint8_t fSensor = 0b01;
        uint8_t senCyc = 0b11111; // ranges from 0 to 31, number of sensor cycles/time to collect data over
        uint8_t sensorConfig = (Rp << 7) | (fSensor << 5) | senCyc; 
        write8LDC(LDC2114_SENSOR0_CONFIG, sensorConfig);
        write8LDC(LDC2114_SENSOR1_CONFIG, sensorConfig);
        write8LDC(LDC2114_SENSOR2_CONFIG, sensorConfig);
        write8LDC(LDC2114_SENSOR3_CONFIG, sensorConfig);

        write8LDC(LDC2114_CNTSC, 0b0);

        // output polarity and data polarity - default is 0x0F
        // bits 4-7 are output polarity (default 0), 0: active low; 1: active high
        // bits 0-3 are data polarity (default 1), 0: data decreases as Fsensor increases; 1: data increases as Fsensor increases
        write8LDC(LDC2114_OPOL_DPOL, 0b00001111);

        // set gain - default is 40, 0x28
        write8LDC(LDC2114_GAIN0, gain);
        write8LDC(LDC2114_GAIN1, gain);
        write8LDC(LDC2114_GAIN2, gain);
        write8LDC(LDC2114_GAIN3, gain);

        // return to normal mode
        write8LDC(LDC2114_RESET, 0b0);

        return true;
    } else {
        // error, not ready to write
        return false;
    }
}

// test function
uint16_t LDC2114::readDevID() {
    uint16_t data = read16LDC(LDC2114_DEVICE_ID_LSB);
    return data;
}

// read dataN channels using sequential register reading. LSB is read first, then MSB.
unsigned long LDC2114::readChannelData(uint8_t channel) {
	int status = read8LDC(LDC2114_STATUS);
	
	int timeout = 100;
	unsigned long reading = 0;
    uint8_t addressMSB;
	uint8_t addressLSB;
	switch (channel) {
		case 0:
			addressLSB = LDC2114_DATA_CH0_LSB;
			addressMSB = LDC2114_DATA_CH0_MSB;
			break;
		case 1:
			addressLSB = LDC2114_DATA_CH1_LSB;
			addressMSB = LDC2114_DATA_CH1_MSB;
			break;
		case 2:
			addressLSB = LDC2114_DATA_CH2_LSB;
			addressMSB = LDC2114_DATA_CH2_MSB;
			break;
		case 3:
			addressLSB = LDC2114_DATA_CH3_LSB;
			addressMSB = LDC2114_DATA_CH3_MSB;
			break;
		default:
			return 0;
	}

	while (timeout && !status) {
        status = read8LDC(LDC2114_STATUS);
        timeout--;
    }

	// optionally, discard the first reading as "stale?" Test functionality without it
	// if (timeout == 100) {
    //     reading = read16LDC(addressLSB);
    //     // lsbReading = read8LDC(addressLSB);
    //     // msbReading = read8LDC(addressMSB);

    //     // reading = (uint16_t)(read8LDC(addressMSB)) << 8;
	// 	// reading |= read8LDC(addressLSB);
	// 	while (timeout && !(status)) {
    //         status = read8LDC(LDC2114_STATUS);
    //         timeout--;
    //     }
    // }
	
	if (timeout) {
        reading = read16LDC(addressLSB) & 0x0FFF;  // mask the 4 MSB bits, they're reserved and empty
		return reading;
	} else {
		// Could not get data, chip readyness flag timeout
		return 0;
	}

	return reading;
}

uint8_t* LDC2114::readOutput(uint8_t outputAddress) {
    // read the output register, which contains the status of the 4 channels
	uint8_t data = read8LDC(outputAddress) & 0x0F;  // mask the 4 MSB bits, they're reserved and empty
    static uint8_t output[4];
    output[0] = get_bit(data, 0);
    output[1] = get_bit(data, 1);
    output[2] = get_bit(data, 2);
    output[3] = get_bit(data, 3);
    return output;
}

uint16_t* LDC2114::readAllData() {
    // using one status read "freeze", read the Data0-3 channels using the LDC2114's sequential data output.
	static uint16_t data[4];
    uint8_t lsbData;
    uint8_t msbData;

    Wire.beginTransmission(_i2caddr);
    Wire.write(LDC2114_DATA_CH0_LSB);
    Wire.endTransmission(false); // restart
    Wire.requestFrom(_i2caddr, (uint8_t) 8);

    for (int i = 0; i < 4; i++) {
        if (Wire.available() >= 2) {
            lsbData = Wire.read();
            msbData = Wire.read() & 0x0F;  // mask the 4 MSB bits, they're reserved and empty
            data[i] = (uint16_t)msbData << 8;
            data[i] |= lsbData;
        }
    }
    return data;
}

uint8_t LDC2114::get_bit(uint8_t num, uint8_t position)
{
	return 1 & (num >> position);
}

// Read 1 byte
uint8_t LDC2114::read8LDC(uint8_t registerAddress) {
    uint8_t data = 0;
    Wire.beginTransmission(_i2caddr);
    Wire.write(registerAddress);
    Wire.endTransmission(false);  // restart
    Wire.requestFrom(_i2caddr, (uint8_t) 1);

    if (Wire.available() == 1) {
        data = Wire.read();
    }

    return data;
}

// write 1 byte to LDC
void LDC2114::write8LDC(uint8_t address, uint8_t data) {
    Wire.beginTransmission(_i2caddr);
    Wire.write(address);
    Wire.write(data);
    Wire.endTransmission();
}

// Read 2 bytes from the LDC. Reads the LSB at 'address' and then the MSB sequentially.
uint16_t LDC2114::read16LDC(uint16_t lsbAddress) {
    uint16_t data;

    Wire.beginTransmission(_i2caddr);
    Wire.write(lsbAddress);
    Wire.endTransmission(false);  // restart

    Wire.requestFrom(_i2caddr, (uint8_t) 2);
    if (Wire.available() == 2) {
        uint8_t lsbData = Wire.read();
        uint8_t msbData = Wire.read();
        data = (uint16_t)msbData << 8;
        data |= lsbData;
    }

    return data;
}
