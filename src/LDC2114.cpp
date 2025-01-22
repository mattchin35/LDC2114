#include "Arduino.h"
#include <Wire.h>
#include "LDC2114.h"


LDC2114::LDC2114(uint8_t i2cAddress) {
	_i2caddr = i2cAddress;
    // _i2caddr = I2C_ADDR_0;
}

// Checking for chip ID, if OK, calls chip init
boolean LDC2114::begin() {
    Wire.begin();

    int devId = read8LDC(LDC2214_DEVICE_ID_LSB);
    if (devId != 0x00) {
        return false;
    }

    // loadSettings();
    return true;
}


// Internal routine to set configuration settings
void LDC2114::loadSettings() {
	// setup the following:
	// 1. activate ch 0 and 1, deactivate ch 2 and 3
	// 2. set polarity to decrease f increases DataN; can do this later
	// 3. set NP scan LDC2114_LP_SCAN_RATE
	// 4. config settings: SENCYC, resonance freq, Rp
	// 5. Gain
	// 6. CNTSC
	// 7. LC divider
	// 8. OPOL_DPOL
	// Must set config mode, then poll ready to write, then set registers, then return config mode

	// Set config mode
	// write8LDC(LDC2114_EN, 0x00);
}


// test function. Confirm it works the same as read8LDC
uint8_t LDC2114::readDevID() {
    uint8_t data = 0;
    Wire.beginTransmission(_i2caddr);
    Wire.write(LDC2114_DEVICE_ID_LSB);
    Wire.endTransmission(false);  // restart
    Wire.requestFrom(_i2caddr, (uint8_t) 1);

    if (Wire.available() == 1) {
        data = Wire.read();
    }

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

	// optionally, discard the first reading as "stale?"
	if (timeout == 100) {
        reading = readDataNSequential(addressLSB);
        // lsbReading = read8LDC(addressLSB);
        // msbReading = read8LDC(addressMSB);

        // reading = (uint16_t)(read8LDC(addressMSB)) << 8;
		// reading |= read8LDC(addressLSB);
		while (timeout && !(status)) {
            status = read8LDC(LDC2114_STATUS);
            timeout--;
        }
    }
	
	if (timeout) {
        reading = readDataNSequential(addressLSB);
		// reading = (uint16_t)(read8LDC(addressMSB)) << 8;
		// reading |= read8LDC(addressLSB);
		return reading;
	} else {
		// Could not get data, chip readyness flag timeout
		return 0;
	}

	return reading;
}

uint16_t LDC2114::readDataNSequential(uint8_t lsbAddress) {
    // read a DataN channel using the LDC2114's sequential data output. 
    // reads the LSB first, then the MSB by requesting 2 bytes from the LSB.
	uint16_t data = 0;

    Wire.beginTransmission(_i2caddr);
    Wire.write(lsbAddress);
    Wire.endTransmission(false);  // restart

    Wire.requestFrom(_i2caddr, (uint8_t) 2);

    if (Wire.available() == 2) {
        lsbData = Wire.read();
        msbData = Wire.read();
        data = (uint16_t)msbData << 8;
        data |= lsbData;
    }

    return data;
}

void LDC2114::readOutput(uint8_t outputAddress, bool& out0, bool& out1, bool& out2, bool& out3) {
    // read the output register, which contains the status of the 4 channels
	uint8_t = data = read8LDC(outputAddress);
    out0 = bool (data & 0x01);
    out1 = bool (data & 0x02);
    out2 = bool (data & 0x04);
    out3 = bool (data & 0x08);
}

uint16_t LDC2114::readAllData(uint8_t lsbAddress) {
    // read Data0-3 channels using the LDC2114's sequential data output. 
    // reads the LSB first, then the MSB by requesting 2 bytes from the LSB.
	uint16_t data = 0;

    Wire.beginTransmission(_i2caddr);
    Wire.write(lsbAddress);
    Wire.endTransmission(false); // restart

    Wire.requestFrom(_i2caddr, (uint8_t) 2);

    if (Wire.available() == 2) {
        lsbData = Wire.read();
        msbData = Wire.read();
        data = (uint16_t)msbData << 8;
        data |= lsbData;
    }

    return data;
}

uint8_t get_bit(uint8_t num, uint8_t position)
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
