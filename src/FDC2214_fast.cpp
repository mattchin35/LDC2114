#include "Arduino.h"
#include <Wire.h>
#include "FDC2214_fast.h"


FDC2214Fast::FDC2214Fast(uint8_t i2caddr) {
	_i2caddr = i2caddr;
    //_i2caddr = FDC2214_I2C_ADDRESS;
}

// Checking for chip ID, if OK, calls chip init
bool FDC2214Fast::begin() {
    Wire.begin();

    int devId = read16FDC(FDC2214_DEVICE_ID);
    if (devId != 0x3054) {
        if (devId != 0x3055) {
            //two valid device ids for FDC2214 0x3054 and 0x3055
            return false;
        }
    }
    loadSettings();
	return true;
}

// Internal routine to do actual chip init
void FDC2214Fast::loadSettings() {
    //  Configuration register
	//	Active channel select: b00 = ch0; b01 = ch1; b10 = ch2; b11 = ch3;
	//  | Sleep Mode: 0 - device active; 1 - device in sleep;
	//  | |Reserved, reserved, set to 1
	//  | ||Sensor Activation Mode: 0 - drive sensor with full current. 1 - drive sensor with current set by DRIVE_CURRENT_CHn 
	//  | |||Reserved, set to 1
	//  | ||||Reference clock: 0 - use internal; 1 - use external clock
	//  | |||||Reserved, set to 0
	//  | ||||||Disable interrupt. 0 - interrupt output on INTB pin; 1 - no interrupt output
	//  | |||||||High current sensor mode: 0 - 1.5mA max. 1 - > 1.5mA, not available if Autoscan is enabled
	//  | ||||||||Reserved, set to 000001
	//  | |||||||||
	//  CCS1A1R0IH000001 -> 0001 1110 1000 0001 -> 0x1E81 	ExtOsc
	//  CCS1A1R0IH000001 -> 0001 1100 1000 0001 -> 0x1C81	IntOsc

	// activate sleep mode to change settings - should be on by default at startup but just in case
	uint16_t config = read16FDC(FDC2214_CONFIG);
	config = set_bit(config, 13);  // sleep mode is bit 13
	write16FDC(FDC2214_CONFIG, config);	
	delay(100);
	
	// Set channel 0 settings //

	// Mux configuration register - active channels and autoscan sequence
	// Autoscan: 0 = single channel, selected by CONFIG.ACTIVE_CHAN
	// |Autoscan sequence. b00 for chan 1-2, b01 for chan 1-2-3, b10 for chan 1-2-3-4
	// || Reserved - must be b0001000001
	// || |         Deglitch frequency. b001 for 1 MHz, b100 for 3.3 MHz, b101 for 10 Mhz, b111 for 33 MHz
	// || |         |
    // ARR0001000001DDD -> b0000 0010 0000 1000 -> h0208
	uint16_t muxVal = 0x0208 | (1 << 15) | ((uint16_t) 0b10 << 13) | 0b101;
	write16FDC(FDC2214_MUX_CONFIG, muxVal);  // set mux config for channels

	// Settle count - cycles needed for sensor oscillations to stabilize
	uint16_t settleCount = 0x000A;  
	// write16FDC(FDC2214_SETTLECOUNT_CH0, 0x64); // 0x64 - settle count maximized, slow application
	write16FDC(FDC2214_SETTLECOUNT_CH0, settleCount); // 0x000A - settle count minimized (10), fast application

	// Reference count - clock cycles needed to support a number of effective bits
	uint16_t rCount = 0x186A;  
	write16FDC(FDC2214_RCOUNT_CH0, rCount);  // 0xFFFF is max Rcount, refernce count maximized for highest accuracy - 100 ms per 4 channel read
	// write16FDC(FDC2214_RCOUNT_CH0, 0x186A);  // calculated Rcount: 6250, 0x186A - 10 ms per 4 channel read

	// no offset - offset settings only apply to FDC2112/2114
	write16FDC(FDC2214_OFFSET_CH0, 0x0000);
	
	// Set clock dividers
	// Reserved
	// | Sensor Frequency Select. b01 = /1 = sensor freq 0.01 to 8.75MHz (differential sensor); 
	// | |                        b10 = /2 = sensor freq 0.01 to 10 (single-ended) or 5 to 10 MHz (differential)
	// | | Reserved
	// | | | Reference divider. Must be > 1. fref = fclk / this register
	// | | | |
	// 00FF00RRRRRRRRRR -> 0010000000000001 -> 0x2001
	write16FDC(FDC2214_CLOCK_DIVIDERS_CH0, 0x2001);
	
	// set drive register
	write16FDC(FDC2214_DRIVE_CH0, 0xF800);  // 0xF800 is max drive strength, can adjust empirically if needed

	// Other channels are set with same values as channel 0
	// Init chan1
	write16FDC(FDC2214_SETTLECOUNT_CH1, settleCount);
	write16FDC(FDC2214_RCOUNT_CH1, rCount);
	write16FDC(FDC2214_OFFSET_CH1, 0x0000);
	write16FDC(FDC2214_CLOCK_DIVIDERS_CH1, 0x2001);
	write16FDC(FDC2214_DRIVE_CH1, 0xF800);	
	
	// Init chan2
	write16FDC(FDC2214_SETTLECOUNT_CH2, settleCount);
	write16FDC(FDC2214_RCOUNT_CH2, rCount);
	write16FDC(FDC2214_OFFSET_CH2, 0x0000);
	write16FDC(FDC2214_CLOCK_DIVIDERS_CH2, 0x2001);
	write16FDC(FDC2214_DRIVE_CH2, 0xF800);	
		
	// Init chan3
	write16FDC(FDC2214_SETTLECOUNT_CH3, settleCount);
	write16FDC(FDC2214_RCOUNT_CH3, rCount);
	write16FDC(FDC2214_OFFSET_CH3, 0x0000);
	write16FDC(FDC2214_CLOCK_DIVIDERS_CH3, 0x2001);
	write16FDC(FDC2214_DRIVE_CH3, 0xF800);	
	
	config = clear_bit(config, 9);  // set internal oscillator as reference freq - 0b0, bit 9
	config = clear_bit(config, 11); // full current mode - 0b0, bit 11
	config = clear_bit(config, 13); // exit sleep mode - 0b0, bit 13
	write16FDC(FDC2214_CONFIG, config); 
	Serial.println("FDC2214 initialized");
	delay(100);
	
}

///**************************************************************************/
///*!
//    @brief  Given a reading calculates the sensor frequency
//*/
///**************************************************************************/
// long long NelsonsLog_FDC2214::calculateFsensor(unsigned long reading){
//    Serial.println("reading: "+ String(reading));
//    //fsensor = (CH_FIN_SEL * fref * data) / 2 ^ 28
//    //should be mega hz so can truncate to long long
//    Serial.println("FDC reading: " + String(reading));
//    unsigned long long temp;
//    temp = 1 * 40000000 * reading;
//    temp = temp / (2^28);
////    Serial.println("frequency: " + String((long)temp));
//    return temp;
//}

///**************************************************************************/
///*!
//    @brief  Given sensor frequency calculates capacitance
//*/
///**************************************************************************/
//double NelsonsLog_FDC2214::calculateCapacitance(long long fsensor){
//    //differential configuration
//    //c sensor = 1                            - (Cboard + Cparacitic)
//    //             / (L * (2*pi * fsensor)^2)
//
//    double pi = 3.14159265359;
//    double L = 18; //uH
//    double Cboard = 33; //pf
//    double Cparacitic = 3; //pf
//
//    double temp = 2 * pi * fsensor;
//    temp = temp * temp;
//
//    temp = temp / 1000000; //uH
//    temp *= L;
//
////    Serial.println("capacitance: " + String(temp));
//    return temp;
//
//}



// Gets 28bit reading for FDC2212 and FDC2214
// Takes in channel number, gives out the formatted 28 bit reading.
unsigned long FDC2214Fast::getReading28(uint8_t channel) {
    int timeout = 100;
    unsigned long reading = 0;
    uint8_t addressMSB;
	uint8_t addressLSB;
	uint8_t bitUnreadConv;
    int status = read16FDC(FDC2214_STATUS);
	switch (channel){
		case (0):
			addressMSB = FDC2214_DATA_CH0_MSB;
			addressLSB = FDC2214_DATA_CH0_LSB;
			bitUnreadConv = FDC2214_CH0_UNREADCONV;
		break;
		case (1):
			addressMSB = FDC2214_DATA_CH1_MSB;
			addressLSB = FDC2214_DATA_CH1_LSB;
			bitUnreadConv = FDC2214_CH1_UNREADCONV;
		break;
		case (2):
			addressMSB = FDC2214_DATA_CH2_MSB;
			addressLSB = FDC2214_DATA_CH2_LSB;
			bitUnreadConv = FDC2214_CH2_UNREADCONV;
		break;
		case (3):
			addressMSB = FDC2214_DATA_CH3_MSB;
			addressLSB = FDC2214_DATA_CH3_LSB;
			bitUnreadConv = FDC2214_CH3_UNREADCONV;
		break;
		default: return 0;
	}
	
	while (timeout && !(status & bitUnreadConv)) {
        status = read16FDC(FDC2214_STATUS);
        timeout--;
    }

    if (timeout) {
        //read the 28 bit result
        reading = (uint32_t)(read16FDC(addressMSB) & FDC2214_DATA_CHx_MASK_DATA) << 16;
        reading |= read16FDC(addressLSB);
        return reading;
    } else {
		// Could not get data, chip readyness flag timeout
        return 0;
    }
}

uint32_t* FDC2214Fast::readAllData() {
    // using one status read "freeze", Read Data0-3 channels 
	static uint32_t data[4];
	unsigned long reading = 0;
    uint8_t addressMSB;
	uint8_t addressLSB;

	addressMSB = FDC2214_DATA_CH0_MSB;
	addressLSB = FDC2214_DATA_CH0_LSB;
	reading = (uint32_t)(read16FDC(addressMSB) & FDC2214_DATA_CHx_MASK_DATA) << 16;
	reading |= read16FDC(addressLSB);
	data[0] = reading;

	addressMSB = FDC2214_DATA_CH1_MSB;
	addressLSB = FDC2214_DATA_CH1_LSB;
	reading = (uint32_t)(read16FDC(addressMSB) & FDC2214_DATA_CHx_MASK_DATA) << 16;
	reading |= read16FDC(addressLSB);
	data[1] = reading;

	addressMSB = FDC2214_DATA_CH2_MSB;
	addressLSB = FDC2214_DATA_CH2_LSB;
	reading = (uint32_t)(read16FDC(addressMSB) & FDC2214_DATA_CHx_MASK_DATA) << 16;
	reading |= read16FDC(addressLSB);
	data[2] = reading;

	addressMSB = FDC2214_DATA_CH3_MSB;
	addressLSB = FDC2214_DATA_CH3_LSB;
	reading = (uint32_t)(read16FDC(addressMSB) & FDC2214_DATA_CHx_MASK_DATA) << 16;
	reading |= read16FDC(addressLSB);
	data[3] = reading;

    return data;
}


///**************************************************************************/
///*!
//    @brief  Takes a reading and calculates capacitance from it
//*/
///**************************************************************************/
//double NelsonsLog_FDC2214::readCapacitance() {
//    int timeout = 100;
//    unsigned long reading = 0;
//    long long fsensor = 0;
//    int status = read16FDC(FDC2214_STATUS_REGADDR);
//    while (timeout && !(status & FDC2214_CH0_UNREADCONV)) {
////        Serial.println("status: " + String(status));
//        status = read16FDC(FDC2214_STATUS_REGADDR);
//        timeout--;
//    }
//    if (timeout) {
//        //read the 28 bit result
//        reading = read16FDC(FDC2214_DATA_CH0_REGADDR) << 16;
//        reading |= read16FDC(FDC2214_DATA_LSB_CH0_REGADDR);
//        fsensor = calculateFsensor(reading);
//        return calculateCapacitance(fsensor);
//    } else {
//        // error not reading
//        Serial.println("error reading fdc");
//        return 0;
//    }
//}


/**************************************************************************/
/*!
    @brief  Scans various gain settings until the amplitude flag is cleared.
            WARNING: Changing the gain setting will generally have an impact on the
            reading.
*/
/**************************************************************************/
//void NelsonsLog_FDC2214::setGain(void) {
//    //todo
//}
/**************************************************************************/
/*!
    @brief  I2C low level interfacing
*/
/**************************************************************************/

// Read 2 bytes from the FDC at 'address'
uint16_t FDC2214Fast::read16FDC(uint16_t address) {
    uint16_t data = 0;

    Wire.beginTransmission(_i2caddr);
    Wire.write(address);
    Wire.endTransmission(false); //restart

    Wire.requestFrom(_i2caddr, (uint8_t) 2);
    if (Wire.available() == 2) {
        data = Wire.read();
        data <<= 8;
        data |= Wire.read();
    }

    return data;
}

// write 2 bytes to FDC  
void FDC2214Fast::write16FDC(uint16_t address, uint16_t data) {
    Wire.beginTransmission(_i2caddr);
    Wire.write(address & 0xFF);
    Wire.write(data >> 8);
    Wire.write(data);
    Wire.endTransmission();
}

uint16_t FDC2214Fast::set_bit(int num, int position) {
	return num | (1 << position);
}

uint16_t FDC2214Fast::get_bit(int num, int position) {
	return 1 & (num >> position);
}

uint16_t FDC2214Fast::clear_bit(int num, int position) {
    return num & ~(1 << position);
}

uint16_t FDC2214Fast::toggle_bit(int num, int position) {
    return num ^ (1 << position);
}
