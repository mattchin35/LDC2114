#ifndef _LDC2114_H_
#define _LDC2114_H_

#if ARDUINO >= 100
#include "Arduino.h"
#define WIRE_WRITE Wire.write
#else
#include "WProgram.h"
#define WIRE_WRITE Wire.send
#endif

#if defined(__SAM3X8E__) // Arduino Due
    typedef volatile RwReg PortReg;
    typedef uint32_t PortMask;
#define HAVE_PORTREG
#elif defined(ARDUINO_ARCH_SAMD)  // Arduino Zero
// not supported
#elif defined(ESP8266) || defined(ESP32) || defined(ARDUINO_STM32_FEATHER) || defined(__arc__)
    typedef volatile uint32_t PortReg;
    typedef uint32_t PortMask;
#elif defined(__AVR__) // Arduino Uno, Mega, Nano
    typedef volatile uint8_t PortReg;
    typedef uint8_t PortMask;
#define HAVE_PORTREG
#else
// chances are its 32 bit so assume that
    typedef volatile uint32_t PortReg;
    typedef uint32_t PortMask;
#endif

#define I2C_ADDR_0   0x2A  // Use this one! 0x2B is only available for the LDC2112
#define I2C_ADDR_1   0x2B  // Do not use this with the LDC2114
// For LDC2112, address can be 0x2A (default, ADDR = Ground) or 0x2B (ADDR is high, V_DD); for LDC2114, address is 0x2A

//bitmasks
#define CH0_UNREADCONV 0x0008         //denotes unread CH0 reading in STATUS register
#define CH1_UNREADCONV 0x0004         //denotes unread CH1 reading in STATUS register
#define CH2_UNREADCONV 0x0002         //denotes unread CH2 reading in STATUS register
#define CH3_UNREADCONV 0x0001         //denotes unread CH3 reading in STATUS register


// registers - max value 256 (0xFF, unsigned 8 bit int)
#define LDC2114_STATUS                      0x00
#define LDC2114_OUT                         0x01
#define LDC2114_DATA_CH0_LSB                0x02
#define LDC2114_DATA_CH0_MSB                0x03
#define LDC2114_DATA_CH1_LSB                0x04
#define LDC2114_DATA_CH1_MSB                0x05
#define LDC2114_DATA_CH2_LSB                0x06
#define LDC2114_DATA_CH2_MSB                0x07
#define LDC2114_DATA_CH3_LSB                0x08
#define LDC2114_DATA_CH3_MSB                0x09

#define LDC2114_MANUFACTURER_ID_LSB         0xFC
#define LDC2114_MANUFACTURER_ID_MSB         0xFD
#define LDC2114_DEVICE_ID_LSB               0xFE
#define LDC2114_DEVICE_ID_MSB               0xFF

#define LDC2114_RESET                       0x0A
#define LDC2114_EN                          0x0C
#define LDC2114_NP_SCAN_RATE                0x0D
#define LDC2114_GAIN0                       0x0E
#define LDC2114_LP_SCAN_RATE                0x0F
#define LDC2114_GAIN1                       0x10
#define LDC2114_INTPOL                      0x11
#define LDC2114_GAIN2                       0x12
#define LDC2114_LP_BASE_INC                 0x13
#define LDC2114_GAIN3                       0x14
#define LDC2114_NP_BASE_INC                 0x15
#define LDC2114_BTPAUSE_MAXWIN              0x16
#define LDC2114_LC_DIVIDER                  0x17
#define LDC2114_HYST                        0x18
#define LDC2114_TWIST                       0x19
#define LDC2114_COMMON_DEFORM               0x1A
#define LDC2114_OPOL_DPOL                   0x1C
#define LDC2114_CNTSC                       0x1E
#define LDC2114_SENSOR0_CONFIG              0x20
#define LDC2114_SENSOR1_CONFIG              0x22
#define LDC2114_SENSOR2_CONFIG              0x24
#define LDC2114_SENSOR3_CONFIG              0x26
#define LDC2114_FTF1_2                      0x28      
#define LDC2114_FTF3                        0x2B     


class LDC2114 {
public:
    LDC2114(uint8_t i2cAddress);
    boolean begin();

    // test functions, if any
    uint8_t readDevID();
    unsigned long readChannelData(uint8_t channel);
    void readOutput(uint8_t outputAddress, bool& out0, bool& out1, bool& out2, bool& out3);

private:
    uint8_t _i2caddr;
    void loadSettings(uint8_t chanMask);
//    void setGain(void);
    void write8LDC(uint8_t address, uint8_t data);
    uint8_t read8LDC(uint8_t address);
    uint8_t get_bit(uint8_t num, uint8_t position);
    uint16_t readDataNSequential(uint8_t lsbAddress);
    uint16_t readAllData(uint8_t lsbAddress);  
};

#endif //include guard
