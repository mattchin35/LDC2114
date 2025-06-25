#include <Wire.h>
#include "FDC2214_fast.h"

#define CHANNEL_COUNT 4

FDC2214Fast sensor(FDC2214_I2C_ADDR_0); // Use FDC2214_I2C_ADDR_1 
unsigned long data[4];  // raw data output
uint32_t* dataPtr; // pointer for data address
int buttonState[4] = {0,0,0,0};

int thresholdStep = 200000;
int hysteresis = 100000;
unsigned long maxval[4] = {0,0,0,0};
unsigned long threshold[4] = {0, 0, 0, 0};

int nBaseline = 5;
int baselineL[5] = {0, 0, 0, 0, 0};
int baselineR[5] = {0, 0, 0, 0, 0};
bool trackBaselineBelowThreshold = true;

const int OUT_L_PIN = 10;
const int OUT_R_PIN = 11;

unsigned long curTime = millis();
unsigned long lastBaselineTime = millis();
unsigned long debounce = 25;


int min(int a, int b) {
  return a < b ? a : b;
}

int min_element(int* arr, int arrayLength) {
  int minval = 1e10;
  for (int i=0; i<arrayLength; i++) {
    minval = min(minval, arr[i]);
  }
  return minval;
}

int max(int a, int b) {
    return a > b ? a : b;
}

int max_element(int* arr, int arrayLength) {
  int maxval = 0;
    for (int i=0; i<arrayLength; i++) {
        maxval = max(maxval, arr[i]);
    }
    return maxval;
}


void updateBaseline(uint32_t (&capacitance)[CHANNEL_COUNT]) {
  if (trackBaselineBelowThreshold) {
    for (int i=0; i<CHANNEL_COUNT; i++) {
      maxval[i] = max(maxval[i], capacitance[i]);
  }} else {
    for (int i=0; i<CHANNEL_COUNT; i++) {
        if (capacitance[i] < threshold[i]) {
          maxval[i] = max(maxval[i], capacitance[i]);
  }}}

  if (millis() - lastBaselineTime >= 1000) {
      for (int i=0; i<nBaseline-1; i++) {
      baselineL[i] = baselineL[i+1];
      baselineR[i] = baselineR[i+1];
      }
      baselineL[nBaseline-1] = maxval[0];
      baselineR[nBaseline-1] = maxval[1];

      threshold[0] = max_element(baselineL, nBaseline) - thresholdStep;
      threshold[1] = max_element(baselineR, nBaseline) - thresholdStep;
      maxval[0] = 0; maxval[1] = 0;
      lastBaselineTime = millis();
  }
}

void updateChannel(unsigned long (&capacitance)[CHANNEL_COUNT], int (&buttonState)[CHANNEL_COUNT], uint8_t channelNum, int digitalPin) {
  if (channelNum >= CHANNEL_COUNT) {
      Serial.println("Invalid channel number");
      return;
  }

  // Threshold crossings: note that for the FDC2214, the capacitance value is inversely proportional to the touch
  // new touch alert
  if ((capacitance[channelNum] <= threshold[channelNum]) && !buttonState[channelNum]) {
    Serial.print("Channel "); Serial.print(channelNum); Serial.println(" touched");
    buttonState[channelNum] = 1;
    digitalWrite(digitalPin, HIGH);
  }

  // end touch alert
  if ((capacitance[channelNum] >= threshold[channelNum] + hysteresis) && buttonState[channelNum])  {
    Serial.print("Channel "); Serial.print(channelNum); Serial.println(" released");
    buttonState[channelNum] = 0;
    digitalWrite(digitalPin, LOW);
  }
}

void setup() {
  Wire.begin();
  // Wire.setClock(400000L);
  
  Serial.begin(38400);  // 115200);
  Serial.println("Starting FDC2214");
  
  // Start FDC2214 with 4 channels init
  bool capOk = sensor.begin();  // deglitch at 10MHz
  if (capOk) {
    Serial.println("Sensor OK");  
  } else { 
    Serial.println("Sensor Fail");  
  }
}

void loop() {
  curTime = millis();
  dataPtr = sensor.readAllData();  // pointer for array with output values
  for (int i=0; i<CHANNEL_COUNT; i++) { // for each channel
    data[i] = dataPtr[i];
  }

  updateChannel(data, buttonState, 0, OUT_L_PIN);
  updateChannel(data, buttonState, 1, OUT_R_PIN);

  Serial.print("Data0:"); Serial.println(data[0]);  
  Serial.print("Data1:"); Serial.println(data[1]);  


  // loop timing
  // Serial.print("ElapsedTime "); Serial.println(millis() - curTime);
  while (millis() - curTime < debounce);
}
