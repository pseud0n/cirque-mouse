// Copyright (c) 2018 Cirque Corp. Restrictions apply. See: www.cirque.com/sw-license

#include <SPI.h>
#include <Mouse.h>

// ___ Using a Cirque TM0XX0XX w/ Curved Overlay and Arduino ___
// This demonstration application is built to work with a Teensy 3.1/3.2 but it can easily be adapted to
// work with Arduino-based systems.
// When using with DK000013 development kit, connect sensor to the FFC connector
// labeled 'Sensor0'.
// This application connects to a TM0XX0XX circular touch pad via SPI. To verify that your touch pad is configured
// for SPI-mode, make sure that R1 is populated with a 470k resistor (or whichever resistor connects pins 24 & 25 of the 1CA027 IC).
// The pad is configured for Absolute mode tracking.  Touch data is sent in text format over USB CDC to
// the host PC.  You can open a terminal window on the PC to the USB CDC port and see X, Y, and Z data
// fill the window when you touch the sensor. Tools->Serial Monitor can be used to view touch data.
// NOTE: all config values applied in this sample are meant for a module using REXT = 976kOhm

//  Pinnacle TM0XX0XX with Arduino
//  Hardware Interface
//  GND
//  +3.3V
//  SCK = Pin 13
//  MISO = Pin 12
//  MOSI = Pin 11
//  SS = Pin 8
//  DR = Pin 7

// Hardware pin-number labels
#define SCK_PIN 8
#define DIN_PIN 9
#define DOUT_PIN 10
#define CS_PIN 7

#define LED_0 21

// Masks for Cirque Register Access Protocol (RAP)
#define WRITE_MASK  0x80
#define READ_MASK   0xA0

// Register config values for this demo
#define SYSCONFIG_1   0x00
#define FEEDCONFIG_1  0x03
#define FEEDCONFIG_2  0x1F
#define Z_IDLE_COUNT  0x05

// Coordinate scaling values
#define PINNACLE_XMAX     2047    // max value Pinnacle can report for X
#define PINNACLE_YMAX     1535    // max value Pinnacle can report for Y
#define PINNACLE_X_LOWER  127     // min "reachable" X value
#define PINNACLE_X_UPPER  1919    // max "reachable" X value
#define PINNACLE_Y_LOWER  63      // min "reachable" Y value
#define PINNACLE_Y_UPPER  1471    // max "reachable" Y value
#define PINNACLE_X_RANGE  (PINNACLE_X_UPPER-PINNACLE_X_LOWER)
#define PINNACLE_Y_RANGE  (PINNACLE_Y_UPPER-PINNACLE_Y_LOWER)
#define ZONESCALE 256   // divisor for reducing x,y values to an array index for the LUT
#define ROWS_Y ((PINNACLE_YMAX + 1) / ZONESCALE)
#define COLS_X ((PINNACLE_XMAX + 1) / ZONESCALE)

// ADC-attenuation settings (held in BIT_7 and BIT_6)
// 1X = most sensitive, 4X = least sensitive
#define ADC_ATTENUATE_1X   0x00
#define ADC_ATTENUATE_2X   0x40
#define ADC_ATTENUATE_3X   0x80
#define ADC_ATTENUATE_4X   0xC0

// Convenient way to store and access measurements
//struct _absData
typedef struct {
  uint16_t xValue;
  uint16_t yValue;
  uint16_t zValue;
  uint8_t buttonFlags;
  bool touchDown;
  bool hovering;
} absData_t;

void foo(absData_t d) {}

absData_t touchData;

//const uint16_t ZONESCALE = 256;
//const uint16_t ROWS_Y = 6;
//const uint16_t COLS_X = 8;

// These values require tuning for optimal touch-response
// Each element represents the Z-value below which is considered "hovering" in that XY region of the sensor.
// The values present are not guaranteed to work for all HW configurations.
const uint8_t ZVALUE_MAP[ROWS_Y][COLS_X] =
{
  {0, 0,  0,  0,  0,  0, 0, 0},
  {0, 2,  3,  5,  5,  3, 2, 0},
  {0, 3,  5, 15, 15,  5, 2, 0},
  {0, 3,  5, 15, 15,  5, 3, 0},
  {0, 2,  3,  5,  5,  3, 2, 0},
  {0, 0,  0,  0,  0,  0, 0, 0},
};

int lastX, lastY;
int slowedX, slowedY;
int xDiff, yDiff;
float xDiffAcc, yDiffAcc;
//int lastXDiff, lastYDiff;
float xMotion, yMotion;
int touchCycles = 1;

#define LENGTH 5

struct Coord {
  int x = 0;
  int y = 0;

  void reset() {
    x = y = 0;
  }
};

struct PosBuffer {
  int size = 0; // For integer division without cast
  unsigned int start = 0;
  struct Coord coords_buffer[LENGTH];

  int last() {
    return (start + size) % LENGTH;
  }

  void reset() {
    start = size = 0;
  }

  void enqueue(int x, int y) {
    Coord *newValue = &coords_buffer[last()];
    newValue->x = x;
    newValue->y = y;
  
    if (size < LENGTH)
      ++size;
    else
      start = (start + 1) % LENGTH;
  }
  
  void dequeue() {
    if (size != 0) {
      start = (start + 1) % LENGTH;
      --size;
    }
  }
  
  Coord mean() {
    Coord result = {0, 0};
    //if (size < MIN_MEAN_LENGTH)
      //return result;
    int i = start;
    int zeroCountX = 0, zeroCountY = 0;
    do {
      result.x += coords_buffer[i].x;
      if (coords_buffer[i].x == 0) ++zeroCountX;
      result.y += coords_buffer[i].y;
      if (coords_buffer[i].y == 0) ++zeroCountY;
      i = (i + 1) % LENGTH;
    } while (i != last());
    if (zeroCountX < size)
      result.x /= (size - zeroCountX);
    if (zeroCountY < size)
      result.y /= (size - zeroCountY);
    return result;
  }
};

PosBuffer pb;

#define GET_X (touchData.yValue)
#define GET_Y (-touchData.xValue)
#define NO_MOVEMENT 10
#define MAX_DIFF 100
#define FAST_MOVEMENT 4
#define CLICK_MAX_TIME 10

int signOf(int x) {
  return (x > 0) - (x < 0);
}

float slow(int x) {
  if (x == 0) return 0;
  int sign = signOf(x);
  int ax = abs(x);
  return (ax * sqrt(sqrt(ax))) * sign / 3;
  
  x = abs(x);
  if (x >= 3)
    x /= 3;
  else
    x = 1;
  
  return x * sign;
}

int getSpeed(int dx, int dy) {
  return sqrt(dx*dx + dy*dy);
}

int applyFriction(float x) {
  if (x == 0)
    return 0;
  int sign = signOf(x);
  x = abs(x);
  x *= 0.97;
  return x * sign;
}

int maxAbs(int a, int b) {
  if (abs(a) > abs(b))
    return a;
  return b;
}

#define MOMENTUM_ROUND 20
#define MOMENTUM_MAX 30

int getMomentum(int diff) {
  if (diff == 0)
    return 0;
  //int result = (diff + (MOMENTUM_ROUND / 2)) / MOMENTUM_ROUND * 10;
  int sign = signOf(diff);
  diff = abs(diff);
  if (diff > 50)
    return 25 * sign;
  if (diff > 20)
    return 20 * sign;  
  return sign * diff * 0.3;
  /*
  int result = diff * 0.3;
  if (result > MOMENTUM_MAX)
    return MOMENTUM_MAX;
  if (result < -MOMENTUM_MAX)
    return -MOMENTUM_MAX;
  return result;
  */
}

// For right hand
int s_btn = 4, l_btn = 3, r_btn = 6, m_btn = 5;

void switchSides() {
  s_btn, m_btn = m_btn, s_btn;
  l_btn, r_btn = r_btn, l_btn;
}

void flash() {
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
}

// setup() gets called once at power-up, sets up serial debug output and Cirque's Pinnacle ASIC.
void setup() {
  Serial.begin(115200);
  //while(!Serial);
  delay(750);

  // These functions are required for use with thick overlays (curved)

  

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(l_btn, INPUT_PULLUP);
  pinMode(r_btn, INPUT_PULLUP);
  pinMode(m_btn, INPUT_PULLUP);
  pinMode(s_btn, INPUT_PULLUP);

  Pinnacle_Init();

  // These functions are required for use with thick overlays (curved)

  
  setAdcAttenuation(ADC_ATTENUATE_2X);
  tuneEdgeSensitivity();

  //Serial.println();
  Serial.println("X\tY\tZ\tBtn\tData");
  Pinnacle_EnableFeed(true);
}

int scrollCount = 0;
#define SCROLL_CLICK_MAX_GAP 30
int scrollTimeDiff = SCROLL_CLICK_MAX_GAP;
bool alreadyIncremented = false;

void moveCursor() {
  alreadyIncremented = false;
  //delay(10);
  //digitalWrite(LED_BUILTIN, LOW);
  //delay(10);
  //while (touchData.yValue == 0 || touchData.xValue == 0)   
  Pinnacle_GetAbsolute(&touchData);
  Pinnacle_CheckValidTouch(&touchData);     // Checks for "hover" caused by curved overlays
  
  ScaleData(&touchData, 1024, 1024);      // Scale coordinates to arbitrary X, Y resolution
  
  
  if (touchData.touchDown && (touchData.zValue > (touchCycles > 0 ? 17 : 10))) {
    scrollCount = 0;
    if (touchCycles == 0) {
      xDiff = yDiff = xDiffAcc = yDiffAcc = 0;
    } else {
      //lastXDiff = xDiff;
      //lastYDiff = yDiff;
      xDiff = GET_X - lastX;
      yDiff = GET_Y - lastY;
      if (abs(yDiff) > MAX_DIFF || abs(xDiff) > MAX_DIFF)
        return; // Try immediately, interference
      if (abs(yDiff) > FAST_MOVEMENT || abs(xDiff) > FAST_MOVEMENT)
        touchCycles = NO_MOVEMENT; // Ignore any wait
      if (touchCycles < NO_MOVEMENT) {
        if (abs(yDiff) <= 2)
          yDiff = 0;
        if (abs(xDiff) <= 2)
          xDiff = 0;
      }
    }
  
    if (touchCycles < NO_MOVEMENT) {
      ++touchCycles;
    }
    
    lastX = GET_X;
    lastY = GET_Y;

    xDiffAcc += slow(xDiff);
    yDiffAcc += slow(yDiff);
      
    //Serial.print(GET_X);
    //Serial.print('\t');
    //Serial.print(GET_Y);
    //Serial.print('\t');
    Serial.print(xDiffAcc);
    Serial.print('\t');
    Serial.print(yDiffAcc);
    Serial.print('\t');
    /*Serial.print(lastXDiff);
    Serial.print('\t');
    Serial.print(lastYDiff);
    Serial.print('\t');*/
    Serial.print(touchData.zValue);
    Serial.print('\t');
    Serial.print(touchCycles);
    Serial.print('\t');
    if(Pinnacle_zIdlePacket(&touchData))
    {
      Serial.println("liftoff");
    }
    else if(touchData.hovering)
    {
      Serial.println("hovering");
    }
    else
    {
      Serial.println("valid");
    }
    //Serial.println();
    //Mouse.move(slow(xDiff), slow(yDiff));
    Mouse.move((int)xDiffAcc, (int)yDiffAcc);

    xDiffAcc -= int(xDiffAcc);
    yDiffAcc -= int(yDiffAcc);
    
  } else if (touchData.zValue == 0) {
    //Serial.println("Not touched");
    if (scrollTimeDiff < SCROLL_CLICK_MAX_GAP) {
      ++scrollTimeDiff;
    } else {
      scrollCount = 0; 
    }
    if (touchCycles != 0 && touchCycles < CLICK_MAX_TIME) {
      Mouse.click();
      Serial.println("Click");
    }
  
    
  
    if (touchCycles != 0) {
      //xMotion = maxAbs(xDiff, lastXDiff) * 0.5;
      //yMotion = maxAbs(yDiff, lastYDiff) * 0.5;
      //if (xDiff * xDiff + yDiff * yDiff < 50) {
      if (1) {
        Serial.println("Too slow");
        //xMotion = yMotion = 0;
      } else {
        //Coord mean = pb.mean();
        
        xMotion = getMomentum(xDiff);
        yMotion = getMomentum(yDiff);
        //Serial.print(mean.x);
        //Serial.print("\t");
        //Serial.print(mean.y);
        //Serial.print("\t");
        Serial.print(xMotion);
        Serial.print("\t");
        Serial.println(yMotion);
        
      }
      //pb.reset();
    }
  
    xMotion = applyFriction(xMotion);
    yMotion = applyFriction(yMotion);
    /*
    Serial.print(xMotion);
    Serial.print("\t");
    Serial.println(yMotion);
    */
    Mouse.move(xMotion, yMotion);
    
    touchCycles = 0;
  }
}

void scrollCursor() {
  //scrollTimeDiff = 0;
  if (not alreadyIncremented) { // First iteration
    alreadyIncremented = true;
    Serial.print("Gap at ");
    Serial.println(scrollTimeDiff);
    if (scrollTimeDiff < SCROLL_CLICK_MAX_GAP || scrollCount == 0) {
      ++scrollCount; 
      Serial.print("Scroll count at ");
      Serial.println(scrollCount);
      if (scrollCount == 3) {
        scrollCount = 0;
        switchSides();
        Serial.print("Switched sides");
        flash();
      }
      //scrollTimeDiff = 0;//SCROLL_CLICK_MAX_GAP;
    } else {
      // too slow!
      //Serial.println("Set to 0");
      scrollCount = 0;
    }
    scrollTimeDiff = 0;
  }
  if (scrollTimeDiff < SCROLL_CLICK_MAX_GAP) {
    ++scrollTimeDiff;
  } else {
    scrollCount = 0;
  }
  Pinnacle_GetAbsolute(&touchData);
  Pinnacle_CheckValidTouch(&touchData);     // Checks for "hover" caused by curved overlays
  
  ScaleData(&touchData, 1024, 1024);      // Scale coordinates to arbitrary X, Y resolution
  
  
  if (touchData.touchDown && touchData.zValue > 20) {
    scrollCount = 0;
    if (touchCycles == 0) {
      xDiff = yDiff = xDiffAcc = yDiffAcc = 0;
      touchCycles = NO_MOVEMENT;
    } else {
      //Serial.println("Scroll");
      
      xDiff = GET_X - lastX;
      yDiff = GET_Y - lastY;
      xDiffAcc += xDiff / 15.0f;
      yDiffAcc += yDiff / 15.0f;
      Serial.println(yDiffAcc);
      //int modified = signOf(yDiff) * sqrt(abs(yDiff));
      //Serial.println(modified);
      //Mouse.move(0, 0, -signOf(yDiff) * sqrt(abs(yDiff)));
      Mouse.move(0, 0, -(int)yDiffAcc);

      xDiffAcc -= int(xDiffAcc);
      yDiffAcc -= int(yDiffAcc);
    }
    lastX = GET_X;
    lastY = GET_Y;
  } else {
    touchCycles = 0;
  }
}

void loop() {
    /*
    if (digitalRead(L_BTN)) Mouse.release(MOUSE_LEFT); else Mouse.press(MOUSE_LEFT);
    if (digitalRead(R_BTN)) Mouse.release(MOUSE_RIGHT); else Mouse.press(MOUSE_RIGHT);
    if (digitalRead(M_BTN)) Mouse.release(MOUSE_MIDDLE); else Mouse.press(MOUSE_MIDDLE);
    */
  
    if (digitalRead(l_btn)) {
      Mouse.release(MOUSE_LEFT);
    } else {
      Mouse.press(MOUSE_LEFT);
    }
    if (digitalRead(r_btn)) {
      Mouse.release(MOUSE_RIGHT);
    } else {
      Mouse.press(MOUSE_RIGHT);
    }
    if (digitalRead(m_btn)) {
      Mouse.release(MOUSE_MIDDLE);
    } else {
      Mouse.press(MOUSE_MIDDLE);
    }
    if (digitalRead(s_btn)) {
      moveCursor(); 
      delay(10);
    } else {
      Serial.println("Pressed scroll");
      scrollCursor();
      delay(10);
    }
}

/*
// loop() continuously checks to see if data-ready (DR) is high. If so, reads and reports touch data to terminal.
void loop()
{
  if(true)
  {
    Pinnacle_GetAbsolute(&touchData);
    Pinnacle_CheckValidTouch(&touchData);     // Checks for "hover" caused by curved overlays

    ScaleData(&touchData, 1024, 1024);      // Scale coordinates to arbitrary X, Y resolution

    xDiff += GET_X - lastX;
    yDiff += GET_Y - lastY;
    Serial.print(touchData.xValue);
    Serial.print('\t');
    Serial.print(touchData.yValue);
    Serial.print('\t');
    Serial.print(xDiff);
    Serial.print('\t');
    Serial.print(yDiff);
    Serial.print('\t');
    Serial.print(touchData.zValue);
    Serial.print('\t');
    Serial.print(touchData.buttonFlags);
    Serial.print('\t');
    if(Pinnacle_zIdlePacket(&touchData))
    {
      Serial.println("liftoff");
    }
    else if(touchData.hovering)
    {
      Serial.println("hovering");
    }
    else
    {
      Serial.println("valid");
    }
    if (abs(xDiff) > DIFF_THRESH) {
      Mouse.move(xDiff, 0);
      xDiff = 0;
    }
    if (abs(yDiff) > DIFF_THRESH) {
      Mouse.move(0, yDiff);
      yDiff = 0;
    }
    lastX = GET_X;
    lastY = GET_Y;
    delay(1);
  }
  AssertSensorLED(touchData.touchDown);
}
*/
void Pinnacle_Init()
{
  RAP_Init();
  DeAssert_CS();
  //pinMode(DR_PIN, INPUT);

  // Host clears SW_CC flag
  Pinnacle_ClearFlags();

  // Host configures bits of registers 0x03 and 0x05
  RAP_Write(0x03, SYSCONFIG_1);
  RAP_Write(0x05, FEEDCONFIG_2);

  // Host enables preferred output mode (absolute)
  RAP_Write(0x04, FEEDCONFIG_1);

  // Host sets z-idle packet count to 5 (default is 30)
  RAP_Write(0x0A, Z_IDLE_COUNT);
  Serial.println("Pinnacle Initialized...");
}

// Reads XYZ data from Pinnacle registers 0x14 through 0x17
// Stores result in absData_t struct with xValue, yValue, and zValue members
void Pinnacle_GetAbsolute(absData_t * result)
{
  uint8_t data[6] = { 0,0,0,0,0,0 };
  RAP_ReadBytes(0x12, data, 6);

  Pinnacle_ClearFlags();

  result->buttonFlags = data[0] & 0x3F;
  result->xValue = data[2] | ((data[4] & 0x0F) << 8);
  result->yValue = data[3] | ((data[4] & 0xF0) << 4);
  result->zValue = data[5] & 0x3F;

  result->touchDown = result->xValue != 0;
}

// Checks touch data to see if it is a z-idle packet (all zeros)
bool Pinnacle_zIdlePacket(absData_t * data)
{
  return data->xValue == 0 && data->yValue == 0 && data->zValue == 0;
}

// Clears Status1 register flags (SW_CC and SW_DR)
void Pinnacle_ClearFlags()
{
  RAP_Write(0x02, 0x00);
  delayMicroseconds(50);
}

// Enables/Disables the feed
void Pinnacle_EnableFeed(bool feedEnable)
{
  uint8_t temp;

  RAP_ReadBytes(0x04, &temp, 1);  // Store contents of FeedConfig1 register

  if(feedEnable)
  {
    temp |= 0x01;                 // Set Feed Enable bit
    RAP_Write(0x04, temp);
  }
  else
  {
    temp &= ~0x01;                // Clear Feed Enable bit
    RAP_Write(0x04, temp);
  }
}


// Adjusts the feedback in the ADC, effectively attenuating the finger signal
// By default, the the signal is maximally attenuated (ADC_ATTENUATE_4X for use with thin, flat overlays
void setAdcAttenuation(uint8_t adcGain)
{
  uint8_t temp = 0x00;

  Serial.println();
  Serial.println("Setting ADC gain...");
  ERA_ReadBytes(0x0187, &temp, 1);
  temp &= 0x3F; // clear top two bits
  temp |= adcGain;
  ERA_WriteByte(0x0187, temp);
  ERA_ReadBytes(0x0187, &temp, 1);
  Serial.print("ADC gain set to:\t");
  Serial.print(temp &= 0xC0, HEX);
  switch(temp)
  {
    case ADC_ATTENUATE_1X:
      Serial.println(" (X/1)");
      break;
    case ADC_ATTENUATE_2X:
      Serial.println(" (X/2)");
      break;
    case ADC_ATTENUATE_3X:
      Serial.println(" (X/3)");
      break;
    case ADC_ATTENUATE_4X:
      Serial.println(" (X/4)");
      break;
    default:
      break;
  }
}

// Changes thresholds to improve detection of fingers
void tuneEdgeSensitivity()
{
  uint8_t temp = 0x00;

  Serial.println();
  Serial.println("Setting xAxis.WideZMin...");
  ERA_ReadBytes(0x0149, &temp, 1);
  Serial.print("Current value:\t");
  Serial.println(temp, HEX);
  ERA_WriteByte(0x0149,  0x04);
  ERA_ReadBytes(0x0149, &temp, 1);
  Serial.print("New value:\t");
  Serial.println(temp, HEX);

  Serial.println();
  Serial.println("Setting yAxis.WideZMin...");
  ERA_ReadBytes(0x0168, &temp, 1);
  Serial.print("Current value:\t");
  Serial.println(temp, HEX);
  ERA_WriteByte(0x0168,  0x03);
  ERA_ReadBytes(0x0168, &temp, 1);
  Serial.print("New value:\t");
  Serial.println(temp, HEX);
}

// This function identifies when a finger is "hovering" so your system can choose to ignore them.
// Explanation: Consider the response of the sensor to be flat across it's area. The Z-sensitivity of the sensor projects this area
// a short distance upwards above the surface of the sensor. Imagine it is a solid cylinder (wider than it is tall)
// in which a finger can be detected and tracked. Adding a curved overlay will cause a user's finger to dip deeper in the middle, and higher
// on the perimeter. If the sensitivity is tuned such that the sensing area projects to the highest part of the overlay, the lowest
// point will likely have excessive sensitivity. This means the sensor can detect a finger that isn't actually contacting the overlay in the shallower area.
// ZVALUE_MAP[][] stores a lookup table in which you can define the Z-value and XY position that is considered "hovering". Experimentation/tuning is required.
// NOTE: Z-value output decreases to 0 as you move your finger away from the sensor, and it's maximum value is 0x63 (6-bits).
void Pinnacle_CheckValidTouch(absData_t * touchData)
{
  uint32_t zone_x, zone_y;
  //eliminate hovering
  zone_x = touchData->xValue / ZONESCALE;
  zone_y = touchData->yValue / ZONESCALE;
  touchData->hovering = !(touchData->zValue > ZVALUE_MAP[zone_y][zone_x]);
}


// Reads <count> bytes from an extended register at <address> (16-bit address),
// stores values in <*data>
void ERA_ReadBytes(uint16_t address, uint8_t * data, uint16_t count)
{
  uint8_t ERAControlValue = 0xFF;

  Pinnacle_EnableFeed(false); // Disable feed

  RAP_Write(0x1C, (uint8_t)(address >> 8));     // Send upper byte of ERA address
  RAP_Write(0x1D, (uint8_t)(address & 0x00FF)); // Send lower byte of ERA address

  for(uint16_t i = 0; i < count; i++)
  {
    RAP_Write(0x1E, 0x05);  // Signal ERA-read (auto-increment) to Pinnacle

    // Wait for status register 0x1E to clear
    do
    {
      RAP_ReadBytes(0x1E, &ERAControlValue, 1);
    } while(ERAControlValue != 0x00);

    RAP_ReadBytes(0x1B, data + i, 1);

    Pinnacle_ClearFlags();
  }
}

// Writes a byte, <data>, to an extended register at <address> (16-bit address)
void ERA_WriteByte(uint16_t address, uint8_t data)
{
  uint8_t ERAControlValue = 0xFF;

  Pinnacle_EnableFeed(false); // Disable feed

  RAP_Write(0x1B, data);      // Send data byte to be written

  RAP_Write(0x1C, (uint8_t)(address >> 8));     // Upper byte of ERA address
  RAP_Write(0x1D, (uint8_t)(address & 0x00FF)); // Lower byte of ERA address

  RAP_Write(0x1E, 0x02);  // Signal an ERA-write to Pinnacle

  // Wait for status register 0x1E to clear
  do
  {
    RAP_ReadBytes(0x1E, &ERAControlValue, 1);
  } while(ERAControlValue != 0x00);

  Pinnacle_ClearFlags();
}



void RAP_Init()
{
  pinMode(CS_PIN, OUTPUT);
  SPI.begin();
}

// Reads <count> Pinnacle registers starting at <address>
void RAP_ReadBytes(byte address, byte * data, byte count)
{
  byte cmdByte = READ_MASK | address;   // Form the READ command byte

  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));

  Assert_CS();
  SPI.transfer(cmdByte);  // Signal a RAP-read operation starting at <address>
  SPI.transfer(0xFC);     // Filler byte
  SPI.transfer(0xFC);     // Filler byte
  for(byte i = 0; i < count; i++)
  {
    data[i] =  SPI.transfer(0xFC);  // Each subsequent SPI transfer gets another register's contents
  }
  DeAssert_CS();

  SPI.endTransaction();
}

// Writes single-byte <data> to <address>
void RAP_Write(byte address, byte data)
{
  byte cmdByte = WRITE_MASK | address;  // Form the WRITE command byte

  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));

  Assert_CS();
  SPI.transfer(cmdByte);  // Signal a write to register at <address>
  SPI.transfer(data);    // Send <value> to be written to register
  DeAssert_CS();

  SPI.endTransaction();
}


// Clips raw coordinates to "reachable" window of sensor
// NOTE: values outside this window can only appear as a result of noise
void ClipCoordinates(absData_t * coordinates)
{
  if(coordinates->xValue < PINNACLE_X_LOWER)
  {
    coordinates->xValue = PINNACLE_X_LOWER;
  }
  else if(coordinates->xValue > PINNACLE_X_UPPER)
  {
    coordinates->xValue = PINNACLE_X_UPPER;
  }
  if(coordinates->yValue < PINNACLE_Y_LOWER)
  {
    coordinates->yValue = PINNACLE_Y_LOWER;
  }
  else if(coordinates->yValue > PINNACLE_Y_UPPER)
  {
    coordinates->yValue = PINNACLE_Y_UPPER;
  }
}

// Scales data to desired X & Y resolution
void ScaleData(absData_t * coordinates, uint16_t xResolution, uint16_t yResolution)
{
  uint32_t xTemp = 0;
  uint32_t yTemp = 0;

  ClipCoordinates(coordinates);

  xTemp = coordinates->xValue;
  yTemp = coordinates->yValue;

  // translate coordinates to (0, 0) reference by subtracting edge-offset
  xTemp -= PINNACLE_X_LOWER;
  yTemp -= PINNACLE_Y_LOWER;

  // scale coordinates to (xResolution, yResolution) range
  coordinates->xValue = (uint16_t)(xTemp * xResolution / PINNACLE_X_RANGE);
  coordinates->yValue = (uint16_t)(yTemp * yResolution / PINNACLE_Y_RANGE);
}


void Assert_CS()
{
  digitalWrite(CS_PIN, LOW);
}

void DeAssert_CS()
{
  digitalWrite(CS_PIN, HIGH);
}

void AssertSensorLED(bool state)
{
  digitalWrite(LED_0, state);
}

bool DR_Asserted()
{
  //return digitalRead(DR_PIN);
  return true;
}
