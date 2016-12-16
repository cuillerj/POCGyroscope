/*
   I2C communication as master
   one connection with gyroscope
   one connection with robot (robot can be polled with timer or can request token with InputRobotRequestPin
*/
#define debugOn
#include <Wire.h>
#include <LSM303.h>     // for accelerometer
//-- accelerometer and magnetometer
// powered by arduino 3.3v
LSM303 compass;
unsigned long refAccX;
float X;
float Y = 0;
int NOBeforeRotation = 0; // keep NO before rotation
int NOAfterRotation = 0;
int NOBeforeMoving = 0; // keep NO before moving straight
int NOAfterMoving = 0; // keep NO before moving straight
// compass calibration to be made
#define compassMin1 -661
#define compassMin2 -1792
#define compassMin3 -3132
#define compassMax1 +1502
#define compassMax2 +497
#define compassMax3 -2560
#define L3GD20H 0b11010111
#define L3GD20  0b11010100
#define L3G_CTRL_REG1 0x20
#define L3G_CTRL_REG2 0x21
#define L3G_CTRL_REG3 0x22
#define L3G_CTRL_REG4 0x23
#define L3G_CTRL_REG5 0x24
volatile boolean dataToRead = false;
uint8_t LOCAL_CTRL_REG[255];        // to store parameters
#include <ApeRobotSensorSubsytemDefine.h>
//int range[3] = {245, 500, 2000};
#define sensitivityDivider 1000000
float sensitivity[3] = {8.75 / sensitivityDivider, 17.5 / sensitivityDivider, 70. / sensitivityDivider};
int x;
int y;
int z;
uint8_t statusL3G;
float bias = 0;
unsigned int count = 0;
float alpha = 0;
unsigned long avg = 0;
unsigned long savTime = 0;
uint8_t inputData[256];
uint8_t receivedCount = 0x00;
boolean flagOnRequest = false;
uint8_t dataIn[33];
uint8_t pollResponseExpectedLenght = 10;
boolean req = false;
unsigned long prevPollTimer;
unsigned long prevL3GTimer;
unsigned long prevSentData;
unsigned long updateNOTimer;
boolean monitGyro = false;
boolean monitMagneto = true;
boolean statRobotRequest = 0;
uint8_t currentStatus = 0x00;
unsigned long startInterruptTime;
unsigned long interruptCount;
void setup() {
  Wire.begin();
  Serial.begin(38400);
  InitParameters(true, 0, 0);
  //  LOCAL_CTRL_REG[selectedRange_Reg] = 0x01;         // default selected range
  Serial.println("starting up L3G4200D");
  setupL3G(LOCAL_CTRL_REG[selectedRange_Reg]); // Configure L3G  - 245, 500 or 2000 deg/sec
  //attachInterrupt(0, L3GDataReady, RISING);
  delay(100); //wait for the sensor to be ready
  uint8_t reg = 0x0f;
  byte regValue = readRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], reg);
  Serial.print("Who am I register ");
  Serial.print(reg, HEX);
  Serial.print(":");
  Serial.println(regValue, BIN);
  if (regValue == L3GD20H || regValue == L3GD20)
  {
    delay(100); //wait for the sensor to be ready
    PrintRegisters();

    //    pinMode(L3GPinInterrupt, INPUT);
    pinMode(SensorOutputReadyPin, INPUT);
#if defined(L3GInterrupt2)
    pinMode(L3GPinInterrupt, INPUT);
    attachInterrupt(L3GInterruptNumber, Robot_L3GDataReady, RISING);
#endif
    //    pinMode(MagnetoPowerPin, OUTPUT);

#if defined(SensorInputRobotRequestPin)
    pinMode(SensorInputRobotRequestPin, INPUT);
#endif
    //    digitalWrite(MagnetoPowerPin, LOW);
    Robot_CalibrateGyro(LOCAL_CTRL_REG[L3GD20H_Address_Reg]);
#if defined(magnetoInstalled)
    {
      //     digitalWrite(MagnetoPowerPin, HIGH);
      //      delay(500);
      compass.init();
      compass.enableDefault();
      /*
        compass.m_min = (LSM303::vector<int16_t>) {   // compass calibration
        -3076, -3378, -2796
        };
        compass.m_max = (LSM303::vector<int16_t>) {
        +2654, +2189, +2991
        };
      */
      compass.m_min = (LSM303::vector<int16_t>) {   // compass calibration
        compassMin1, compassMin2, compassMin3
      };
      compass.m_max = (LSM303::vector<int16_t>) {
        compassMax1, compassMax2, compassMax3
      };
      //     digitalWrite(MagnetoPowerPin, LOW);
      bitWrite(currentStatus, monitMagnetoStatusBit, 0);
      //     Serial.println(UpdateNorthOrientation());
    }
#endif
    Serial.println("ready");
    digitalWrite(SensorOutputReadyPin, HIGH);
  }

  else {
    Serial.println("not the right sensor");
  }
}
void loop() {
  delayMicroseconds(1);
  //  if ((millis() - prevL3GTimer) >= LOCAL_CTRL_REG[L3GcycleDuration_Reg] && monitGyro == true) //LOCAL_CTRL_REG[cycleDuration_Reg]
#if defined(L3GInterrupt2)
  if (dataToRead == true && monitGyro == true) //LOCAL_CTRL_REG[cycleDuration_Reg]
#else
  if ((millis() - prevL3GTimer) >= LOCAL_CTRL_REG[L3GcycleDuration_Reg] && monitGyro == true ) //LOCAL_CTRL_REG[cycleDuration_Reg]
#endif
  {
    dataToRead = false;
    float z = getGyroValues(); // This will update x, y, and z with new values
#if defined(L3GInterrupt2)
    {
    }
#else
    uint8_t statusL3G = readRegister(L3GD20H_Address, 0x27);
    if ((statusL3G && B01000100) != 0x00)
    {
#endif
    if (abs(z - bias) > L3GZero_rate_level * sensitivity[LOCAL_CTRL_REG[selectedRange_Reg]])
    {
      alpha = alpha + (z - bias) * (micros() - savTime + LOCAL_CTRL_REG[GyroBiasMicrosec_Reg]);
      savTime = micros();
      float fAlpha = alpha * sensitivity[LOCAL_CTRL_REG[selectedRange_Reg]];
      int ialpha = fAlpha / 1000;
      if (ialpha >= 0)
      {
        LOCAL_CTRL_REG[relativeHeading_Reg1] = 0x00;
      }
      else
      {
        LOCAL_CTRL_REG[relativeHeading_Reg1] = 0x01;
      }
      LOCAL_CTRL_REG[relativeHeading_Reg2] = uint8_t (abs(ialpha / 256));
      LOCAL_CTRL_REG[relativeHeading_Reg3] = uint8_t (abs(ialpha));
    }

    //   statusL3G = 0x00;
#if defined(L3GInterrupt2)
    attachInterrupt(L3GInterruptNumber, Robot_L3GDataReady, RISING);
#else
    {
    }
  }
#endif
  }

  //  delay(LOCAL_CTRL_REG[cycleDuration_Reg]); //Just here to slow down the serial to make it more readable
#if defined(SensorInputRobotRequestPin)
  if ( (digitalRead(SensorInputRobotRequestPin) == HIGH) ) //LOCAL_CTRL_REG[robotPollingTimer_Reg]
#else
  if ((millis() - prevPollTimer >= LOCAL_CTRL_REG[robotPollingTimer_Reg1] * 256 + LOCAL_CTRL_REG[robotPollingTimer_Reg2]) ) //LOCAL_CTRL_REG[robotPollingTimer_Reg]
#endif
  {
    int dataLen = Robot_PollSlave(LOCAL_CTRL_REG[robotAddress_Reg], currentStatus);
    uint8_t receiveAddress = dataIn[0];
    if (receiveAddress == LOCAL_CTRL_REG[robotAddress_Reg])
    {
      uint8_t cmd = dataIn[1];
      uint8_t numberRegs = dataIn[2];
      uint8_t parameter = dataIn[2];
      switch (cmd)
      {
        case idleRequest:
          break;
        case setRegisterRequest:  // set register
          switch (numberRegs)
          {
            case 0xff:
              InitParameters(true, 0, 0);     // reset default value
              break;
            default:
              if (numberRegs <= maxRegsNumberUpdate)
              {
                for (int i = 0; i < min(numberRegs, maxRegsNumberUpdate); i++)
                {
                  InitParameters(false, dataIn[2 * i + 3], dataIn[2 * i + 4]);
                }
              }
              break;
          }
          break;
        case readRegisterRequest:
          {
            if (numberRegs <= maxRegsNumberRead)
            {
              Robot_SendRegistersValue(receiveAddress, numberRegs, dataIn);
            }
            break;
          }
        case startMonitorGyro:
          {
            int NO = UpdateNorthOrientation();
            LOCAL_CTRL_REG[savedNorthOrientationBefore_Reg1] = uint8_t(NO / 256);
            LOCAL_CTRL_REG[savedNorthOrientationBefore_Reg2] = uint8_t(NO);
            monitGyro = true;
            attachInterrupt(L3GInterruptNumber, Robot_L3GDataReady, RISING);
            Serial.println("Start monitor gyro");
            bitWrite(currentStatus, monitGyroStatusBit, 1);
            savTime = micros();
            startInterruptTime = millis();
            interruptCount = 0;
            break;
          }

        case stopMonitorGyro:
          {
            int NO = UpdateNorthOrientation();
            LOCAL_CTRL_REG[savedNorthOrientationAfter_Reg1] = uint8_t(NO / 256);
            LOCAL_CTRL_REG[savedNorthOrientationAfter_Reg2] = uint8_t(NO);
            monitGyro = false;
            detachInterrupt(L3GInterruptNumber);
            Serial.println("Stop monitor gyro");
            bitWrite(currentStatus, monitGyroStatusBit, 0);
            interruptCount = 0;
            break;
          }
        case startInitMonitorGyro:
          {
            int NO = UpdateNorthOrientation();
            LOCAL_CTRL_REG[savedNorthOrientationBefore_Reg1] = uint8_t(NO / 256);
            LOCAL_CTRL_REG[savedNorthOrientationBefore_Reg2] = uint8_t(NO);
            monitGyro = true;
            alpha = 0;
            LOCAL_CTRL_REG[relativeHeading_Reg1] = 0x00;
            LOCAL_CTRL_REG[relativeHeading_Reg2] = 0x00;
            LOCAL_CTRL_REG[relativeHeading_Reg3] = 0x00;
            attachInterrupt(L3GInterruptNumber, Robot_L3GDataReady, RISING);
            Serial.println("Start & Init monitor gyro");
            bitWrite(currentStatus, monitGyroStatusBit, 1);
            savTime = micros();
            startInterruptTime = millis();
            interruptCount = 0;
            break;
          }
        case stopInitMonitorGyro:
          {
            int NO = UpdateNorthOrientation();
            LOCAL_CTRL_REG[savedNorthOrientationAfter_Reg1] = uint8_t(NO / 256);
            LOCAL_CTRL_REG[savedNorthOrientationAfter_Reg2] = uint8_t(NO);
            monitGyro = false;
            detachInterrupt(L3GInterruptNumber);
            alpha = 0;
            LOCAL_CTRL_REG[relativeHeading_Reg1] = 0x00;
            LOCAL_CTRL_REG[relativeHeading_Reg2] = 0x00;
            LOCAL_CTRL_REG[relativeHeading_Reg3] = 0x00;
            Serial.println("Stop & Init monitor gyro");
            bitWrite(currentStatus,  monitGyroStatusBit, 0);
            interruptCount = 0;
            break;
          }
        case calibrateGyro:
          {
            monitGyro = false;
            bitWrite(currentStatus,  monitGyroStatusBit, 0);
            detachInterrupt(L3GInterruptNumber);
            alpha = 0;
            LOCAL_CTRL_REG[relativeHeading_Reg1] = 0x00;
            LOCAL_CTRL_REG[relativeHeading_Reg2] = 0x00;
            LOCAL_CTRL_REG[relativeHeading_Reg3] = 0x00;
            Serial.println("Calibrate");
            Robot_CalibrateGyro(receiveAddress);
            break;
          }
        case startMonitorMagneto:
          {
            //           digitalWrite(MagnetoPowerPin, HIGH);
            monitMagneto = true;
            Serial.println("Start monitor magneto");
            bitWrite(currentStatus, monitMagnetoStatusBit, 1);
            break;
          }
        case stopMonitorMagneto:
          {
            //           digitalWrite(MagnetoPowerPin, LOW);
            monitMagneto = false;
            Serial.println("Stop monitor magneto");
            bitWrite(currentStatus, monitMagnetoStatusBit, 0);
            break;
          }

        case setGyroSelectedRange:
          {
            LOCAL_CTRL_REG[selectedRange_Reg] = parameter;
            SetGyroRegistersRange(LOCAL_CTRL_REG[selectedRange_Reg]);
            Serial.print("Set selected Range:");
            Serial.println(parameter);
            PrintRegisters();
            break;
          }
        case printGyroRegisters:
          {
            SetGyroODR(parameter);
            PrintRegisters();
            break;
          }
        case setGyroODR:
          {
            LOCAL_CTRL_REG[L3GODR_Reg] = parameter;
            Serial.print("Set ODR:");
            Serial.println(parameter, HEX);
            //          interruptCount = 0;
            //         startInterruptTime = millis();
            SetGyroODR(LOCAL_CTRL_REG[L3GODR_Reg]);
            PrintRegisters();
            break;
          }
        default:
          {

          }
      }
    }
    req = !req;
  }
  if (monitMagneto && millis() - updateNOTimer >= (LOCAL_CTRL_REG[MagnetocycleDuration_Reg1] * 256 + LOCAL_CTRL_REG[MagnetocycleDuration_Reg2]) )
  {
    int NO = UpdateNorthOrientation();
  }
}
float getGyroValues() {
  // digitalWrite(MagnetoPowerPin, LOW);
  prevL3GTimer = millis();
  uint8_t MSB;
  uint8_t LSB;
  if (L3GAxeOrientation == 1)
  {
    MSB = readRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], 0x29);
    LSB = readRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], 0x28);
  }
  if (L3GAxeOrientation == 2)
  {
    MSB = readRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], 0x2b);
    LSB = readRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], 0x2a);
  }
  if (L3GAxeOrientation == 3)
  {
    MSB = readRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], 0x2D);
    LSB = readRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], 0x2C);
  }

  //   int16_t value = (int16_t)((MSB << 8) | LSB);
  int16_t value = ((uint16_t)((MSB << 8) | LSB));
  //   Serial.print("-");
  //   Serial.print(value, HEX);

  // uint8_t statusB = readRegister(L3GD20H_Address, 0x27);
  // statusL3G = statusB;
  return value;
}

int setupL3G(uint8_t range) {
  //digitalWrite(MagnetoPowerPin, LOW);
  //From  Jim Lindblom of Sparkfun's code
  // Enable x, y, z and turn off power down:
  uint8_t valueR;
  if (L3GAxeOrientation == 1)
  {
    valueR = 0b01001001;
    writeRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], L3G_CTRL_REG1, valueR);
    LOCAL_CTRL_REG[L3GRegistersCopySubsystemMapping + L3G_CTRL_REG1] = valueR;
  }
  if (L3GAxeOrientation == 2)
  {
    valueR = 0b01001010;
    writeRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], L3G_CTRL_REG1, valueR);
    LOCAL_CTRL_REG[L3GRegistersCopySubsystemMapping + L3G_CTRL_REG1] = valueR;
  }
  if (L3GAxeOrientation == 3)
  {
    //    writeRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], L3G_CTRL_REG1, 0b00001100);       // ODR=190
    uint8_t reg = 0x0f;
    //  byte regValue = readRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], reg);
    //   if (regValue == L3GD20H )
    //   {
    valueR = 0b01001100;
    writeRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], L3G_CTRL_REG1, valueR);    // ODR=190
    LOCAL_CTRL_REG[L3GRegistersCopySubsystemMapping + L3G_CTRL_REG1] = valueR;
    //   }
    //   if (regValue == L3GD20 )
    //   {
    //     writeRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], L3G_CTRL_REG1, 0b00001111);    // ODR=190
    //   }
  }
  //
  // If you'd like to adjust/use the HPF, you can edit the line below to configure L3G_CTRL_REG2:
  // LVLen set on
  //  valueR = 0b00100000;
  //  writeRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], L3G_CTRL_REG2, valueR);
  //  LOCAL_CTRL_REG[L3GRegistersCopySubsystemMapping + L3G_CTRL_REG2] = valueR;

  // Configure L3G_CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
#if defined(L3GInterrupt2)
  valueR = 0b00001000;
  writeRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], L3G_CTRL_REG3, valueR);
  LOCAL_CTRL_REG[L3GRegistersCopySubsystemMapping + L3G_CTRL_REG3] = valueR;
#endif

  // L3G_CTRL_REG4 controls the full-scale range, among other things:
  SetGyroRegistersRange(range);
  SetGyroODR(LOCAL_CTRL_REG[L3GODR_Reg]);     // set default ODT according to l3GODRValue
  // L3G_CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  valueR = 0b00000000;
  writeRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], L3G_CTRL_REG5, valueR);
  LOCAL_CTRL_REG[L3GRegistersCopySubsystemMapping + L3G_CTRL_REG5] = valueR;
}

void writeRegister(int deviceAddress, byte address, byte val) {
  Wire.beginTransmission(deviceAddress); // start transmission to device
  Wire.write(address);       // send register address
  Wire.write(val);         // send value to write
  Wire.endTransmission();     // end transmission
}

uint8_t readRegister(uint8_t deviceAddress, byte address) {
  uint8_t v;
  Wire.beginTransmission(deviceAddress);
  Wire.write(address); // register to read
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, 1); // read a byte
  while (!Wire.available()) {
    // waiting
  }
  v = Wire.read();
  return v;
}
void InitParameters(boolean allReg, uint8_t regNumber, uint8_t regValue)
{
  if (allReg || regNumber == L3GD20H_Address_Reg)
  {
    Serial.print("L3GD20H_Address:");
    if (allReg)
    {
      LOCAL_CTRL_REG[L3GD20H_Address_Reg] = L3GD20H_Address;         // default L3G address
    }
    else
    {
      LOCAL_CTRL_REG[regNumber] = uint8_t(regValue);
    }
    Serial.println(LOCAL_CTRL_REG[L3GD20H_Address_Reg], HEX);
  }
  if (allReg || regNumber == selectedRange_Reg)
  {
    Serial.print("L3GD20H selected range:");
    if (allReg)
    {
      LOCAL_CTRL_REG[selectedRange_Reg] = selectedRange;         // default selected range
    }
    else
    {
      LOCAL_CTRL_REG[regNumber] = uint8_t(regValue);
    }
    Serial.println(LOCAL_CTRL_REG[selectedRange_Reg]);
  }
#if defined(L3GInterrupt2)
#else

  if (allReg || regNumber == L3GcycleDuration_Reg)
  {
    Serial.print("L3G polling cycle:");
    if (allReg)
    {
      LOCAL_CTRL_REG[L3GcycleDuration_Reg] = L3GPollingCycle;         // default selected range
    }
    else
    {
      LOCAL_CTRL_REG[regNumber] = uint8_t(regValue);
    }
    Serial.println(LOCAL_CTRL_REG[L3GcycleDuration_Reg]);
  }
#endif
  if (allReg || regNumber == robotAddress_Reg)
  {
    Serial.print("Robot addrsss:");
    if (allReg)
    {
      LOCAL_CTRL_REG[robotAddress_Reg] = robotI2CAddress;         // robot I2 address
    }
    else
    {
      LOCAL_CTRL_REG[regNumber] = uint8_t(regValue);
    }
    Serial.println(LOCAL_CTRL_REG[robotAddress_Reg], HEX);
  }
  if (allReg || regNumber == MagnetocycleDuration_Reg1 || regNumber == MagnetocycleDuration_Reg2)
  {
    Serial.print("Magneto cycleDuration:");
    if (allReg)
    {
      LOCAL_CTRL_REG[MagnetocycleDuration_Reg1] = uint8_t(MagnetoCycleDuration / 256); // define robot polling cycle
      LOCAL_CTRL_REG[MagnetocycleDuration_Reg2] = uint8_t(MagnetoCycleDuration);  // define robot polling cycle
    }
    else
    {
      LOCAL_CTRL_REG[regNumber] = uint8_t(regValue);
      LOCAL_CTRL_REG[L3GRegistersCopySubsystemMapping + regNumber] = uint8_t(regValue);
    }
    Serial.println(LOCAL_CTRL_REG[MagnetocycleDuration_Reg1] * 256 + LOCAL_CTRL_REG[MagnetocycleDuration_Reg2]);
  }
  if (allReg || regNumber == robotPollingTimer_Reg1 || regNumber == MagnetocycleDuration_Reg2)
  {
    Serial.print("robotPollingTimer:");
    if (allReg)
    {
      LOCAL_CTRL_REG[robotPollingTimer_Reg1] = uint8_t(robotPollingTimer / 256); // define robot polling cycle
      LOCAL_CTRL_REG[robotPollingTimer_Reg2] = uint8_t(robotPollingTimer);  // define robot polling cycle
    }
    else
    {
      LOCAL_CTRL_REG[regNumber] = uint8_t(regValue);
    }
    Serial.println(LOCAL_CTRL_REG[robotPollingTimer_Reg1] * 256 + LOCAL_CTRL_REG[robotPollingTimer_Reg2]);
  }
  if (allReg || regNumber == L3GODR_Reg)
  {
    Serial.print("Gyro ODR:");
    if (allReg)
    {
      LOCAL_CTRL_REG[L3GODR_Reg] = L3GODRValue;         // robot I2 address
    }
    else
    {
      LOCAL_CTRL_REG[regNumber] = uint8_t(regValue);
    }
    Serial.println(LOCAL_CTRL_REG[L3GODR_Reg], HEX);
  }
  if (allReg || regNumber == GyroBiasMicrosec_Reg)
  {
    Serial.print("GyroBiasMicrosec:");
    if (allReg)
    {
      LOCAL_CTRL_REG[GyroBiasMicrosec_Reg] = GyroBiasMicrosec;         // robot I2 address
    }
    else
    {
      LOCAL_CTRL_REG[regNumber] = uint8_t(regValue);
    }
    Serial.println(LOCAL_CTRL_REG[GyroBiasMicrosec_Reg], HEX);
  }
}
void receiveEvent(int howMany) {
  //  Serial.println(howMany);
  while (Wire.available()) { // loop through all but the last
    inputData[receivedCount] = Wire.read(); // receive byte as a character
    //   Serial.print(inputData[receivedCount], HEX);        // print the character
    //   Serial.print("-");
    receivedCount++;
  }
  //  Serial.println();
}
int Robot_PollSlave(int deviceAddress, byte parameter) {

  int idx = 0;
  prevPollTimer = millis();
  //  Serial.println(address, HEX);
  Wire.beginTransmission(deviceAddress);
  Wire.write(deviceAddress);
  Wire.write(idleRequest);
  Wire.write(parameter);
  Wire.write(pollResponseExpectedLenght);                   // Command Data = dummy zeroes
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, pollResponseExpectedLenght); // read a byte
  while (Wire.available()) {
    dataIn[idx] = Wire.read();
    idx++;
  }
#if defined(debugOn)
  Serial.print("interrupt cycle:");
  Serial.println(float(interruptCount * 1000 / (millis() - startInterruptTime)));
#endif
  return idx;
}
void Robot_SendRegistersValue(uint8_t deviceAddress,  uint8_t numberRegs, uint8_t dataIn[33])
{
  Wire.beginTransmission(deviceAddress); // transmit to device #8
  Wire.write(deviceAddress);
  Wire.write(readRegisterResponse);
  Wire.write(numberRegs);
  for (int i = 0; i < numberRegs; i++)
  {
    Wire.write(dataIn[i + 3]);
    Wire.write(LOCAL_CTRL_REG[dataIn[i + 3]]);            // sends one byte
  }
  Wire.endTransmission();    // stop transmitting
}
void Robot_L3GDataReady()
{
  detachInterrupt(L3GInterruptNumber);
  dataToRead = true;
  interruptCount++;
  // Serial.print(".");
}


void Robot_CalibrateGyro(uint8_t deviceAddress)
{
  for (int i = 0; i < nbReadForBiasComputing; i++)
  {
    uint8_t MSB;
    uint8_t LSB;
    if (L3GAxeOrientation == 1)
    {
      MSB = readRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], 0x29);
      LSB = readRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], 0x28);
    }
    if (L3GAxeOrientation == 2)
    {
      MSB = readRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], 0x2b);
      LSB = readRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], 0x2a);
    }
    if (L3GAxeOrientation == 3)
    {
      MSB = readRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], 0x2D);
      LSB = readRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], 0x2C);
    }
    // uint8_t bitSign = bitRead(MSB, 7);
    // bitWrite(MSB, 7, 0);
    float value = (int16_t)((MSB << 8) | LSB);
    //   int16_t value = ((uint16_t)((MSB << 8) | LSB));
    //   Serial.print("-");
    //   Serial.print(value, HEX);
    //  if (bitSign == 1)
    //   {
    //    value = -value;
    //  }
    //   Serial.print("-");
    //   Serial.println(value, HEX);
    //    byte zMSB = readRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], 0x2D);
    //   byte zLSB = readRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], 0x2C);
    //    z = ((zMSB << 8) | zLSB);
    bias = bias + value;
    delay(03);
  }
  bias = bias / nbReadForBiasComputing;

  Serial.print("bias:");
  Serial.println(bias);
  Wire.beginTransmission(deviceAddress); // transmit to device #8
  Wire.write(deviceAddress);
  Wire.write(calibrateGyro);
  Wire.endTransmission();    // stop transmitting
}

int UpdateNorthOrientation()
{
  int NO = 0;
#if defined(magnetoInstalled)

  compass.read();
  float northOrientation = compass.heading();
#else
  return 0;
#endif
#if defined(debugMagnetoOn)
  Serial.print(" magneto orientation: ");
  Serial.println(northOrientation);

  NO = int(northOrientation);
  LOCAL_CTRL_REG[headingNorthOrientation_Reg1] = uint8_t(NO / 256);
  LOCAL_CTRL_REG[headingNorthOrientation_Reg2] = uint8_t(NO);
#endif
  updateNOTimer = millis();
  return NO;

}
void PrintRegisters()
{
  for (uint8_t reg = L3GRegistersCopySubsystemFirst; reg < L3GRegistersCopySubsystemFirst + L3GRegistersCopySubsystemNumber; reg++)
  {
    byte regValue = readRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], reg);
    uint8_t idx = L3GRegistersCopySubsystemMapping + reg;
    LOCAL_CTRL_REG[idx] = regValue;
    Serial.print("L3G register 0x");
    Serial.print(reg, HEX);
    Serial.print(":");
    Serial.println(regValue, BIN);
    delay(50); //wait for the sensor to be ready
  }

}
void SetGyroRegistersRange(uint8_t range)
{
  uint8_t valueR;
  digitalWrite(SensorOutputReadyPin, LOW);
  if (range == 0) {
    valueR = 0b00000000;
    writeRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], L3G_CTRL_REG4, valueR);

    Serial.println("range 0");
  } else if (range == 1) {
    valueR = 0b00010000;
    writeRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], L3G_CTRL_REG4, valueR);
    Serial.println("range 1");
  } else {
    valueR = 0b00100000;
    writeRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], L3G_CTRL_REG4, valueR);
    Serial.println("range 2");
  }
  LOCAL_CTRL_REG[L3GRegistersCopySubsystemMapping + L3G_CTRL_REG4] = valueR;
  Robot_CalibrateGyro(L3GD20H_Address);
  digitalWrite(SensorOutputReadyPin, HIGH);
}
void SetGyroODR(uint8_t parameter)
{
  digitalWrite(SensorOutputReadyPin, LOW);
  uint8_t  value = readRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], L3G_CTRL_REG1);  // read current value
  value = value & 0x0f;           // keep the half left part of the byte
  parameter = parameter & 0xf0;   // keep the half right part of the byte
  value = value  | parameter;
  writeRegister(LOCAL_CTRL_REG[L3GD20H_Address_Reg], L3G_CTRL_REG1, value);
  LOCAL_CTRL_REG[L3GRegistersCopySubsystemMapping + L3G_CTRL_REG1] = value;
  Robot_CalibrateGyro(L3GD20H_Address);
  digitalWrite(SensorOutputReadyPin, HIGH);
}

