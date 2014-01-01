/* RGB Analog Example, Teensyduino Tutorial #2
   http://www.pjrc.com/teensy/tutorial2.html

   This example code is in the public domain.
*/

#include <Wire.h>

//This is a list of registers in the ITG-3200. Registers are parameters that determine how the sensor will behave, or they can hold data that represent the
//sensors current status.
//To learn more about the registers on the ITG-3200, download and read the datasheet.
const char WHO_AM_I = 0x00;
const char SMPLRT_DIV= 0x15;
const char DLPF_FS = 0x16;
const char GYRO_XOUT_H = 0x1D;
const char GYRO_XOUT_L = 0x1E;
const char GYRO_YOUT_H = 0x1F;
const char GYRO_YOUT_L = 0x20;
const char GYRO_ZOUT_H = 0x21;
const char GYRO_ZOUT_L = 0x22;
//This is a list of settings that can be loaded into the registers.
//DLPF, Full Scale Register Bits
//FS_SEL must be set to 3 for proper operation
//Set DLPF_CFG to 3 for 1kHz Fint and 42 Hz Low Pass Filter
const char DLPF_CFG_0 = (1<<0);
const char DLPF_CFG_1 = (1<<1);
const char DLPF_CFG_2 = (1<<2);
const char DLPF_FS_SEL_0 = (1<<3);
const char DLPF_FS_SEL_1 = (1<<4);

const char itgAddress = 0x68;

const int FLpropPin =  10;
const int FRpropPin =  9;
const int BLpropPin =  14;
const int BRpropPin =  15;
const int ledPin = 11;

int FLpow;
int FRpow;
int BLpow;
int BRpow;

float xBias = 0;
float yBias = 0;
float zBias = 0;

int sampleCounter = 0;
int samplesSinceLastCommand = 0;
boolean biasCalibrated = false;

const int gyroBiasCount = 300;
int gyroDiv = 15;
int lift = 0;
int yaw = 0;
int pitch = 0;
int roll = 0;

int adjYaw = 0;
int adjPitch = 0;
int adjRoll = 0;

int xRate, yRate, zRate;
int ledState = 0;
int serialParseState = 0;
int p1, p2, p3, p4,p5;

HardwareSerial Uart = HardwareSerial();

void setup()   {                
  pinMode(FLpropPin, OUTPUT);
  pinMode(FRpropPin, OUTPUT);
  pinMode(BLpropPin, OUTPUT);
  pinMode(BRpropPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  
   
  Serial.begin(57600);
  Uart.begin(57600);
  
  Wire.begin();        // join i2c bus (address optional for master)
  
  InitItg3200();  
  delay(50);
  digitalWrite(ledPin, LOW);
}

void PowerBoundCheck() {
  if(FLpow < 0) FLpow = 0;
  if(FRpow < 0) FRpow = 0;
  if(BLpow < 0) BLpow = 0;
  if(BRpow < 0) BRpow = 0;

  if(FLpow > 255) FLpow = 255;
  if(FRpow > 255) FRpow = 255;
  if(BLpow > 255) BLpow = 255;
  if(BRpow > 255) BRpow = 255;  
}

void UpdateMotorControl() {
  
  FLpow = lift - pitch - adjPitch - roll - adjRoll - yaw - adjYaw;
  FRpow = lift - pitch - adjPitch + roll + adjRoll + yaw + adjYaw;
  BLpow = lift + pitch + adjPitch - roll - adjRoll + yaw + adjYaw;
  BRpow = lift + pitch + adjPitch + roll + adjRoll - yaw - adjYaw;
    
  PowerBoundCheck();

  analogWrite(FLpropPin, FLpow);
  analogWrite(FRpropPin, FRpow);
  analogWrite(BLpropPin, BLpow);
  analogWrite(BRpropPin, BRpow);

}

void pulsePin(int pin, int duty, int ms) {
  digitalWrite(ledPin, HIGH);
  analogWrite(pin, duty);
  delay(ms);
  digitalWrite(ledPin, LOW);
  analogWrite(pin, 0);  
  delay(ms);
}

void MotorCheck(int power, int duration){
  pulsePin(FLpropPin,power, duration);
  pulsePin(FRpropPin,power, duration);
  pulsePin(BLpropPin,power, duration);
  pulsePin(BRpropPin,power, duration);  
}
void SerialControl() {
  char b;
  if(!biasCalibrated)
    return;
  
  while (Serial.available() > 0) {
    digitalWrite(ledPin, LOW);
    switch(serialParseState) {
      case 0:
        b = Serial.read();
        if(b == 'g')
          serialParseState = 1;
        break;
       case 1:
        p1 = Serial.read();
        serialParseState = 2;
        break;
       case 2:
        p2 = Serial.read();
        serialParseState = 3;
        break;
       case 3:
        p3 = Serial.read();
        serialParseState = 4;
        break;
       case 4:
        p4 = Serial.read();
        serialParseState = 5;
        break;
       case 5:
        p5 = Serial.read();
        serialParseState = 6;
        break;
       case 6:
        b = Serial.read();
        if(b == 's'){//passes packet terminator check
          lift = p1;
          pitch = p2-128;
          yaw = p3-128;
          roll = p4-128;
          gyroDiv = p5;
//          Serial.println(lift);
//          Serial.println(pitch);
//          Serial.println(yaw);
//          Serial.println(roll);          
        }
        serialParseState = 0;
        break;
    }
  }
  digitalWrite(ledPin, HIGH);
}

void UartControl() {
  char b;
  if(!biasCalibrated)
    return;
  
  while (Uart.available() > 0) {
    digitalWrite(ledPin, LOW);
    switch(serialParseState) {
      case 0:
        b = Uart.read();
        if(b == 'g')
          serialParseState = 1;
        break;
       case 1:
        p1 = Uart.read();
        serialParseState = 2;
        break;
       case 2:
        p2 = Uart.read();
        serialParseState = 3;
        break;
       case 3:
        p3 = Uart.read();
        serialParseState = 4;
        break;
       case 4:
        p4 = Uart.read();
        serialParseState = 5;
        break;
       case 5:
        p5 = Uart.read();
        serialParseState = 6;
        break;
       case 6:
        b = Uart.read();
        if(b == 's'){//passes packet terminator check
          lift = p1;
          pitch = p2-128;
          yaw = p3-128;
          roll = p4-128;
          gyroDiv = p5;
        }
        if(b == 'r') {
          biasCalibrated = false;
          sampleCounter = 0; 
        }
        samplesSinceLastCommand = 0;
        serialParseState = 0;
        break;
    }
  }
  digitalWrite(ledPin, HIGH);
}


void ReadGyro(){
  
  //Create variables to hold the output rates.
  //Read the x,y and z output rates from the gyroscope.
  xRate = readX();//pitch
  yRate = readY();//roll
  zRate = readZ();//yaw  
    
  if(sampleCounter < gyroBiasCount){
    xBias += xRate;
    yBias += yRate;
    zBias += zRate;    
    sampleCounter++;
    
    if((sampleCounter % 10) == 0) {
      ledState = !ledState;
      digitalWrite(ledPin, ledState);
    }
    
    if(sampleCounter == gyroBiasCount) {
      xBias /= gyroBiasCount;
      yBias /= gyroBiasCount;
      zBias /= gyroBiasCount;
      biasCalibrated = true;      
      digitalWrite(ledPin, HIGH);
    }
  }   
}

void blinkTest(){
 digitalWrite(ledPin, HIGH);
 delay(200);
 digitalWrite(ledPin, LOW);
 delay(200);
}

void SerialPassthrough(){
    if(Uart.available() > 0) {
      ledState = !ledState;
      Serial.write(Uart.read());
    }
    if(Serial.available() > 0) {
      ledState = !ledState;
      Uart.write(Serial.read());      
    }
    
    digitalWrite(ledPin, ledState);
}

void loop()                     
{    
//  SerialPassthrough();
//  return;
  
  ReadGyro();
  UartControl();
//  SerialControl();
  
  if(biasCalibrated) {
    if(lift > 0) {
      adjPitch = -(xRate-xBias)/gyroDiv;
      adjRoll = -(yRate-yBias)/gyroDiv;
      adjYaw = (zRate-zBias)/gyroDiv;
    } else {
      adjPitch = 0;
      adjRoll = 0;
      adjYaw = 0;      
      pitch = 0;
      roll = 0;
      yaw = 0;
    }    
    UpdateMotorControl();
  }

  //Wait 10ms before reading the values again. (Remember, the output rate was set to 100hz and 1reading per 10ms = 100hz.)
  delay(10);  
  
  //require heartbeat - dead man switch
  if(samplesSinceLastCommand > 100){
    lift = 0;
    adjPitch = 0;
    adjRoll = 0;
    adjYaw = 0;      
    pitch = 0;
    roll = 0;
    yaw = 0;
    UpdateMotorControl();
  }
  else {
    samplesSinceLastCommand++;
  }
  
}



void InitItg3200(){
  //Read the WHO_AM_I register and print the result
  char id=0; 
  id = itgRead(itgAddress, 0x00);  
  Serial.print("ID: ");
  Serial.println(id, HEX);
  
  //Configure the gyroscope
  //Set the gyroscope scale for the outputs to +/-2000 degrees per second
  itgWrite(itgAddress, DLPF_FS, (DLPF_FS_SEL_0|DLPF_FS_SEL_1|DLPF_CFG_0));
  //Set the sample rate to 100 hz
  itgWrite(itgAddress, SMPLRT_DIV, 9);

}

//This function will write a value to a register on the itg-3200.
//Parameters:
//  char address: The I2C address of the sensor. For the ITG-3200 breakout the address is 0x69.
//  char registerAddress: The address of the register on the sensor that should be written to.
//  char data: The value to be written to the specified register.
void itgWrite(char address, char registerAddress, char data)
{
  //Initiate a communication sequence with the desired i2c device
  Wire.beginTransmission(address);
  //Tell the I2C address which register we are writing to
  Wire.write(registerAddress);
  //Send the value to write to the specified register
  Wire.write(data);
  //End the communication sequence
  Wire.endTransmission();
}
//This function will read the data from a specified register on the ITG-3200 and return the value.
//Parameters:
//  char address: The I2C address of the sensor. For the ITG-3200 breakout the address is 0x69.
//  char registerAddress: The address of the register on the sensor that should be read
//Return:
//  unsigned char: The value currently residing in the specified register
unsigned char itgRead(char address, char registerAddress)
{
  //This variable will hold the contents read from the i2c device.
  unsigned char data=0;
  
  //Send the register address to be read.
  Wire.beginTransmission(address);
  //Send the Register Address
  Wire.write(registerAddress);
  //End the communication sequence.
  Wire.endTransmission();
  
  //Ask the I2C device for data
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 1);
  
  //Wait for a response from the I2C device
  if(Wire.available()){
    //Save the data sent from the I2C device
    data = Wire.read();
  }
  
  //End the communication sequence.
  Wire.endTransmission();
  
  //Return the data read during the operation
  return data;
}
//This function is used to read the X-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second. 
//Usage: int xRate = readX();
int readX(void)
{
  int data=0;
  data = itgRead(itgAddress, GYRO_XOUT_H)<<8;
  data |= itgRead(itgAddress, GYRO_XOUT_L);  
  
  return data;
}
//This function is used to read the Y-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second. 
//Usage: int yRate = readY();
int readY(void)
{
  int data=0;
  data = itgRead(itgAddress, GYRO_YOUT_H)<<8;
  data |= itgRead(itgAddress, GYRO_YOUT_L);  
  
  return data;
}
//This function is used to read the Z-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second. 
//Usage: int zRate = readZ();
int readZ(void)
{
  int data=0;
  data = itgRead(itgAddress, GYRO_ZOUT_H)<<8;
  data |= itgRead(itgAddress, GYRO_ZOUT_L);  
  
  return data;
}


