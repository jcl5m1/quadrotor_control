
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
boolean telemeteryMode = true;

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
        if(b == 't'){
          telemeteryMode = !telemeteryMode;          
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
  
  if(telemeteryMode){
    Serial.write(0xAA);
    Serial.write(xRate>>8);
    Serial.write(xRate & 255);
    Serial.write(yRate>>8);
    Serial.write(yRate & 255);
    Serial.write(zRate>>8);
    Serial.write(zRate & 255);
  }
  UartControl();
  
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


