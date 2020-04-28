#include <HardWire.h>

//#define debug
// Global Variables
unsigned char EncoderPinA = 2;
unsigned char EncoderPinB = 4;
unsigned char SteeringPot = A0;
unsigned char DriveMotorDirectionPin = 8;
unsigned char SteeringMotorDirectionPin = 10;
unsigned char DriveMotorPWMPin = 9;
unsigned char SteeringMotorPWMPin = 11;
unsigned char i2cIndex = 0;

int DesiredADCValue = 512;
unsigned char MotorPWM = 0;
unsigned long ticks = 1000000;//Start at a large number so it doesnt underflow
long MeasuredADCValue = 0;
unsigned char drivingDirection = 0;
unsigned char steeringDirection = 0;
bool steeringMoving = false;
long lastTimeSinceMotorCMD = 0;
    
void setup() {
    pinMode(EncoderPinA, INPUT);
    pinMode(EncoderPinB, INPUT);
    pinMode(DriveMotorDirectionPin, OUTPUT);
    pinMode(SteeringMotorDirectionPin, OUTPUT);
    pinMode(DriveMotorPWMPin, OUTPUT);
    pinMode(SteeringMotorPWMPin, OUTPUT);
    //Serial.begin(115200);
    // Address of arduino
    //Wire.begin(4);
    // callback
    //Wire.onReceive(receiveCommand);
    //Wire.onRequest(receiveRequest);
    attachInterrupt(digitalPinToInterrupt(EncoderPinA), count_ticks, RISING);

    Serial.begin(115200);
    
    
}

void loop() {
    setSteeringAngle();
    runSerialComm();
    if((millis()-lastTimeSinceMotorCMD)>5000){
      setMotorSpeed(0, DriveMotorPWMPin);
      
    }
    
      
}

void runSerialComm(){
  String command;
  while(Serial.available()){
    delay(3);
    if(Serial.available()>0){
        char c = Serial.read();
        command += c;
      }
  } 
  
  int com;
  if(command.length() > 0){
    com = command.substring(0, 1).toInt();
    if(com==0){
      //Serial.println("Data");
      Serial.println(MeasuredADCValue);
      Serial.println(ticks);
    }
    else if(com==1){
      //Serial.println("Recieved Turn CMD");
      DesiredADCValue = command.substring(1, 5).toInt();
      
    }
    else if(com == 2){
      //Serial.println("Recieved Drive CMD");
      int motDir = command.substring(1, 2).toInt();
      int motorSpeed = command.substring(2, 5).toInt();
      //Serial.println(motDir);
      //Serial.println(motorSpeed);
      setMotorDirection(motDir, DriveMotorDirectionPin);
      setMotorSpeed(motorSpeed, DriveMotorPWMPin);
      lastTimeSinceMotorCMD = millis();
    }
  } 
}


void count_ticks(){
    // if direction is forward increment ticks
    if(drivingDirection){
        //drivingDirection = 0;
        ticks --;
    }

    // otherwise decrement ticks
    else{
        //drivingDirection = 1;
        ticks ++;
    }
    //if (ticks<0){
    //  ticks = 0;
    //}

}

void setMotorDirection(int motdirection, int motorDirectionPin){
    // Set the direction pin to the given direction
    if(motdirection == 1){
    digitalWrite(motorDirectionPin, HIGH);
    }

    else{
    digitalWrite(motorDirectionPin, LOW);
    }
    drivingDirection = motdirection;

}

//TODO: Verify direction is correct
void setSteeringAngle(){
    int ADCTol = 9;
    int Motorspeed = 70;
    unsigned long currentADCVal = getADCVal();
    noInterrupts();
    int tempADCVal = DesiredADCValue;
    interrupts();
    //Serial.println(currentADCVal);
    if(tempADCVal < (currentADCVal - ADCTol)){
        if(steeringDirection != 1){
           setMotorDirection(1, SteeringMotorDirectionPin);
           steeringDirection = 1;
        }
        
        if(steeringMoving == false){
          setMotorSpeed(Motorspeed, SteeringMotorPWMPin);
          steeringMoving = true;
        }
        //Serial.println("Moving Right");
    }
    else if(tempADCVal > (currentADCVal + ADCTol)){
        if(steeringDirection != 0){
           setMotorDirection(0, SteeringMotorDirectionPin);
           steeringDirection = 0;
        }
        
        if(steeringMoving == false){
          setMotorSpeed(Motorspeed, SteeringMotorPWMPin);
          steeringMoving = true;
        }
       // Serial.println("Moving Left");
    }
    else{
        setMotorSpeed(0, SteeringMotorPWMPin);
        steeringMoving = false;
    }
}

unsigned long getADCVal(){
    noInterrupts();
    MeasuredADCValue = analogRead(SteeringPot);
    interrupts();
    return MeasuredADCValue;
}

void setMotorSpeed(int newspeed, int motorPWMPin){
    analogWrite(motorPWMPin, newspeed);
}

void receiveCommand(int bytes){
    char temp[50];
	int i = 0;

    // Get the entire command from the Raspberry pi
	while(Wire.available()){
		temp[i]=Wire.read();
		i++;
	}
    // Null Terminate it
	temp[i]='\0';

    // Determine which type of command it is
    char msg = (temp[0]&0b00001111);

    // If it is a steering command
    if(msg == 1){ //Not sure if MSB and LSB first
        DesiredADCValue = (((int)(temp[1]&0b11111111))<<8) + ((int)(temp[2]&0b11111111));
        lastTimeSinceMotorCMD = millis();
    }
    // otherwise, if it is a motorspeed command
    else if(msg == 2){
        drivingDirection = temp[2]&0b00000001;
        setMotorDirection(drivingDirection, DriveMotorDirectionPin);
        MotorPWM = temp[1];
        setMotorSpeed(MotorPWM, DriveMotorPWMPin);
        lastTimeSinceMotorCMD = millis();

    }
    

    
	#ifdef debug
	Serial.println("Message Received");
	Serial.println((int)(temp[0]));
  Serial.println(temp[1], BIN);
  Serial.println(temp[2], BIN);
    Serial.println(DesiredADCValue);
    //Serial.println(MotorPWM);
    //Serial.println(drivingDirection);
	#endif
}

void receiveRequest(){
    switch(i2cIndex){
      case 0:
      Wire.write((byte)(MeasuredADCValue>>8));
      //Wire.write(4);
      break;

      case 1:
      Wire.write((byte)MeasuredADCValue);
      //Wire.write(7);
      break;

      case 2:
      Wire.write((byte)(ticks>>24));
      break;

      case 3:
      Wire.write((byte)(ticks>>16));
      break;

      case 4:
      Wire.write((byte)(ticks>>8));
      break;

      case 5:
      Wire.write((byte)ticks);
      break;
      
    }
    i2cIndex++;
    if (i2cIndex == 6){
      i2cIndex = 0;
    }
    //Wire.write((uint8_t*)&MeasuredADCValue, sizeof(long));
    //Wire.write(MeasuredADCValue);
    //Wire.write(ticks);
    #ifdef debug
  //Serial.println("Request Received");
  //Serial.println(MeasuredADCValue);
  
  #endif
    
    
}
