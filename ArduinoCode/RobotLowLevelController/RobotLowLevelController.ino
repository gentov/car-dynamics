#include <Wire.h>
//define debug
// Global Variables
unsigned char EncoderPinA = 1;
unsigned char EncoderPinB = 1;
unsigned char SteeringPot = 1;
unsigned char DriveMotorDirectionPin = 1;
unsigned char SteeringMotorDirectionPin = 1;
unsigned char DriveMotorPWMPin = 1;
unsigned char SteeringMotorPWMPin = 1;

int DesiredADCValue = 512;
unsigned char MotorPWM = 0;
unsigned long ticks = 0;
unsigned long MeasuredADCValue = 0;
unsigned char drivingDirection = 0;
    
void setup() {
    Serial.begin(9600);
    // Address of arduino
    Wire.begin(8);
    // callback
    Wire.onReceive(receiveCommand);
    Wire.onRequest(receiveRequest);
    attachInterrupt(digitalPinToInterrupt(EncoderPinA), count_ticks, RISING);
}

void loop() {
    setSteeringAngle();
}


void count_ticks(){
    // if direction is forward increment ticks
    if(digitalRead(EncoderPinB) == HIGH){
        drivingDirection = 1;
        ticks ++;
    }

    // otherwise decrement ticks
    else{
        drivingDirection = 0;
        ticks --;
    }

}

void setMotorDirection(int direction, int motorDirectionPin){
    // Set the direction pin to the given direction
    if(direction == 1){
    digitalWrite(motorDirectionPin, HIGH);
    }

    else{
    digitalWrite(motorDirectionPin, LOW);
    }

}

//TODO: Verify direction is correct
void setSteeringAngle(){
    int ADCTol = 5;
    int Motorspeed = 12;
    unsigned long currentADCVal = getADCVal();
    noInterrupts();
    int tempADCVal = DesiredADCValue;
    interrupts();
    if(tempADCVal < (currentADCVal - ADCTol)){
        setMotorDirection(1, SteeringMotorDirectionPin);
        setMotorSpeed(Motorspeed, SteeringMotorPWMPin);
    }
    else if(tempADCVal > (currentADCVal + ADCTol)){
        setMotorDirection(0, SteeringMotorDirectionPin);
        setMotorSpeed(Motorspeed, SteeringMotorPWMPin);
    }
    else{
        setMotorSpeed(0, SteeringMotorPWMPin);
    }
}

unsigned long getADCVal(){
    noInterrupts();
    MeasuredADCValue = analogRead(SteeringPot);
    interrupts();
    return MeasuredADCValue;
}

void setMotorSpeed(int speed, int motorPWMPin){
    analogWrite(motorPWMPin, speed);
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
        DesiredADCValue = ((int)temp[1])<<8 + ((int)temp[2]);
    }
    // otherwise, if it is a motorspeed command
    else if(msg == 2){
        drivingDirection = temp[2]&0b00000001;
        setMotorDirection(drivingDirection, DriveMotorDirectionPin);
        MotorPWM = temp[1];
        setMotorSpeed(MotorPWM, DriveMotorPWMPin);

    }
	#ifdef debug
	Serial.println("Message Received");
	Serial.println(temp[0]&0b00001111,BIN);
    Serial.println(DesiredADCValue);
    Serial.println(MotorPWM);
    Serial.println(drivingDirection);
	#endif
}

void receiveRequest(){
    Wire.write(MeasuredADCValue);
    Wire.write(ticks);
    
}
