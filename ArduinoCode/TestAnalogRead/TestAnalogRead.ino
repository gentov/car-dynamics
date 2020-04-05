void setup() {
  // put your setup code here, to run once:
  long numReads = 0;
  Serial.begin(9600);
  bool logic= false;
  long currentTime = millis();
  long timeToEnd = currentTime+1000;
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  while(timeToEnd>millis()){
    analogRead(A0);
    numReads = numReads+1;
    
  }
  Serial.println(numReads);
  Serial.println(millis()-currentTime);


  currentTime = millis();
  timeToEnd = currentTime+1000;
  numReads = 0;
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  while(timeToEnd>millis()){
    analogRead(A0);
    numReads = numReads+1;
    logic = ~logic;
    digitalWrite(10, logic);
    
  }
  Serial.println(numReads);
  Serial.println(millis()-currentTime);

  currentTime = millis();
  timeToEnd = currentTime+1000;
  numReads = 0;
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  while(timeToEnd>millis()){
    analogRead(A0);
    numReads = numReads+1;
    logic = ~logic;
    digitalWrite(10, logic);
    analogWrite(11, 60);
    
  }
  Serial.println(numReads);
  Serial.println(millis()-currentTime);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  
  
}
