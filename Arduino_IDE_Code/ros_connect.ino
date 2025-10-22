// กำหนดว่า LED แต่ละสีต่ออยู่กับขาไหน
const int RED_ledPin = 3;
const int YELLOW_ledPin = 4;
const int GREEN_ledPin = 5;

void setup() {
  pinMode(RED_ledPin, OUTPUT);
  pinMode(YELLOW_ledPin, OUTPUT);
  pinMode(GREEN_ledPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); 

    if (input == "1") { 
      digitalWrite(RED_ledPin, HIGH);
      digitalWrite(YELLOW_ledPin, LOW);
      digitalWrite(GREEN_ledPin, LOW);
    } 
    else if (input == "2") { 
      digitalWrite(RED_ledPin, LOW);
      digitalWrite(YELLOW_ledPin, HIGH);
      digitalWrite(GREEN_ledPin, LOW);
    } 
    else if (input == "3") { 
      digitalWrite(RED_ledPin, LOW);
      digitalWrite(YELLOW_ledPin, LOW);
      digitalWrite(GREEN_ledPin, HIGH);
    }
    else {
      digitalWrite(RED_ledPin, LOW);
      digitalWrite(YELLOW_ledPin, LOW);
      digitalWrite(GREEN_ledPin, LOW);
    }
  }
}
