
#define WHEEL_ANGLE_PIN A2
#define ENCODER_A_PIN 2
#define ENCODER_B_PIN 3
#define BUFFER_LENGTH 128
#define BAUD 9600 

int angle = 512;
char output[BUFFER_LENGTH];
int encoder = 0;

void setup() {
  Serial.begin(BAUD);
  attachInterrupt(0, counter, CHANGE); // set encoder interrupt
  pinMode(WHEEL_ANGLE_PIN, INPUT);
}

void loop() {
  angle = analogRead(WHEEL_ANGLE_PIN);
  sprintf(output, "{\"a\":%d,\"b\":%d}", encoder, angle);
  Serial.println(output);
  Serial.flush();
};

void counter(void) {
  if (digitalRead(ENCODER_A_PIN) == HIGH) { 
    if (digitalRead(ENCODER_B_PIN) == LOW) {  
      encoder++; // CW
    } 
    else {
      encoder--; // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(ENCODER_B_PIN) == HIGH) {   
      encoder++; // CW
    } 
    else {
      encoder--; // CCW
    }
  } 
}

