#define MODE_PIN 13
#define STEERING_PIN A1
#define WHEEL_ANGLE_PIN A2
#define SENSITIVITY_PIN A0
#define BIAS_PIN A3
#define ENCODER_A_PIN 2
#define ENCODER_B_PIN 3
#define BUFFER_LENGTH 128
#define BAUD 9600

int steering_min = 100;
int steering_max = 500;
int steering = 330;
int sensitivity = 330;
int bias = 330;
int angle = 512;
int mode =0;
int mode_counter = 0;
int mode_limit = 0;
char output[BUFFER_LENGTH];
int encoder = 0;

void setup() {
  Serial.begin(BAUD);
  pinMode(MODE_PIN, INPUT);
  digitalWrite(MODE_PIN, HIGH);
  attachInterrupt(0, counter, CHANGE); // set encoder interrupt
  pinMode(STEERING_PIN, INPUT);
  pinMode(SENSITIVITY_PIN, INPUT);
  pinMode(BIAS_PIN, INPUT);
  pinMode(WHEEL_ANGLE_PIN, INPUT);
}

void loop() {
  if (!digitalRead(MODE_PIN)) {
    if (mode_counter > mode_limit) {
      mode_counter = 0;
      if (mode) {
        mode = 0;
      }
      else {
        mode = 1;
      }
    }
    else {
      mode_counter++;
    }
  }
  else {
    mode_counter = 0;
  }
  steering = analogRead(STEERING_PIN);
  if (steering < steering_min) {
    steering = -1;
  }
  else if (steering > steering_max) {
    steering = 1;
  }
  else {
    steering = 0;
  }
  sensitivity = analogRead(SENSITIVITY_PIN);
  bias = analogRead(BIAS_PIN);
  angle = analogRead(WHEEL_ANGLE_PIN);
  sprintf(output, "{'mode':%d, 'steering':%d, 'sensitivity':%d, 'bias':%d, 'encoder':%d, 'angle':%d}", mode, steering, sensitivity, bias, encoder, angle);
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

