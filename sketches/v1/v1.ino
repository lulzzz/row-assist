#define MODE_PIN 13
#define X_PIN A1
#define Y_PIN A0
#define Z_PIN A2
#define BUFFER_LENGTH 128
#define BAUD 9600

int x_min = 100;
int x_max = 900;
int x = 512;
int y = 512;
int z = 512;
int mode =0;
int mode_counter = 0;
int mode_limit = 1;
char output[BUFFER_LENGTH];

void setup() {
  Serial.begin(BAUD);
  pinMode(MODE_PIN, INPUT);
  digitalWrite(MODE_PIN, HIGH);
  pinMode(X_PIN, INPUT);
  pinMode(Y_PIN, INPUT);
  pinMode(Z_PIN, INPUT);
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
  x = analogRead(X_PIN);
  if (x < x_min) {
    x = -1;
  }
  else if (x > x_max) {
    x = 1;
  }
  else {
    x = 0;
  }
  y = analogRead(Y_PIN);
  z = analogRead(Z_PIN);
  sprintf(output, "{'mode':%d, 'x':%d, 'y':%d, 'z':%d}", mode, x, y, z);
  Serial.println(output);
  Serial.flush();
}
