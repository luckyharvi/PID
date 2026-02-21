// =====================
// Pin Motor VNH2SP30
// =====================
#define INA 25
#define INB 26
#define PWM 27      // PWM pin ESP32

// =====================
// Encoder pins
// =====================
#define ENCA 34     
#define ENCB 35     

// =====================
// Encoder variables
// =====================
volatile long encoderCount = 0;
volatile long lastCount = 0;
const int CPR = 1320; 

// =====================
// PID Variables
// =====================
float setRPM = 35.0;

float Kp = 6.9444;
float Ki = 9.6451;
float Kd = 0.125;

float error = 0, lastError = 0;
float integral = 0, derivative = 0;
int pwmOutput = 0;

unsigned long lastRPMTime = 0;
float motorRPM = 0;        // <- nilai RPM SETELAH FILTER
float dtSec = 0.1f;

const unsigned long interval = 100; // ms

// Low-pass filter coefficient (0..1)
const float alpha = 0.1f;  // 0.3 = cukup halus, tapi masih responsif

// =====================
// Encoder ISR
// =====================
void IRAM_ATTR readEncoderA() {
  if (digitalRead(ENCA) == digitalRead(ENCB)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void IRAM_ATTR readEncoderB() {
  if (digitalRead(ENCA) != digitalRead(ENCB)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

// =====================
// Motor control
// =====================
void motorForward(int pwmValue) {
  digitalWrite(INA, HIGH);
  digitalWrite(INB, LOW);

  pwmValue = constrain(pwmValue, 0, 255);
  int duty = pwmValue * 4;    // 0–255 -> 0–1023
  ledcWrite(PWM, duty);
}

// =====================
// PID Calculation
// =====================
int computePID(float currentRPM, float dt) {
  error = setRPM - currentRPM;

  // P
  float P = Kp * error;

  // I
  integral += error * dt;
  integral = constrain(integral, -800, 800);
  float I = Ki * integral;

  // D
  float D = Kd * (error - lastError) / dt;
  lastError = error;

  float output = P + I + D;

  if (output > 255) output = 255;
  if (output < 0)   output = 0;

  return (int)output;
}

// =====================
// RPM Calculation (dengan low-pass filter)
// =====================
bool calculateRPM() {
  unsigned long now = millis();
  if (now - lastRPMTime >= interval) {
    dtSec = (now - lastRPMTime) / 1000.0f;

    // --- hitung RPM mentah dari encoder ---
    float motorRPM_raw = (encoderCount * 60.0f) / (CPR * dtSec);

    lastCount = encoderCount;   // simpan pulse untuk debug
    encoderCount = 0;
    lastRPMTime = now;

    // --- LOW PASS FILTER sederhana (exponential) ---
    // motorRPM sebelumnya difilter lagi dengan nilai baru
    motorRPM = (1.0f - alpha) * motorRPM + alpha * motorRPM_raw;

    // --- kirim ke Serial Plotter ---
    Serial.print("Setpoint:");
    Serial.print(setRPM);
    Serial.print(",RPM:");
    Serial.print(motorRPM);      // sudah terfilter
    Serial.print(",PWM:");
    Serial.print(pwmOutput);
    Serial.print(",Pulses:");
    Serial.println(lastCount);

    return true;
  }
  return false;
}

// =====================
// Setup
// =====================
void setup() {
  Serial.begin(115200);

  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(PWM, OUTPUT);

  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB), readEncoderB, CHANGE);

  ledcAttach(PWM, 20000, 10);    // 20 kHz, 10-bit

  lastRPMTime = millis();
  Serial.println("System Ready - PID + Low-pass RPM");
}

// =====================
// Loop
// =====================
void loop() {
  if (calculateRPM()) {
    pwmOutput = computePID(motorRPM, dtSec);
    motorForward(pwmOutput);
  }
}
