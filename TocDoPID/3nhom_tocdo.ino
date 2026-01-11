// ======================= Cấu hình phần cứng ==========================
const byte motorPin1   = 6;   // IN1 L298N
const byte motorPin2   = 7;   // IN2 L298N
const byte motorEnable = 4;   // ENA (PWM ~9)
const byte motorPin1   = 11;   // IN1 L298N
const byte motorPin2   = 12;   // IN2 L298N
const byte motorEnable = 13;   // ENA (PWM ~9) 

const byte encA = 15;          // INT1  (UNO: 2 = INT0, 3 = INT1)
const byte encB = 16;          // INT0

volatile long encoderCount = 0;    // Tổng xung

const long pulsesPerRev = 998.4;   

const float targetRPM = 100;     

// ======================= PID tham số ================================
float Kp = 2;
float Ki = 0.00015;
float Kd = 0.01;

float integralError = 0;
float previousError = 0;
const float integralLimit = 1000;  // giới hạn chống tràn


int pwmOut = 0;
const int pwmMin = 15;     // PWM tối thiểu để động cơ quay
const int pwmMax = 255;     // PWM tối đa (0–255)

// ======================= Chu kỳ tính PID ============================
const unsigned long interval = 10; // ms – 0.1 s
unsigned long prevMillis = 0;

long lastCount = 0;   
bool isRunning = true;

// ======================= Ngắt encoder ===============================
void ISR_encA() {
  if (digitalRead(encA) == digitalRead(encB)) encoderCount++;
  else encoderCount--;
}
void ISR_encB() {
  if (digitalRead(encA) == digitalRead(encB)) encoderCount--;
  else encoderCount++;
}

// ======================= SETUP ======================================
void setup() {
  Serial.begin(115200);
  Serial.println("Khoi tao dong co va encoder (PID toc do)...");

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorEnable, OUTPUT);

  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);

  // quay thuận ban đầu
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  analogWrite(motorEnable, 0);

  integralError = 0;
  previousError = 0;

  attachInterrupt(digitalPinToInterrupt(encA), ISR_encA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB), ISR_encB, CHANGE);

  encoderCount = 0;
  lastCount = 0;
}

// ======================= LOOP =======================================
void loop() {
  unsigned long now = millis();
  if (isRunning && (now - prevMillis >= interval)) {
    prevMillis = now;

    // ====== Tính tốc độ thực tế (RPM) ======
    long delta = encoderCount - lastCount;      // xung trong interval
    lastCount = encoderCount;

    // xung/s = delta * (1000 / interval)
    float pulsesPerSec = delta * (1000.0 / interval);
    float rpm = (pulsesPerSec / pulsesPerRev) * 60.0;

    // ====== Sai số tốc độ ======
    float error = targetRPM - rpm;

    // ====== PID ======
    integralError += error * (interval / 1000.0);
    integralError = constrain(integralError, -integralLimit, integralLimit);

    float derivative = (error - previousError) / (interval / 1000.0);
    previousError = error;

    float pidOut = Kp * error + Ki * integralError + Kd * derivative;

    pwmOut = constrain((int)pidOut, pwmMin, pwmMax);
    if (pwmOut < 0) pwmOut = 0;

    analogWrite(motorEnable, pwmOut);

    // ====== Thông tin debug ======
    Serial.print("RPM: ");  Serial.print(rpm);
    Serial.print(" | Target: "); Serial.print(targetRPM);
    Serial.print(" | PWM: "); Serial.println(pwmOut);
  }
}
