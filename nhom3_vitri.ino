// ==== Chân L298N ====
const byte motorPin1   = 4;   // IN1
const byte motorPin2   = 5;   // IN2
const byte motorEnable = 9;   // ENA PWM

// ==== Chân encoder ====
const byte encA = 3;          // INT0
const byte encB = 2;          // INT1

volatile long encoderCount = 0;   // đếm xung

// ==== Thông số encoder ====
const long pulsesPerRev = 998.4;         // khoảng 800 xung / vòng

 Tích phân -----
    integralError += error * (interval / 1000.0);
    integralError = constrain(integralError, -integralLimit, integralLimit);

    // ----- Đạo hàm -----
    float derivative = (error - previousError) / (interval / 1000.0);
 Tích phân -----
    integralError += error * (interval / 1000.0);
    integralError = constrain(integralError, -integralLimit, integralLimit);

    // ----- Đạo hàm -----
    float derivative = (error - previousError) / (interval / 1000.0);
ch phân -----
    integralError += error * (interval / 1000.0);
    integralError = constrain(integralError, -integralLimit, integralLimit);

    // ----- Đạo hàm -----
    float derivative = (error - previousError) / (interval / 1000.0);
 Tích phân -----
    integralError += error * (interval / 1000.0);
    integralError = constrain(integralError, -integralLimit, integralLimit);

    // ----- Đạo hàm -----
    float derivative = (error - previousError) / (interval / 1000.0);
ch phân -----
    integralError += error * (interval / 1000.0);
    integralError = constrain(integralError, -integralLimit, integralLimit);

    // ----- Đạo hàm -----
    float derivative = (error - previousError) / (interval / 1000.0);


float integralError = 0;
float previousError = 0;
const float integralLimit = 1000;  // giới hạn chống tràn

// ==== PWM giới hạn ====
int pwmOut = 0;
const int pwmMin = 15;    // tối thiểu để động cơ khởi động
const int pwmMax =  Tích phân -----
    integralError += error * (interval / 1000.0);
    integralError = constrain(integralError, -integralLimit, integralLimit);

    // ----- Đạo hàm -----
    float derivative = (error - previousError) / (interval / 1000.0);
20;   // tối đa (0–255)

// ==== Chu kỳ tính PID ====
const unsigned long interval = 15;   // ms
unsigned long prevMillis = 0;

bool isRunning = true;

// ----------------- Ngắt encoder -----------------
void ISR_encA() {
  if (digitalRead(encA) == digitalRead(encB)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}
void ISR_encB() {
  if (digitalRead(encA) == digitalRead(encB)) {
    encoderCount--;
  } else {
    encoderCount++;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Khoi tao dong co va encoder...");

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorEnable, OUTPUT);

  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);

  // quay thuận ban đầu
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  analogWrite(motorEnable, 0);  // tránh giật

  integralError = 0;
  previousError = 0;

  attachInterrupt(digitalPinToInterrupt(encA), ISR_encA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB), ISR_encB, CHANGE);

  encoderCount = 0;
}

void loop() {
  unsigned long now = millis();
  if (isRunning && (now - prevMillis >= interval)) {
    prevMillis = now;

    // ----- Sai số -----
    float error = targetCounts - encoderCount;

    // ----- Tích phân -----
    integralError += error * (interval / 1000.0);
    integralError = constrain(integralError, -integralLimit, integralLimit);

    // ----- Đạo hàm -----
    float derivative = (error - previousError) / (interval / 1000.0);

    // ----- PID output -----
    float pidOut = Kp * error + Ki * integralError + Kd * derivative;

    pwmOut = constrain((int)pidOut, pwmMin, pwmMax);
    if (pwmOut < 0) pwmOut = 0;

    analogWrite(motorEnable, pwmOut);

    previousError = error;

    Serial.print("Count: ");
    Serial.print(encoderCount);
    Serial.print(" | PWM: ");
    Serial.print(pwmOut);
    Serial.print(" | Error: ");
    Serial.println(error);

    // ----- Dừng khi đủ 30 vòng -----
    if (encoderCount >= targetCounts) {
      isRunning = false;
      analogWrite(motorEnable, 0);
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, LOW);
      Serial.println("Hoan thanh ne");
    }
  }
}
