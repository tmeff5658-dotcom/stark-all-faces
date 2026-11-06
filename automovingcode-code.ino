#include <Arduino.h>
#include <ESP32Servo.h>

// ==========================================
// 🛠️ 1. مداخل المحركات (L298N)
// ==========================================
const int ENA = 2;   
const int IN1 = 25;  
const int IN2 = 26;  
const int IN3 = 32;  
const int IN4 = 33;  
const int ENB = 15;  

// ==========================================
// 🎯 2. الحساسات والسيرفو
// ==========================================
const int TRIG_PIN = 5;
const int ECHO_PIN = 18;
const int FRONT_SERVO_PIN = 14;

Servo frontServo;

// ==========================================
// 🚗 دوال الحركة
// ==========================================
void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void moveBackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void turnLeft() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void turnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void stopRobot() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// قراءة المسافة بدقة
long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  long cm = duration * 0.034 / 2;
  return cm == 0 ? 999 : cm;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // إعداد دبابيس المحركات
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  
  // ضبط سرعة مناسبة وثابتة للحركة الذاتية (قيمة من 0 إلى 255)
  analogWrite(ENA, 180); 
  analogWrite(ENB, 180);

  // إعداد الحساسات
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // إعداد السيرفو وتوسيته للأمام
  frontServo.attach(FRONT_SERVO_PIN);
  frontServo.write(90);
  delay(500);

  Serial.println("Stark Autonomous Mode Initialized!");
}

void loop() {
  long distance = getDistance();
  Serial.print("Front Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance > 30) {
    // الطريق آمن، تابع التقدم للأمام
    moveForward();
  } else {
    // تم رصد عائق! توقف أولاً
    stopRobot();
    delay(300);

    // تراجع قليلاً للخلف لمنع الاصطدام ولإعطاء مجال للاستدارة
    moveBackward();
    delay(400);
    stopRobot();
    delay(200);

    // التفت بالسيرفو يميناً واقرأ المسافة
    frontServo.write(20);
    delay(500);
    long rightDist = getDistance();

    // التفت بالسيرفو يساراً واقرأ المسافة
    frontServo.write(160);
    delay(500);
    long leftDist = getDistance();

    // أعد السيرفو للوسط (للأمام)
    frontServo.write(90);
    delay(300);

    // قارن المسافات واتخذ القرار
    if (leftDist > rightDist) {
      Serial.println("Turning Left (Clearer path)");
      turnLeft();
      delay(600); // زمن الاستدارة
    } else {
      Serial.println("Turning Right (Clearer path)");
      turnRight();
      delay(600); // زمن الاستدارة
    }
    stopRobot();
    delay(200);
  }

  delay(100);
}