#include <Arduino.h>
#include <BluetoothSerial.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run menuconfig to enable it
#endif

BluetoothSerial SerialBT;

// دبابيس درايفر المحركات L298N
const int ENA = 2;   // سرعة المحرك الأيسر (PWM)
const int IN1 = 25;  // اتجاه المحرك الأيسر 1
const int IN2 = 26;  // اتجاه المحرك الأيسر 2

const int IN3 = 32;  // اتجاه المحرك الأيمن 1
const int IN4 = 33;  // اتجاه المحرك الأيمن 2
const int ENB = 15;  // سرعة المحرك الأيمن (PWM)

// دوال الحركة (إذا تحرك الروبوت بالعكس عند الضغط على Forward، يمكنك عكس أقطاب IN1/IN2 مع IN3/IN4 هنا)
void moveForward() {
  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW);
  Serial.println("Moving Forward (Towards Front Caster Wheel) ⬆️");
}

void moveBackward() {
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, HIGH);
  Serial.println("Moving Backward ⬇️");
}

void turnLeft() {
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW);
  Serial.println("Turning Left ⬅️");
}

void turnRight() {
  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, HIGH);
  Serial.println("Turning Right ➡️");
}

void stopRobot() {
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, LOW);
  Serial.println("Stopped 🛑");
}

void setup() {
  // بدء الاتصال التسلسلي للمراقبة عبر الكمبيوتر
  Serial.begin(115200);
  delay(1000);

  // تشغيل بلوتوث الـ ESP32 باسم Stark_Robot_BT
  SerialBT.begin("Stark_Robot_BT");
  Serial.println("Bluetooth Started! Pair your phone with 'Stark_Robot_BT'");

  // ضبط دبابيس المحركات كـ Outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // ضبط السرعة الافتراضية للمحركات (من 0 إلى 255)
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
}

void loop() {
  // استقبال الأوامر من تطبيق الهاتف عبر البلوتوث
  if (SerialBT.available()) {
    char cmd = SerialBT.read();
    
    switch (cmd) {
      case 'F':
      case 'f':
        moveForward();
        break;
        
      case 'B':
      case 'b':
        moveBackward();
        break;
        
      case 'L':
      case 'l':
        turnLeft();
        break;
        
      case 'R':
      case 'r':
        turnRight();
        break;
        
      case 'S':
      case 's':
        stopRobot();
        break;
        
      default:
        stopRobot();
        break;
    }
  }
}