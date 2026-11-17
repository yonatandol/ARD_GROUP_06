#define IR_USE_AVR_TIMER2
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <VL53L1X.h>
#include <Servo.h>
#include <IRremote.hpp>

// --- Pin Assignments ---
int btnPin    = 2;
int buzzerPin = 3;
int servoPin  = 6;
int ledPin    = 7;
int trigPin   = 8;
int echoPin   = 9;
int irPin     = 11;

// --- Hardware Objects ---
LiquidCrystal_I2C lcd(0x27, 16, 2);
VL53L1X laser;
Servo gateServo;

// --- Calibration & Thresholds ---
int spinTimeMs   = 350;  // Adjust (ms) to set how far the gate swings (~90 deg)
int queueDistCm  = 25;   // Ultrasonic detection distance (cm)
int boothOccMm   = 200;  // Laser distance when voter is inside (< 200 mm)
int boothClearMm = 250;  // Laser distance when booth is empty (> 250 mm)

int state = 0;
int votesA = 0;
int votesB = 0;
int votesC = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);

  pinMode(btnPin, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, HIGH); // Active-LOW buzzer OFF
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Booting Booth...");

  gateServo.attach(servoPin);
  gateServo.write(90); // Keep 360 servo stopped on boot

  IrReceiver.begin(irPin, ENABLE_LED_FEEDBACK);

  laser.setTimeout(500);
  if (!laser.init()) {
    lcd.clear();
    lcd.print("VL53L1X Error!");
    Serial.println("Failed to detect VL53L1X!");
    while (1);
  }
  laser.setDistanceMode(VL53L1X::Short);
  laser.setMeasurementTimingBudget(50000);
  laser.startContinuous(50);

  showIdleScreen();
}

void loop() {
  int distCm = getUltrasonicCm();
  int distMm = laser.read();

  // STATE 0: Booth idle, checking ultrasonic sensor for incoming voter
  if (state == 0) {
    if (distCm > 0 && distCm < queueDistCm) {
      state = 1;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Voter Detected");
      lcd.setCursor(0, 1);
      lcd.print("Press BTN Enter");
    }
  }

  // STATE 1: Voter in queue, waiting for button press (Pin 2) to open gate
  else if (state == 1) {
    if (distCm >= queueDistCm || distCm == 0) {
      state = 0;
      showIdleScreen();
    } else if (digitalRead(btnPin) == LOW) {
      delay(50);
      beep(1, 150);
      digitalWrite(ledPin, HIGH);
      openGate();
      state = 2;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Gate Open!");
      lcd.setCursor(0, 1);
      lcd.print("Enter Booth...");
      delay(400);
    }
  }

  // STATE 2: Gate open, waiting for voter to step into VL53L1X laser zone
  else if (state == 2) {
    if (distMm > 0 && distMm < boothOccMm) {
      closeGate();
      beep(1, 100);
      state = 3;
      IrReceiver.resume();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Select Candidate");
      lcd.setCursor(0, 1);
      lcd.print("1:A   2:B   3:C");
    }
  }

  // STATE 3: Voter inside booth, listening for IR remote vote (1, 2, or 3)
  else if (state == 3) {
    if (distMm > boothClearMm) {
      digitalWrite(ledPin, LOW);
      closeGate();
      state = 0;
      showIdleScreen();
      return;
    }

    if (IrReceiver.decode()) {
      uint16_t cmd = IrReceiver.decodedIRData.command;
      Serial.print("IR Command HEX: 0x");
      Serial.println(cmd, HEX);

      bool validVote = false;
      String choice = "";

      if (cmd == 0x45 || cmd == 0xC) {
        votesA++;
        choice = "Candidate A";
        validVote = true;
      } else if (cmd == 0x46 || cmd == 0x18) {
        votesB++;
        choice = "Candidate B";
        validVote = true;
      } else if (cmd == 0x47 || cmd == 0x5E) {
        votesC++;
        choice = "Candidate C";
        validVote = true;
      }

      IrReceiver.resume();

      if (validVote) {
        beep(2, 120);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Voted: " + choice);
        lcd.setCursor(0, 1);
        lcd.print("Exit Gate Open");

        printResultsSerial();
        openGate();
        state = 4;
        delay(800);
      }
    }
  }

  // STATE 4: Vote recorded, waiting for voter to leave laser zone
  else if (state == 4) {
    if (distMm > boothClearMm) {
      delay(400);
      closeGate();
      digitalWrite(ledPin, LOW);
      beep(1, 200);
      state = 0;
      showIdleScreen();
    }
  }

  delay(50);
}

// --- 360-Degree Gate Control ---
void openGate() {
  gateServo.write(125); // Spin open
  delay(spinTimeMs);
  gateServo.write(90);  // Stop
}

void closeGate() {
  gateServo.write(55);  // Spin closed
  delay(spinTimeMs);
  gateServo.write(90);  // Stop
}

// --- Other Helpers ---
int getUltrasonicCm() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 25000);
  if (duration == 0) return 0;
  return duration * 0.034 / 2;
}

void beep(int times, int durationMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(buzzerPin, LOW);
    delay(durationMs);
    digitalWrite(buzzerPin, HIGH);
    if (i < times - 1) delay(100);
  }
}

void showIdleScreen() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Voting Booth");
  lcd.setCursor(0, 1);
  lcd.print("Status: READY");
}

void printResultsSerial() {
  Serial.println("--- LIVE TALLY ---");
  Serial.print("A: "); Serial.print(votesA);
  Serial.print(" | B: "); Serial.print(votesB);
  Serial.print(" | C: "); Serial.println(votesC);
}