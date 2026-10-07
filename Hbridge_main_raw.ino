// =====================================================
// EEE-4202 INDUSTRIAL ELECTRONICS & DRIVES
// SYMMETRIC PINOUT: LEFT DRIVERS (12/11), RIGHT DRIVERS (3/2)
// ENCODERS ON D6/D5 (PIN CHANGE INTERRUPTS)
// =====================================================

// --- 1. DRIVER PINS (SYMMETRIC LEFT/RIGHT) ---
const int leftPMOS_BJT  = 9; // Left High-Side BJT Level Shifter
const int leftNMOS      = 5; // Left Low-Side NMOS PWM

const int rightPMOS_BJT = 13;  // Right High-Side BJT Level Shifter
const int rightNMOS     = 6;  // Right Low-Side NMOS PWM

// --- 2. MIDDLE LOGIC PINS (BUTTONS & ENCODERS) ---
const int btnForward  = 4;
const int btnStop     = 7;
const int btnBackward = 8;

const int encoderCHA  = 2;    // Encoder Channel A (Pin Change INT)
const int encoderCHB  = 3;    // Encoder Channel B (Pin Change INT)

// --- 3. SEPARATE ANALOG INPUT ---
const int potPin      = A0;   // Speed Target Reference

// --- CONTROL CONSTANTS & SYSTEM STATE ---
const int maxSafePWM = 244;
enum MotorState { IDLE, FORWARD, BACKWARD };
MotorState systemState = IDLE;

int targetSpeedPWM = 0;
int currentSpeedPWM = 0;

volatile long pulseCount = 0;
unsigned long lastObserverCheck = 0;
const unsigned long sampleInterval = 100; // 100ms telemetry window

// Function Prototypes
void gradualDeceleration();
void allPinsLow();

// ================= SETUP =================
void setup() {
  Serial.begin(9600);
  Serial.println("--- Symmetric H-Bridge Control (Encoders D6/D5) Active ---");

  // Output Drivers
  pinMode(leftPMOS_BJT, OUTPUT);
  pinMode(leftNMOS, OUTPUT);
  pinMode(rightPMOS_BJT, OUTPUT);
  pinMode(rightNMOS, OUTPUT);

  // Buttons (Internal Pull-ups)
  pinMode(btnForward, INPUT_PULLUP);
  pinMode(btnStop, INPUT_PULLUP);
  pinMode(btnBackward, INPUT_PULLUP);

  // Encoders (Internal Pull-ups)
  pinMode(encoderCHA, INPUT_PULLUP);
  pinMode(encoderCHB, INPUT_PULLUP);

  // Enable Pin Change Interrupts on D5 and D6 (Port D: PCINT21 & PCINT22)
  PCICR |= (1 << PCIE2);     // Enable PCINT2 interrupt vector (Pins D0-D7)
  PCMSK2 |= (1 << PCINT21) | (1 << PCINT22); // Enable mask for D5 and D6

  allPinsLow();
}

// ================= MAIN LOOP =================
void loop() {
  // Read Potentiometer
  int potValue = analogRead(potPin);
  targetSpeedPWM = map(potValue, 0, 1023, 0, 255);

  unsigned long currentTime = millis();

  // 100ms Observer Window for RPM Telemetry
  if (currentTime - lastObserverCheck >= sampleInterval) {
    noInterrupts();
    long currentPulses = abs(pulseCount);
    pulseCount = 0; // Clear sample window counter
    interrupts();

    long measuredRPM = 0;
    if (systemState != IDLE) {
      measuredRPM = (currentPulses * 600L) / 7L; // Standard 7 PPR formula
    }

    Serial.print("Target PWM: ");
    Serial.print(targetSpeedPWM);
    Serial.print(" | State: ");
    if (systemState == FORWARD) Serial.print("FORWARD ");
    else if (systemState == BACKWARD) Serial.print("BACKWARD");
    else Serial.print("IDLE    ");

    Serial.print(" | Measured Speed: ");
    Serial.print(measuredRPM);
    Serial.println(" RPM");

    // PWM Safety Protection
    if (targetSpeedPWM > maxSafePWM && systemState != IDLE) {
      Serial.println("!!! PWM SAFETY LIMIT EXCEEDED - BRAKING !!!");
      gradualDeceleration();
    }

    lastObserverCheck = currentTime;
  }

  // Button Inputs
  if (digitalRead(btnForward) == LOW && systemState == IDLE) {
    systemState = FORWARD;
  } else if (digitalRead(btnBackward) == LOW && systemState == IDLE) {
    systemState = BACKWARD;
  } else if (digitalRead(btnStop) == LOW && systemState != IDLE) {
    gradualDeceleration();
  }

  // H-Bridge Execution
  switch (systemState) {
    case FORWARD:
      digitalWrite(leftPMOS_BJT, HIGH);
      digitalWrite(rightPMOS_BJT, LOW);
      digitalWrite(leftNMOS, LOW);

      currentSpeedPWM = targetSpeedPWM;
      analogWrite(rightNMOS, currentSpeedPWM); // Output to D2
      break;

    case BACKWARD:
      digitalWrite(leftPMOS_BJT, LOW);
      digitalWrite(rightPMOS_BJT, HIGH);
      digitalWrite(rightNMOS, LOW);

      currentSpeedPWM = targetSpeedPWM;
      analogWrite(leftNMOS, currentSpeedPWM); // Output to D11
      break;

    case IDLE:
      allPinsLow();
      break;
  }
}

// ================= PIN CHANGE INTERRUPT SERVICE ROUTINE =================
ISR(PCINT2_vect) {
  static int lastEncoded = 0;
  int MSB = digitalRead(encoderCHA);
  int LSB = digitalRead(encoderCHB);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) pulseCount++;
  if (sum == 0b1110 || sum == 0b1000 || sum == 0b0001 || sum == 0b0111) pulseCount--;

  lastEncoded = encoded;
}

// ================= SAFETY FUNCTIONS =================
void gradualDeceleration() {
  for (int speed = currentSpeedPWM; speed >= 0; speed--) {
    if (systemState == FORWARD) analogWrite(rightNMOS, speed);
    else if (systemState == BACKWARD) analogWrite(leftNMOS, speed);
    delay(50);
  }
  currentSpeedPWM = 0;
  systemState = IDLE;
  allPinsLow();
}

void allPinsLow() {
  digitalWrite(leftPMOS_BJT, LOW);
  digitalWrite(leftNMOS, LOW);
  digitalWrite(rightPMOS_BJT, LOW);
  digitalWrite(rightNMOS, LOW);
}