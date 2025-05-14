// Motion Parameters
double d = 5; // Distance to travel (ft)
double r = 2.75; // Wheel Radius (Inches)
double maxAccel = 12.5; // inch/sec²

// Motor Control Variables
volatile long int count = 0;
int v_out = 30;  // Initial PWM value
double w = 0.0, wtarget = 0.0, error = 0.0;
double Kp = 0.2, Ki = 0.1, Kd = 0.2;  // PID gains
double integral = 0.0, derivative = 0.0, lastError = 0.0;
double dv = 25.0;
unsigned long int t0, t;

// Motion Profile Calculations
double interval = 1000 * (sqrt((12 * d) / maxAccel)); // Time to reach max speed (ms)
double maxSpeed = interval * maxAccel / 1000; // Max linear speed (inches/sec)
double wmax = 60 * maxSpeed / (2 * 3.1415 * r); // Max angular speed (rpm)

// Motor Pins
const int ENA_PIN = 5;   // ENA (PWM) pin for speed control
const int LN1_PIN = 6;   // Motor direction pin 1
const int LN2_PIN = 7;   // Motor direction pin 2

// Interrupt function for encoder
void encoderISR() {
  if (digitalRead(3) == HIGH)
    count++;
  else
    count--;
}

// Function to calculate motor speed from encoder counts
double getSpeed() {
  long int count0, count1;
  double theta0, theta1;

  noInterrupts();
  count0 = count;
  interrupts();

  theta0 = count0 * 1.5; // Convert counts to degrees
  delay(50);

  noInterrupts();
  count1 = count;
  interrupts();

  theta1 = count1 * 1.5;

  w = abs((theta1 - theta0) / 0.05); // Convert to degrees/sec
  return (w / 360.0 * 60.0); // Convert to RPM
}

void setup() {
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), encoderISR, RISING);
  Serial.begin(9600);

  pinMode(ENA_PIN, OUTPUT);
  pinMode(LN1_PIN, OUTPUT);
  pinMode(LN2_PIN, OUTPUT);

  digitalWrite(LN1_PIN, HIGH);
  digitalWrite(LN2_PIN, LOW);
  analogWrite(ENA_PIN, v_out); // Start motor with initial speed

  t0 = millis();
}

void loop() {
   t = millis() - t0;
   
  // Motion profile control
  if (t < interval) {
    wtarget = wmax * (double(t) / interval);
    digitalWrite(LN1_PIN, HIGH);
    digitalWrite(LN2_PIN, LOW); // Forward
  }
  else if (t < 2 * interval) {
    wtarget = wmax * (1 - double(t - interval) / interval);
    digitalWrite(LN1_PIN, HIGH);
    digitalWrite(LN2_PIN, LOW); // Forward
  }
  else if (t < 3 * interval){
    wtarget = (wmax * ((double(t) - 2 * interval) / interval));
    digitalWrite(LN1_PIN, LOW);
    digitalWrite(LN2_PIN, HIGH); // Reverse
  }
  else if (t < 4 * interval){
    wtarget = (wmax * (1 - double(t - 3 * interval) / interval));
    digitalWrite(LN1_PIN, LOW);
    digitalWrite(LN2_PIN, HIGH); // Reverse
  }
  else {
    v_out = 0;
    wtarget = 0;
    analogWrite(ENA_PIN, 0);  // Ensure PWM is set to 0
    digitalWrite(LN1_PIN, LOW); // Make sure both motor direction pins are LOW
    digitalWrite(LN2_PIN, LOW);
    return; // Exit loop iteration early to prevent further updates
}

  // Get actual speed from encoder
  w = getSpeed();
  
  // PID Speed Control
  error = wtarget - w;
  integral += error * 0.05;
  derivative = (error - lastError) / 0.05;
  dv = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  v_out = v_out + int(dv);
  v_out = constrain(v_out, 0, 255);

  analogWrite(ENA_PIN, v_out); // Set motor speed

  // Debug Output
  //Serial.print("Time: "); Serial.print(t);
  //Serial.print(" | wtarget: "); Serial.print(wtarget);
  //Serial.print(" | w: "); Serial.print(w);
  //Serial.print(" | v_out: "); Serial.println(v_out);
  //Serial.print("Count = "); Serial.println(count);

  Serial.print("w:");    // Label for variable w
  Serial.print(w);       // Value of w (motor speed)
  Serial.print("\t");    // Tab to separate the variables
  Serial.print("wtarget:");  // Label for variable wtarget
  Serial.println(wtarget);   // Value of wtarget (target speed)


  delay(10);
}
