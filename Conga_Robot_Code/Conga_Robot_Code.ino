#define ROBOT_NUM 2

// ================================================================================
////Variables for movment of the robots
unsigned long previousTimeRun = 0;
const long runInterval = 500;  // Interval in milliseconds (0.5 seconds)
unsigned long currentTime = 0;

//Pins for Motors
#define MOT_A1_PIN 5
#define MOT_A2_PIN 6
#define MOT_B1_PIN 9
#define MOT_B2_PIN 10

//Initisalises the variables to hold the sensor values
int sensDiffThresh = 45;
int SensorLeft = 0;
int SensorRight = 0;

int LeftSensor = A1; // This stores the value of the Left Sensor pin to use later on in the sketch
int RightSensor = A2; // This stores the value of the Right Sensor pin to use later on in the sketch
const int slowSpeed = 20; // the range for speed is(0,255)
const int maxSpeed = 100; // the range for speed is(0,255)
const float percentMotorDecrease = 0.20; // Percentage that the right motor is faster/slower than the left

// Define variables for the LDR Sensor readings and the difference between the two
int SensorDifference = 0; // This value is used to determine the difference between the Left and Right
int error = 40;// difference between motors for turning

// Variables to track current speed of motors
int leftServoSpeed = 0;
int rightServoSpeed = 0;

// ================================================================================
////Variables for flashing

#define NUMBER_OF_FLASHES (ROBOT_NUM) //Number of flashes per second, works up to 30 until it starts to get inconsistent
#define LED_PIN 13

unsigned long previousTimeFlash = 0; //Tracks time that the time since last change of state of the LEDs
const int interval = 1000/NUMBER_OF_FLASHES/2; // Calculate thlashes per second, and then dividing by 2 as each flash requires one ON pulse and one OFF pulse.

// ================================================================================
////Variables for reading if flashes are the correct code

#define SensorCentre A0

unsigned long startTime = 0;
int flashCount = 0;
int lightThreshold = 400; // Threshold as what counts as a flash
bool currentlyOn = false; // Indicated if the current flash has already been detected

bool correctSignal = false;

// ================================================================================
/// Configure the hardware once after booting up.  This runs once after pressing
//// reset or powering up the board.
void setup(void)
{
  // Initialize the stepper driver control pins to output drive mode.
  pinMode(MOT_A1_PIN, OUTPUT);
  pinMode(MOT_A2_PIN, OUTPUT);
  pinMode(MOT_B1_PIN, OUTPUT);
  pinMode(MOT_B2_PIN, OUTPUT);

  // Start with drivers off, motors coasting.
  digitalWrite(MOT_A1_PIN, LOW);
  digitalWrite(MOT_A2_PIN, LOW);
  digitalWrite(MOT_B1_PIN, LOW);
  digitalWrite(MOT_B2_PIN, LOW);

  // Initialize LEDs as an output
  pinMode(LED_PIN, OUTPUT);

  // Initialize the serial UART at 9600 bits per second.
  Serial.begin(115200);
}
// ================================================================================

void set_motor_pwm(int pwm, int IN1_PIN, int IN2_PIN)
{
  if (pwm < 0) {  // reverse speeds
    analogWrite(IN1_PIN, -pwm);
    digitalWrite(IN2_PIN, LOW);

  } else { // stop or forward
    digitalWrite(IN1_PIN, LOW);
    analogWrite(IN2_PIN, pwm);
  }
}
// ================================================================================
/// Set the current on both motors.

void set_motor_currents(int pwm_A, int pwm_B)
{
  set_motor_pwm(pwm_A, MOT_A1_PIN, MOT_A2_PIN);
  set_motor_pwm(pwm_B, MOT_B1_PIN, MOT_B2_PIN);
}

// ================================================================================

void spin(int pwm_A, int pwm_B)
{
  set_motor_currents(pwm_A, pwm_B);
}

// ===============================================================================
void Flash() {
  unsigned long currentTimeFlash = millis();

  if (currentTimeFlash >= interval + previousTimeFlash) { //
    previousTimeFlash = currentTimeFlash;

    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); //Sets the ledPin to whatever it currently is not.
  }
}
// ===============================================================================

void FlashCounter() {
  unsigned long currentTimeCount = millis(); //Stores start time

  int lightIntensity = analogRead(SensorCentre); //Loads litgh
  if (lightIntensity > lightThreshold && !currentlyOn intesity
) { //If light is above threshold and has not been previously detected in this cycle
    flashCount++;
    currentlyOn = true;
  } else if (lightIntensity <= lightThreshold) {
    currentlyOn = false;
  }


  if (currentTimeCount - startTime >= 1000) { //If a second has passed since start time
    Serial.println("Flashes per second: " + String(flashCount));
    if(flashCount==(ROBOT_NUM-1) || (ROBOT_NUM-1)==0){
      correctSignal = true;
  }
    else{
      correctSignal = false;
    }
  startTime = currentTime;
  flashCount = 0;
}
}


// ===============================================================================

void updateDirection()
{
  SensorLeft = 1023 - analogRead(LeftSensor); // Reads LDR sensor circuit 

  //delay(1); // delay to let adc settle

  SensorRight = 1023 - analogRead(RightSensor); // Reads LDR sensor circuit 

  //delay(1); // delay to let adc settle

  SensorDifference = abs(SensorLeft - SensorRight); // Calculates differance between the two values

  // This section of the sketch is what actually interprets the data and then runs the motors accordingly.

  if ((SensorLeft > SensorRight) && (SensorDifference > sensDiffThresh))
  { // If left is more bright and the difference between the two sensors is above the threshold, Do:
    leftServoSpeed = slowSpeed; //Sets left to slow speed
    rightServoSpeed = maxSpeed-(maxSpeed*percentMotorDecrease); //Sets right to max speed (with percentage increase to fix differing speed issues
  }

  if ((SensorLeft < SensorRight) && (SensorDifference > sensDiffThresh))
  { // If left is more bright and the difference between the two sensors is above the threshold, Do:
    leftServoSpeed = maxSpeed; //Sets left to high speed
    rightServoSpeed = slowSpeed-(maxSpeed*percentMotorDecrease); //Sets right to slow speed (with percentage increase to fix differing speed issues
  }
  else if (SensorDifference < sensDiffThresh)
  { // If Sensor Differance is below threshold, Do:
    leftServoSpeed = maxSpeed; //Full Speed Foward
    rightServoSpeed = maxSpeed-(maxSpeed*percentMotorDecrease); // Full speed foward (with modifier)
  }
}

// ===============================================================================

void loop()
{
  currentTime = millis();  // Get the current time
  
  Flash();
  
  FlashCounter();
  
  //updateDirection();

  //spin(leftServoSpeed,rightServoSpeed);
  
  if ((currentTime - previousTimeRun) >= runInterval)
  {
    previousTimeRun = currentTime;
    
    if (correctSignal == true)
    {
      updateDirection();
      
      spin(leftServoSpeed, rightServoSpeed);
      
      Serial.println("Updated Direction and speed");
    }
    else
    {
      spin(0, 0);
      
      Serial.println("Halt");
    }
  }
}
