#define LDR_PIN A0

// Variables to track light detection
unsigned long startTime = 0;
int flashCount = 0;
int lightThreshold = 400; // Threshold as what counts as a flash
bool currentlyOn = false; // Indicated if the current flash has already been detected

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);

  // Set the LDR pin as an input
  pinMode(LDR_PIN, INPUT);
}

void loop() {
  unsigned long currentTime = millis(); //Stores start time

  int lightIntensity = analogRead(LDR_PIN); //Loads light intesity

  if (lightIntensity > lightThreshold && !currentlyOn) { //If light is above threshold and has not been previously detected in this cycle
    flashCount++;
    currentlyOn = true;
  } else if (lightIntensity <= lightThreshold) {
    currentlyOn = false;
  }


  if (currentTime - startTime >= 1000) { //If a second has passed since start time
    Serial.println("Flashes per second: " + String(flashCount));
    flashCount = 0;
    startTime = currentTime;
  }
}
