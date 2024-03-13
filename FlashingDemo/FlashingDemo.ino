// Pin number for the built-in LED (on most Arduino boards, this is pin 13)
#define NUMBER_OF_FLASHES 5 //Number of flashes per second, works up to 30 until it starts to get inconsistent
#define LED_PIN 13

// Variables to track time
unsigned long previousTimeFlash = 0;
const int interval = 1000/NUMBER_OF_FLASHES/2; // Calculate thlashes per second, and then dividing by 2 as each flash requires one ON pulse and one OFF pulse.


void setup() {
  // Initialize the built-in LED pin as an output
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  unsigned long currentTimeFlash = millis();

  if (currentTimeFlash >= interval + previousTimeFlash) { //
    previousTimeFlash = currentTimeFlash;

    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); //Sets the ledPin to whatever it currently is not.
  }
}
