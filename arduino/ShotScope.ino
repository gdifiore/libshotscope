void setup() {
  // Initialize the serial communication at 9600 baud rate
  Serial.begin(9600);
}

void loop() {
  // Print a message to the serial monitor
  Serial.println("Hello, world!");

  // Wait for 1 second
  delay(1000);
}