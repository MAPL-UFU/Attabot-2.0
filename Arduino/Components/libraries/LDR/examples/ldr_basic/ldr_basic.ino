/* 
PINOUT LDR MODULE
GND: GND
VCC: 5V
DO: Digital Port 3 (3)
*/

// Define the LDR analog port
const int sensorPin = A0;

// Declare the variable to store the LDR state
unsigned int ldrSensorState;

void setup() {
  // Initiate the serial communication
  Serial.begin(57600);
  
  // Set the LDR pin as an input
  pinMode(sensorPin, INPUT);
}

void loop() {
  // Read the LDR sensor value and store it
  ldrSensorState = analogRead(sensorPin);
  
  // Print the LDR sensor value for debugging purposes
  /* Here it's possible to determine the light intensity
     that changes the sensor state */
     
  Serial.print("Sensor value: "); 
  Serial.println(ldrSensorState);

  // Check if the LDR sensor value is below a threshold determined using the ldrSensorState
  if (ldrSensorState < 200){
    Serial.println("Event: light detected"); // Send an event message
  }
  
  // Wait for a second
  delay(1000); 
}
