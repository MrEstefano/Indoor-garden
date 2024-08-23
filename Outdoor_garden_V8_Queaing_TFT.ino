/********************************************************************/
/*  Name    : Indoor Garden V8                                      */
/*  Author  : Stefan Zakutansky ATU.ie student                      */
/*  Date    : 15. 8.2023                                            */
/*  Notes   : Queuing architecture, to mantain favorable soil       */
/*            moisture condition, measuring moisture by capacitive  */
/*            sensor, and turning LOW active water pump,TFT display */
/********************************************************************/

#include <Adafruit_GFX.h>  
#include <Adafruit_TFTLCD.h>  
#include <SPI.h>

// Assign human-readable names to some common 16-bit color values:
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

// Pin definitions for the TFT
#define CS 6
#define DC 10
#define RST 11
#define MOSI 8
#define SCK 7
#define MISO 12

Adafruit_TFTLCD tft(CS, DC, MOSI, MISO, RST);



#define relayPin 2           /* Pump is contrpolled via PUMP+ on PCB*/
#define moisturePin A0        /* Capacitive moisture sensor is connected to SOIL on PCB*/
#define drySoil 800         /* dry soil moisture value from calibration*/
#define wetSoil 410          /* wet soil moisture value from calibration*/ 
#define moistureThreshold 75  /* treshold value from experiments for soil moisture */

  
const unsigned long moistureCheckInterval = 240000; // Interval to check moisture (4 minute) 
const unsigned long pumpRunTime = 90000; // How long to run the pump (90 seconds)
unsigned long startTime;
unsigned long currentTime;
unsigned long elapsedTime;
bool pumpState = false;
byte count =10; 
bool t =0;


// Structure for a task in the queue to add a delay (the execution time)
struct Task { 
  void (*function)(); // Pointer to the function to execute 
  unsigned long executeTime; // Time when the task should be executed 
}; 

// Queue for tasks 
const int queueSize = 10; 
Task taskQueue[queueSize]; 
int queueStart = 0; 
int queueEnd = 0; 
unsigned long countdownTime = 0; // Holds the countdown time for display
bool isCountingDown = false; // Indicates if a countdown is active
int moistureValue = 0; // Stores the last measured soil moisture value

// Function to add a task to the queue 
void enqueue(void (*function)(), unsigned long delayTime) { 
  unsigned long currentTime = millis(); 
  taskQueue[queueEnd].function = function; 
  taskQueue[queueEnd].executeTime = currentTime + delayTime; 
  queueEnd = (queueEnd + 1) % queueSize; 
  countdownTime = delayTime; // Set the countdown time
  isCountingDown = true; // Start the countdown
} 

// Function to execute and remove the first task in the queue 
void dequeue() { 
  if (queueStart != queueEnd) { // Check if the queue is not empty 
    unsigned long currentTime = millis();
    if (currentTime >= taskQueue[queueStart].executeTime) { 
      taskQueue[queueStart].function(); // Execute the task 
      queueStart = (queueStart + 1) % queueSize; // Remove the task from the queue 
      isCountingDown = false; // Stop the countdown after task execution
    }
  }
} 


// Task to check soil moisture 
void checkMoisture() { 
  moistureValue = analogRead(moisturePin); // Read the capacitive sensor
  //moistureValue = constrain(moistureValue,drySoil,wetSoil); // needed to eliminate random readings outside the dry-wet boundries
  Serial.print("Raw Moisture Value : ");
  Serial.println(moistureValue);  
  moistureValue = map(moistureValue ,drySoil ,wetSoil ,0 ,100); // map the range in percantage
  Serial.print("Soil Moisture Level: ");
  Serial.print(moistureValue);   
  Serial.println(" %");
  
  //check the moisture percentage with treashold value and last state of the pump
  if (moistureValue < moistureThreshold && !pumpState) { 
    Serial.println("Soil Moisture low. Turning on the pump...");
    digitalWrite(relayPin, LOW); // Turn on the relay (active low) 
    pumpState = true;
    enqueue(stopPump, pumpRunTime); // Schedule a task to stop the pump after 30 seconds    
    enqueue(checkMoisture, moistureCheckInterval); // Re-schedule the moisture check task     
  }
  else{
  Serial.println("Soil Moisture above treashold. Lets check again...in 4 minutes ");
  enqueue(checkMoisture, moistureCheckInterval); // Re-schedule the moisture check task 
  } 
} 
// Function to update the TFT screen with the countdown and moisture value
void updateDisplay() {
  if (isCountingDown) {
    unsigned long currentTime = millis();
    unsigned long timeRemaining = (taskQueue[queueStart].executeTime > currentTime) ?
    (taskQueue[queueStart].executeTime - currentTime) : 0;
    tft.fillScreen(BLACK); // Clear the screen (white background)
    // Display countdown
    tft.setCursor(10, 10);
    tft.setTextColor(CYAN); // Black text
    tft.setTextSize(2);
    tft.print("Countdown: ");
    tft.print(timeRemaining / 1000); // Display countdown in seconds
    tft.print("sec");
    // Display next task
    tft.setCursor(10, 40);
    if (taskQueue[queueStart].function == checkMoisture) {
      tft.print("Next: Moisture Check");
      } 
    else if (taskQueue[queueStart].function == stopPump) {
      tft.print("Next: Stop Pump");
    }
    // Display last soil moisture value
    tft.setCursor(10, 70);
    tft.print("Moisture: ");
    tft.print(moistureValue);
  }
}
// Task to stop the pump 
void stopPump() { 
  if(pumpState){
  Serial.println("Turning off the pump..."); 
  digitalWrite(relayPin, HIGH); // Turn off the relay 
  pumpState = false;
  }
} 


void setup() { 
  Serial.begin(112500); 
  pinMode(moisturePin, INPUT); 
  pinMode(relayPin, OUTPUT); 
  digitalWrite(relayPin, HIGH); // Initially turn off the pump 
  tft.begin(0x9341); // Initialize with the display&#39;s identifier
  tft.setRotation(1); // Adjust rotation as needed
  tft.fillScreen(WHITE); // Clear the screen to white
  // Schedule the first moisture check 
  enqueue(checkMoisture, 0); // Start immediately 
 
} 

void loop() {   
  dequeue(); // Execute tasks as their time comes
  updateDisplay(); // Update the display with the countdown and moisture value
}
