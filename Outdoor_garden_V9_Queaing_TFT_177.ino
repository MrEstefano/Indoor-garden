/********************************************************************/
/*  Name    : Indoor Garden V7                                      */
/*  Author  : Stefan Zakutansky ATU.ie student                      */
/*  Date    : 15. 8.2023                                            */
/*  Notes   : Queuing architecture, to mantain favorable soil       */
/*            moisture condition, measuring moisture by capacitive  */
/*            sensor, and turning LOW active water pump,TFT display */
/********************************************************************/

#include <Adafruit_GFX.h>  
#include <Adafruit_Sensor.h>  
#include <Adafruit_ST7735.h> 
#include <Wire.h>
#include <SPI.h>

// Pin definitions for the 1.77-inch TFT (ST7735)
#define TFT_CS    6
#define TFT_RST   11  
#define TFT_DC    10 
#define TFT_SCLK 7   
#define TFT_MOSI 8

// constant variables
#define relayPin 2           /* Pump is contrpolled via PUMP+ on PCB*/
#define moisturePin A0        /* Capacitive moisture sensor is connected to SOIL on PCB*/
#define drySoil 800         /* dry soil moisture value from calibration*/
#define wetSoil 410          /* wet soil moisture value from calibration*/ 
#define moistureThreshold 85  /* treshold value from experiments for soil moisture */

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

//global variables
byte count =10; 
bool pumpState = false;
bool screen_initialized = false;
unsigned long moistureCheckInterval = 240000; // Interval to check moisture (4 minute) 
unsigned long pumpRunTime = 60000; // How long to run the pump (60 seconds)
unsigned long countdownTime = 0; // Holds the countdown time for display
bool isCountingDown = true; // Indicates if a countdown is active
int lastMoistureValue = 0; // Stores the last measured soil moisture value

// Structure for a task in the queue
struct Task {
  void (*function)(); // Pointer to the function to execute
  unsigned long executeTime; // Time when the task should be executed
  bool isCountingDown; // Start the countdow
};

// Queue for tasks
const int queueSize = 10;
Task taskQueue[queueSize];
int queueStart = 0;
int queueEnd = 0;

// Function to add a task to the queue
void enqueue(void (*function)(), unsigned long delayTime) {
  unsigned long currentTime = millis();
  taskQueue[queueEnd].function = function;
  taskQueue[queueEnd].executeTime = currentTime + delayTime;
  taskQueue[queueEnd].isCountingDown = isCountingDown;
  queueEnd = (queueEnd + 1) % queueSize;
  countdownTime = delayTime; // Set the countdown time  
}

// Function to execute and remove the first task in the queue
void dequeue() {
  if (queueStart != queueEnd) { // Check if the queue is not empty
    unsigned long currentTime = millis();
    if (currentTime >= taskQueue[queueStart].executeTime) {
      taskQueue[queueStart].function(); // Execute the task
      taskQueue[queueEnd].isCountingDown = isCountingDown;
      queueStart = (queueStart + 1) % queueSize; // Remove the task from the queue
      
    }
  }
}

// Task to check soil moisture
void checkMoisture() {
  isCountingDown=false;
  lastMoistureValue = analogRead(moisturePin); // Read the moisture level
  Serial.print("Soil Moisture Level: ");
  Serial.println(lastMoistureValue);
  lastMoistureValue = map(lastMoistureValue ,drySoil ,wetSoil ,0 ,100); // map the range in percantage
  Serial.print("Soil Moisture Level: ");
  Serial.print(lastMoistureValue);   
  Serial.println(" %");
  tft.fillRect(65,5,25,25,ST7735_BLACK);  //clear moisture field
  tft.fillRect(5,105,128,25,ST7735_BLACK);  //task 
  //Check if below treshold
  if (lastMoistureValue < moistureThreshold && !pumpState) {
    Serial.println("Moisture low. Turning on the pump...");
    digitalWrite(relayPin, LOW); // Turn on the relay (active low)
    // enqueue(stopPump, pumpRunTime); // Schedule a task to stop the pump after 30 seconds
    Serial.println("Soil Moisture low. Turning on the pump...");
    digitalWrite(relayPin, LOW); // Turn on the relay (active low) 
    pumpState = true;
    isCountingDown = true;
    enqueue(stopPump, pumpRunTime); // Schedule a task to stop the pump after 30 seconds    
    enqueue(checkMoisture, moistureCheckInterval); // Re-schedule the moisture check task 
  }
  else{
    isCountingDown = true;
  Serial.println("Soil Moisture above treashold. Lets check again...in 4 minutes ");
  enqueue(checkMoisture, moistureCheckInterval); // Re-schedule the moisture check task 
  }
}

// Task to stop the pump
void stopPump() {
  if(pumpState){
    isCountingDown = true; // Stop the countdown after task execution
  Serial.println("Turning off the pump..."); 
  digitalWrite(relayPin, HIGH); // Turn off the relay 
  pumpState = false;
  tft.fillRect(5,105,128,25,ST7735_BLACK);  //task 
  }
}

// Function to update the TFT screen with the countdown and moisture value
void updateDisplay() {
  if (isCountingDown) {
    unsigned long currentTime = millis();
    unsigned long timeRemaining = (taskQueue[queueStart].executeTime > currentTime) ? 
                                  (taskQueue[queueStart].executeTime - currentTime) : 0;
    //1st line
    tft.setTextSize(2);
    tft.setCursor(5, 5);
    tft.println("SOIL");
    tft.setCursor(65, 5);
    tft.println(lastMoistureValue);
    tft.setCursor(105, 5);
    tft.println("%");
    //second linre
    tft.setCursor(5, 30);
    tft.println("ONSET");
    tft.setCursor(75, 30);
    tft.println(moistureThreshold);
    tft.setCursor(105, 30);
    tft.println("%");
    //third line
    tft.setCursor(5, 55);
    tft.setTextSize(2);
    tft.println("TIME");
    tft.setCursor(60, 55);
    tft.println(timeRemaining / 1000); // Display countdown in seconds
    tft.setCursor(100, 55);
    tft.println("s");
    //4th line
    tft.setCursor(5, 80);
    tft.setTextSize(2);
    tft.println("NEXT:"); 
    //5th line
    if (taskQueue[queueStart].function == checkMoisture) {
      delay(500);
      tft.fillRect(60,55,50,25,ST7735_BLACK);  //countdown  
      tft.setCursor(5, 105);
      tft.println("Soil Check");
    } else if (taskQueue[queueStart].function == stopPump) {
      delay(500);
      tft.fillRect(60,55,50,25,ST7735_BLACK);  //countdown   
      tft.setCursor(5, 105);
      tft.println("Stop Pump");
    }
  }
}

void startIpDisplay(){
  tft.drawRoundRect(40, 90, 40,25,5, ST7735_BLACK);
  tft.fillRoundRect(40, 90, 40,25,5, ST7735_BLACK);
  tft.setCursor(55, 95);
  tft.println(count);
  switch (count){
    case 9: tft.fillRect(22, 128, 8, 19,ST7735_RED);
      break;
    case 8: tft.fillRect(33, 128, 8, 19,ST7735_YELLOW); 
      break;
    case 7:  tft.fillRect(44, 128, 8, 19,ST7735_YELLOW);
      break;
    case 6: tft.fillRect(55, 128, 8, 19,ST7735_GREEN);
      break;
    case 5: tft.fillRect(66, 128, 8, 19,ST7735_GREEN);
      break;
    case 4: tft.fillRect(77, 128, 8, 19,ST7735_GREEN);
      break;
    case 3: tft.fillRect(88, 128, 8, 19,ST7735_GREEN);
      break;
    case 2: tft.fillRect(98, 128, 8, 19,ST7735_GREEN);
      break;   
    case 1: tft.fillRect(108, 128, 8, 19,ST7735_GREEN);
      break;
    default:
      break;                                               
  }   
}

void setup() {
  Serial.begin(9600);
  pinMode(moisturePin, INPUT);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH); // Initially turn off the pump
  /* Initialize TFT 1.77 inch screen*/
  tft.initR(INITR_BLACKTAB);    
  tft.fillScreen(ST7735_BLACK); 
  //run intro graphics
  tft.drawRoundRect(3, 5, 122, 150,5, ST7735_BLUE);
  tft.setCursor(30, 20);
  tft.setTextColor(ST7735_GREEN);
  tft.setTextSize(2);
  tft.println("SYSTEM");
  tft.setCursor(30, 45);
  tft.println("STARTS");
  tft.setCursor(50, 70);
  tft.println("IN");
  tft.drawRect(8 , 125, 112, 25,ST7735_BLUE);
  tft.fillRect(11, 128, 8, 19, ST7735_RED);
    for (byte i = 10;count > 0;i--){
    count = count - 1;
    delay (250);
    startIpDisplay();    
  }
  tft.fillScreen(ST7735_BLACK); // Clear the screen (white background)
  // Schedule the first moisture check
  enqueue(checkMoisture, 0); // Start immediately
}

void loop() {
  dequeue(); // Execute tasks as their time comes
  updateDisplay(); // Update the display with the countdown and moisture value
}