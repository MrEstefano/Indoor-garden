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

#define TFT_CS    6
#define TFT_RST   11  
#define TFT_DC    10 
#define TFT_SCLK 7   
#define TFT_MOSI 8

#define relayPin 2           /* Pump is contrpolled via PUMP+ on PCB*/
#define moisturePin A0        /* Capacitive moisture sensor is connected to SOIL on PCB*/
#define drySoil 800         /* dry soil moisture value from calibration*/
#define wetSoil 410          /* wet soil moisture value from calibration*/ 
#define moistureThreshold 75  /* treshold value from experiments for soil moisture */

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
  
const unsigned long moistureCheckInterval = 240000; // Interval to check moisture (4 minute) 
const unsigned long pumpRunTime = 90000; // How long to run the pump (90 seconds)
unsigned long startTime;
unsigned long currentTime;
unsigned long elapsedTime;
bool pumpState = false;
byte count =10; 
bool t =0;
int soilHumidity;

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

// Function to add a task to the queue 
void enqueue(void (*function)(), unsigned long delayTime) { 
  unsigned long currentTime = millis(); 
  taskQueue[queueEnd].function = function; 
  taskQueue[queueEnd].executeTime = currentTime + delayTime; 
  queueEnd = (queueEnd + 1) % queueSize; 
} 

// Function to execute and remove the first task in the queue 
void dequeue() { 
  if (queueStart != queueEnd) { 
    // Check if the queue is not empty 
    unsigned long currentTime = millis();
    if (currentTime >= taskQueue[queueStart].executeTime) { 
      taskQueue[queueStart].function(); // Execute the task 
      queueStart = (queueStart + 1) % queueSize; // Remove the task from the queue 
    }
  }
} 

// Task to check soil moisture 
void checkMoisture() { 
  int moistureValue = analogRead(moisturePin); // Read the capacitive sensor
  //moistureValue = constrain(moistureValue,drySoil,wetSoil); // needed to eliminate random readings outside the dry-wet boundries
  Serial.print("Raw Moisture Value : ");
  Serial.println(moistureValue);  
  moistureValue = map(moistureValue ,drySoil ,wetSoil ,0 ,100); // map the range in percantage
  Serial.print("Soil Moisture Level: ");
  Serial.print(moistureValue);   
  Serial.println(" %");
  soilHumidity=moistureValue;
  enqueue(updateTFT,moistureCheckInterval);
  //check the moisture percentage with treashold value and last state of the pump
  if (moistureValue < moistureThreshold && !pumpState) { 
    Serial.println("Soil Moisture low. Turning on the pump...");
    digitalWrite(relayPin, LOW); // Turn on the relay (active low) 
    pumpState = true;
    enqueue(stopPump, pumpRunTime); // Schedule a task to stop the pump after 30 seconds    
    enqueue(checkMoisture, moistureCheckInterval); // Re-schedule the moisture check task 
    startTime = millis(); // Record the starting time
    enqueue(countDown,moistureCheckInterval); 
    
  }
  else{
  Serial.println("Soil Moisture above treashold. Lets check again...in 4 minutes ");
  enqueue(checkMoisture, moistureCheckInterval); // Re-schedule the moisture check task 
  startTime = millis(); // Record the starting time
  enqueue(countDown,moistureCheckInterval);
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

void display(){
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

void countDown (){
  currentTime = millis(); // Get the current time
  elapsedTime =(currentTime - startTime) / 1000; // Calculate elapsed time in seconds
  if (elapsedTime <= moistureCheckInterval) {
    static unsigned long remainingTime = moistureCheckInterval - elapsedTime;
    static unsigned int minutes = remainingTime / 60;
    static unsigned int seconds = remainingTime % 60;    
    tft.setTextSize(2);
    tft.fillRect(0,80,128,105,ST7735_BLACK);
    tft.setCursor(5, 80);
    tft.println("0");
    tft.setCursor(18, 80);
    tft.println(minutes);
    tft.setCursor(28, 80);
    tft.println(":");
    tft.setCursor(38, 80);
    tft.println(seconds);
    
  } 
  enqueue(countDown,1000); 
  Serial.println("Count down has been updated..."); 
}

void updateTFT(){  
  tft.fillScreen(ST7735_BLACK);
  tft.setTextSize(2);
  tft.setCursor(5, 5);
  tft.println("SOIL");
  tft.setCursor(65, 5);
  tft.println(soilHumidity);
  tft.setCursor(105, 5);
  tft.println("%");

  tft.setCursor(5, 30);
  tft.println("ONSET");
  tft.setCursor(75, 30);
  tft.println(moistureThreshold);
  tft.setCursor(105, 30);
  tft.println("%");
  tft.setCursor(5, 55);
  tft.println("MONITORING"); 
  Serial.println("Screen has been updated...");  


}

void setup() { 
  Serial.begin(112500); 
  pinMode(moisturePin, INPUT); 
  pinMode(relayPin, OUTPUT); 
  digitalWrite(relayPin, HIGH); // Initially turn off the pump 
  
  tft.initR(INITR_BLACKTAB);    /* Initialize TFT 1.77 inch screen*/
  tft.fillScreen(ST7735_BLACK); 
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
    display();
    
  }
  // Schedule the first moisture check 
  enqueue(checkMoisture, 0); // Start immediately 
  // Shedule first TFT screen update 
  startTime = millis(); // Record the starting time
  enqueue(countDown,0); 
  enqueue(updateTFT,0);
} 

void loop() {   
  dequeue(); // Execute tasks as their time comes
}
