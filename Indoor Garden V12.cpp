/**************************************************************************************/
/*  Name    : Indoor Garden V12                                                       */
/*  Author  : Stefan Zakutansky ATU.ie student                                        */
/*  Date    : 25. 8.2023                                                              */
/*  Notes   : The code operates on Queuing architecture to faciliate a "mutex" for    */
/*            global variables. The system measures a soil moisture by capacitive   	*/
/*            sensor. The values are computed by PID alghoritm to calculate           */
/*            a pump runtime acordingly. The sampling rate of 4 minutes provides      */
/*            favorable soil condition to mantain healthy growth of the plant.        */
/*            The pump is active LOW. The Countdown for each task currently running is*/
/*            displayed, same as soil moisture, setpoint and total pump runtime       */
/*            per day on a TFT display. User can intercat with encoder push button,   */
/*            in order to orientate in short menu to adjust the treashold value and   */
/*            dayly water limit                                                       */
/**************************************************************************************/

#include <Adafruit_GFX.h>  
#include <Adafruit_Sensor.h> 
#include <Adafruit_ST7735.h>
#include <Wire.h>
#include <SPI.h>
#include <Encoder.h> 

// Pin definitions for the 1.77-inch TFT (ST7735)
#define TFT_CS  6
#define TFT_RST 11  
#define TFT_DC  10
#define TFT_SCLK 7  
#define TFT_MOSI 8

// Rotary encoder pins
#define ENCODER_CLK 3
#define ENCODER_DT  4
#define ENCODER_SW  5 

// constant variables
#define relayPin 2           /* Pump is contrpolled via PUMP+ on PCB*/
#define moisturePin A0        /* Capacitive moisture sensor is connected to SOIL on PCB*/
#define drySoil 800         /* dry soil moisture value from calibration*/
#define wetSoil 410          /* wet soil moisture value from calibration*/

//TFT screen entity
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

//encoder entity
Encoder myEnc(ENCODER_CLK, ENCODER_DT); // Create an Encoder object

//global variables
byte count =10;
bool pumpState = false;
unsigned long moistureCheckInterval = 120000; // Interval to check moisture 2 minute)
unsigned long pumpRunTime = 0; // How long to run the pump (60 seconds)
unsigned int pumpRunTimePerDay =0;
unsigned long countdownTime = 0; // Holds the countdown time for display
bool isCountingDown = true; // Indicates if a countdown is active
int lastMoistureValue = 0; // Stores the last measured soil moisture value

//PID variables
int setpoint = 70;  // Desired soil moisture level
int tempSetpoint = 70; // Temporary variable for the setpoint during adjustment
float kp = 2.0;      // Proportional gain
float ki = 0.1;      // Integral gain
float kd = 1.0;      // Derivative gain
float lastError = 0;
float integral = 0;
unsigned long lastTime = 0;
 

// Menu variables controlled by encoder
int menuIndex = 0;  // To keep track of the current menu item
bool isAdjusting = false;
bool isEditingSetpoint = false;
bool isFlashing = false;
unsigned long lastFlashTime = 0;
unsigned long flashInterval = 250; // Flash interval in milliseconds

//data logging variables
unsigned long startingTimeStamp;
unsigned long oneDay = 86400000;

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

// Handle encoder button press
void readEncoderButton(){
  if (digitalRead(ENCODER_SW) == LOW) {
  if (!isAdjusting) {
      isAdjusting = true;  // Enter adjustment mode
      if (menuIndex == 0) {
        tempSetpoint = setpoint;  // Load current setpoint
      } else if (menuIndex == 1) {
        tempDailyLimit = dailyLimit;  // Load current daily limit
      }
    } 
    else {
      //Serial.println("button was pressed ");
       isAdjusting = false;  // Exit adjustment mode
      if (menuIndex == 0) {
        setpoint = tempSetpoint;  // Save adjusted setpoint
      } else if (menuIndex == 1) {
        dailyLimit = tempDailyLimit;  // Save adjusted daily limit
      }
      delay(200);  // Debounce delay
    }
  }

 // If in adjusting mode, handle encoder rotation to change value
  if (isAdjusting) {
    long newPosition = myEnc.read() / 4; // Adjust sensitivity as needed
    if (newPosition != 0) {
      if (menuIndex == 0) {
        tempSetpoint -= newPosition;
        tempSetpoint = constrain(tempSetpoint, 0, 100); // Constrain within valid range
        tft.fillRect(75,30,35,25,ST7735_BLACK);  //set point block rwswt
      } else if (menuIndex == 1) {
        tempDailyLimit -= newPosition * 0.1; // Adjust daily limit in 0.1L increments
        tempDailyLimit = constrain(tempDailyLimit, 0.1, 10.0); // Constrain within valid range
        tft.fillRect(75,55,35,25,ST7735_BLACK);  //set point block rwswt
      }
      myEnc.write(0); // Reset encoder position to zero
    }
  } else {
    // Handle encoder rotation to navigate menu
    long newPosition = myEnc.read() / 4;
    if (newPosition != 0) {
      menuIndex -= newPosition;
      menuIndex = constrain(menuIndex, 0, 1); // Constrain within the number of menu item
      myEnc.write(0); // Reset encoder position to zero
    }
  }

   
  

//Calculate PID controled for pump runtime by checking if soil moister near setpoint
float computePID(int input) {
  unsigned long currentTime = millis();
  float timeChange = (float)(currentTime - lastTime) / 1000.0; // Convert to seconds
  
  // Calculate the error
  float error = setpoint - input;

  // Proportional term
  float pTerm = kp * error;

  // Integral term
  integral += error * timeChange;
  float iTerm = ki * integral;
  
  // Derivative term
  float derivative = (error - lastError) / timeChange;
  float dTerm = kd * derivative;

  // Combine all terms
  float output = pTerm + iTerm + dTerm;

  // Save error and time for next calculation
  lastError = error;
  lastTime = currentTime;
  return output;  //time for pump to run
  
}

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
void checkMoisture(){    
  lastMoistureValue = analogRead(moisturePin); // Read the moisture level
  Serial.print("Raw Soil Moisture Level: ");
  Serial.println(lastMoistureValue);
  lastMoistureValue = map(lastMoistureValue ,drySoil ,wetSoil ,0 ,100); // map the range in percantage
  Serial.print("Soil Moisture Level: ");
  Serial.print(lastMoistureValue);  
  Serial.println(" %");  
  Serial.print("Setpoint: ");
  Serial.println(setpoint); 
  float pidOutput = computePID(lastMoistureValue); // Calculate the PID output  
  Serial.print("PID output in ms: ");
  Serial.println(pidOutput); 

  isCountingDown=false; //make sure we have actual data before printing to screent   
  tft.fillRect(65,5,25,25,ST7735_BLACK);  //refresh moisture field
  tft.fillRect(5,105,120,25,ST7735_BLACK);  //refresh task block reset
 
  if (pidOutput > 0 && !pumpState ) {
    pumpRunTime = ((unsigned long)pidOutput);   // cast to positive number from PID-calculation
    pumpRunTimePerDay += pumpRunTime;         //accumulate the total runtime
    tft.fillRect(5,130,120,25,ST7735_BLACK);  //refresh total pump run time counter   
    Serial.println("Moisture low. Turning on the pump...");
    digitalWrite(relayPin, LOW); // Turn on the pump, which is active LOW
    pumpState = true;    
    Serial.print("pump runtime:");
    Serial.println(pumpRunTime);
    isCountingDown = true;  //now print new data to screen
    enqueue(stopPump, pumpRunTime); // Schedule a task to stop the pump after 30 seconds    
    enqueue(checkMoisture, moistureCheckInterval); // Re-schedule the moisture check task
  }
  else { 
    isCountingDown = true;  //now print new data to screen
    Serial.println("Soil Moisture above treashold. Lets check again...in 12 min ");
    enqueue(checkMoisture, moistureCheckInterval); // Re-schedule the moisture check task 
  }
   
}

// Task to stop the pump
void stopPump() {
  if(pumpState){  //protection if user decides to increase sampling rate
    Serial.print("Total pump run time in 24h:");
    Serial.println(pumpRunTimePerDay);
    Serial.println("Moisture low. Turning on the pump...");
    isCountingDown = true; // Stop the countdown after task execution 
    Serial.println("Turning off the pump...");
    digitalWrite(relayPin, HIGH); // Turn off the relay
    pumpState = false;
    tft.fillRect(5,105,128,25,ST7735_BLACK);  //refres task block resetr
  }
}

// Function to update the TFT screen with the countdown and moisture value
void updateDisplay() {
  if (isCountingDown) {    
    unsigned long currentTime = millis();
    unsigned long timeRemaining = (taskQueue[queueStart].executeTime > currentTime) ?
                                  (taskQueue[queueStart].executeTime - currentTime) : 0;
    char buff[4]; // 3 characters + NUL
    sprintf(buff, "%10d", timeRemaining / 1000);       // Right-justified long converted to seconds

  if (isAdjusting) {
    if (currentTime - lastFlashTime > flashInterval) {
      isFlashing = !isFlashing;
      lastFlashTime = currentTime;
      tft.fillRect(75,30,35,25,ST7735_BLACK);  //refresh setpoint number      
    }
  } else {
    isFlashing = false;
  }

    //1st line
    tft.setTextSize(2);
    tft.setCursor(5, 5);
    tft.println("SOIL:");
    tft.setCursor(65, 5);
    tft.println(lastMoistureValue);
    tft.setCursor(105, 5);
    tft.println("%");

    //2nd line
    //tft.setCursor(5, 30);
    //tft.setTextSize(2);
    //tft.println("TASK:");

    //3nd line
    if (taskQueue[queueStart].function == checkMoisture) {
      tft.fillRect(60,55,50,25,ST7735_BLACK);  //refresh countdown field  
      tft.setCursor(55, 30);
      tft.println("Soil Check");
    } 
    else if (taskQueue[queueStart].function == stopPump) {
      tft.fillRect(5,55,50,25,ST7735_BLACK);  //refresh countdown field
      tft.setCursor(55, 30);
      tft.println("Pump on");
    }
    else if (taskQueue[queueStart].function == stopPump) {
      tft.fillRect(5,55,50,25,ST7735_BLACK);  //refresh countdown field
      tft.setCursor(55, 30);
      tft.println("Pump on");
    }

   //third line
    tft.setCursor(5, 55);
    tft.setTextSize(2);
    tft.println("TIME");
    tft.setCursor(60, 55);
    tft.println(buff); // Display countdown in seconds right side justified
    tft.setCursor(100, 55);
    tft.println("s");
   
    //second linre
    tft.setCursor(5, 30);
    tft.println("SET:");
    tft.setCursor(75, 30);
   if (menuIndex == 0) {
      tft.setTextColor(isAdjusting && isFlashing ? ST7735_RED : ST7735_RED); // Flashing text color red when editing
    } 
    else {
      tft.setTextColor(ST7735_GREEN); // Normal text color
    }
    tft.print("Setpoint: ");
    tft.print(isAdjusting ? tempSetpoint : setpoint);
    tft.setTextColor(ST7735_GREEN);
    tft.setCursor(105, 30);
    tft.println("%");

// Water Limit Menu Item
  tft.setCursor(5, 55);
    tft.println("LMIT:");
  tft.setCursor(75, 55);
  if (menuIndex == 1) {
    tft.setTextColor(isAdjusting && isFlashing ? ST7735_RED : ST7735_BLACK); // Flash if adjusting
  } else {
    tft.setTextColor(ST7735_BLACK); // Normal text color
  }
  tft.print("Water Limit: ");
  tft.print(isAdjusting ? tempDailyLimit : dailyLimit, 1);
  tft.print(" L");



    
    

  

    //6th line     
    tft.setCursor(5, 130);  
    tft.println(pumpRunTimePerDay/ 1000);   //print counter for total pump run
    
  }
}

//Graphics for screen startup
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
  pinMode(ENCODER_SW, INPUT_PULLUP); // Set encoder button as input with internal pull-up

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
    startIpDisplay(); 
    delay(500);   
  }
  tft.fillScreen(ST7735_BLACK); // Clear the screen (white background)
 
  // Schedule the first moisture check
  enqueue(checkMoisture, 0); // Start meassuring soil moisture immediately
  startingTimeStamp = millis();
}

//24h log refresh
void dataLoggingForPumpRunTimeTotal(){
  unsigned long currentTimePump = millis();
  if (currentTimePump - startingTimeStamp > oneDay) {
    pumpRunTimePerDay = 0;
    startingTimeStamp = currentTimePump;
  }
}

void loop() {  
  dequeue(); // Execute tasks as their time comes
  updateDisplay(); // Update the display with the countdown and moisture value
  readEncoderButton();  //Encoder button
  dataLoggingForPumpRunTimeTotal(); //Keep checking if 24h has passed
}
