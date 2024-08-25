/**************************************************************************************/
/*  Name    : Indoor Garden V12                                                       */
/*  Author  : Stefan Zakutansky ATU.ie student                                        */
/*  Date    : 25. 8.2023                                                              */
/*  Notes   : The code operates on Queuing architecture to faciliate a "mutex" for    */
/*            global variables. The system measures a soil moisture by capacitive    	*/
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
volatile unsigned int pumpRunTimePerDay =0;
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
bool isFlashing = false;
unsigned long lastFlashTime = 0;
unsigned long flashInterval = 250; // Flash interval in milliseconds


// Flow sensor pin and variables
#define FLOW_SENSOR_PIN 5
volatile unsigned int flowPulseCount = 0;
float flowRate = 0.0;
float waterUsedToday = 0.0; // Liters used today
const float dailyLimit = 1.5; // Daily limit in liters

//data logging variables
unsigned long startingTimeStamp;
unsigned long oneDay = 86400000UL;

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
int queueEnd = 0;// Function to add a task to the queue
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



// Handle encoder button press
void readEncoderButton(){
  if (digitalRead(ENCODER_SW) == LOW) {
    if (!isAdjusting) {
      isAdjusting = true;  // Enter adjustment mode
       switch (menuIndex){
         case 0 : tempSetpoint = setpoint;  // Load current setpoint
                 break;
         case 1 : tempDailyLimit = dailyLimit;  // Load current daily limit
                 break
         case 2 : pumpRunTimePerDay;  // Load current daily pump run time
                 break;  
        default:
                break;
      }    
    } 
    else {
      //Serial.println("button was pressed ");
       isAdjusting = false;  // Exit adjustment mode
      switch (menuIndex){
         case 0 : setpoint = tempSetpoint;  // Save adjusted setpoint
                 break;
         case 1 : dailyLimit = tempDailyLimit;  // Save adjusted daily limit
                 break
         case 2 : pumpRunTimePerDay;  // Load current daily pump run time
                 break;   
         default:
                break;
      }    
      delay(200);  // Debounce delay
    }
  }

  //If in adjusting mode, handle encoder rotation to change value
  if (isAdjusting) {
    long newPosition = myEnc.read() / 4; // Adjust sensitivity as needed
    if (newPosition != 0) {
     switch (menuIndex){
              //Moisture treashold Setpint item in menu list      
      case 0: tempSetpoint -= newPosition;                   
              tempSetpoint = constrain(tempSetpoint, 0, 100); // Constrain within valid range       
              tft.fillRect(75,105,35,25,ST7735_BLACK);  //menu point block reset
              tft.fillRect(5,130,128,25,ST7735_BLACK);  //menu item and its value block reset
              break;
               //Water limit setpoint item in menu list
       case 1: tempDailyLimit -= newPosition * 0.1; // Adjust daily limit in 0.1L increments  
               tempDailyLimit = constrain(tempDailyLimit, 0.1, 10.0); // Constrain within valid range
               tft.fillRect(75,105,35,25,ST7735_BLACK);  //menu point block reset     
               tft.fillRect(5,130,128,25,ST7735_BLACK);  //menu item and its value block reset
               break;
                //pump run dataloger
        case 2: tft.fillRect(75,105,35,25,ST7735_BLACK);  //menu point block reset     
                tft.fillRect(5,130,128,25,ST7735_BLACK);  //menu item and its value block reset
                break;
          default: 
                break;  
      }         
      myEnc.write(0); // Reset encoder position to zero
    }
  } 
  else {
    // Handle encoder rotation to navigate menu
    long newPosition = myEnc.read() / 4;
    if (newPosition != 0) {
      menuIndex -= newPosition;
      menuIndex = constrain(menuIndex, 0, 2); // Constrain within the number of menu item
      myEnc.write(0); // Reset encoder position to zero
    }
  }
 enqueue(readEncoderButton, 0);  //Start Encoder button  immediately
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
  tft.fillRect(5,55,128,25,ST7735_BLACK);  //refresh task block 
 
  if (pidOutput > 0 && !pumpState ) {
    pumpRunTime = ((unsigned long)pidOutput);   // cast to positive number from PID-calculation
    pumpRunTimePerDay += pumpRunTime;         //accumulate the total runtim
    Serial.println("Moisture low. Turning on the pump...");calculateWaterFlow    
    digitalWrite(relayPin, LOW); // Turn on the pump, which is active LO
    pumpState = true;    
    Serial.print("pump runtime:");
    Serial.println(pumpRunTime);   
    isCountingDown = true;  //now print new data to screen
    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), flowSensorISR, RISING);  //Enable ISR
    enqueue(calculateWaterFlow,0);  //monitor flow meter while pump is on
    enqueue(stopPump, pumpRunTime); // Schedule a task to stop the pump after 30 seconds    
    enqueue(checkMoisture, moistureCheckInterval); // Re-schedule the moisture check task
  }
  else { 
    isCountingDown = true;  //now print new data to screen
    Serial.println("Soil Moisture above treashold. Lets check again...in 4 min ");
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
    dettachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN)); //Disable ISR
    pumpState = false;
    enqueue(dataLoggingForPumpRunTimeTotal,0); //Keep checking if 24h has passed
    tft.fillRect(5,55,128,25,ST7735_BLACK);  //refresh task block 
  }
}

// Interrupt service routine for flow sensor
void flowSensorISR() {
  flowPulseCount++;
}

// Function to calculate water flow and usage
void calculateWaterFlow() {
  static unsigned long lastCalcTime = 0;
  static unsigned long lastCheckTime = 0;
  unsigned long currentTime = millis();
  unsigned long currentTimeReschedule = millis();
  unsigned long timeElapsed = currentTime - lastCalcTime;
  if(currentTimeReschedule - lastCheckTime < pumpRuntime){   //keep rescheduling while pump runs
    if (timeElapsed >= 1000) {                               // Calculate flow rate every second
      flowRate = (flowPulseCount / 7.5) / (timeElapsed / 1000.0); // Liters per second
      waterUsedToday += (flowRate * (timeElapsed / 1000.0)); // Update total water used
      flowPulseCount = 0;
      lastCalcTime = currentTime;
      }  
  enqueue(calculateWaterFlow,0);         //monitor flow meter while pump is on imedietly
  lastCheckTime = currentTimeReschedule; 
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

   //handle the interval between red and green interval
   if (isAdjusting) {  
     if (currentTime - lastFlashTime > flashInterval) {
       isFlashing = !isFlashing;
       lastFlashTime = currentTime;
     }
   }  
   else {
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
    tft.setCursor(5, 30);
    tft.setTextSize(2);
    tft.println("TASK RUNING NOW:");

    //3nd line
    if (taskQueue[queueStart].function == checkMoisture) {
      tft.fillRect(60,80,50,25,ST7735_BLACK);  //refresh countdown field  
      tft.setCursor(5, 55);
      tft.println("Soil moist Check");
    } 
    else if (taskQueue[queueStart].function == stopPump) {
      tft.fillRect(60,80,50,25,ST7735_BLACK);  //refresh countdown field
      tft.setCursor(5, 55);
      tft.println("Pump off in");
    }
    else if (taskQueue[queueStart].function == calculateWaterFlow) {
      tft.fillRect(60,80,50,25,ST7735_BLACK);  //refresh countdown field
      tft.setCursor(5, 55);
      tft.println("Water Flow check");
    }
   
   //4th line
    tft.setCursor(5, 80);
    tft.println("TIME");
    tft.setCursor(60, 80);
    tft.println(buff); // Display countdown in seconds right side justified
    tft.setCursor(100, 80);
    tft.println("s");
   
    //5th linre
    tft.setCursor(5, 105);
    tft.println("MENU:");
    tft.setCursor(75, 105);
    switch (menuIndex){
      case 0: tft.setTextColor(isAdjusting && isFlashing ? ST7735_RED : ST7735_GREEN); // Flashing text color red when editing
              tft.println("menuIndex");
              tft.setTextColor(ST7735_GREEN);
              tft.setCursor(5, 130);
              tft.println("SETPOINT: ");
              tft.setTextColor(isAdjusting && isFlashing ? ST7735_RED : ST7735_GREEN); // Flashing text color red when editing
              tft.print(isAdjusting ? tempSetpoint : setpoint);     
              tft.println(" %");
              break;
       case 1: tft.setTextColor(isAdjusting && isFlashing ? ST7735_RED : ST7735_GREEN); // Flash if adjusting
               tft.println("menuIndex");
               tft.setTextColor(ST7735_GREEN);
               tft.setCursor(5, 130);
               tft.print("Water Limit: ");
               tft.setTextColor(isAdjusting && isFlashing ? ST7735_RED : ST7735_GREEN); // Flashing text color red when editing
               tft.print(isAdjusting ? tempDailyLimit : dailyLimit, 1);
               tft.print(" L");
               break;
        case 2: tft.setTextColor(isAdjusting && isFlashing ? ST7735_RED : ST7735_GREEN); // Flash if adjusting
                tft.println("menuIndex");
                tft.setTextColor(ST7735_GREEN);
                tft.setCursor(5, 130);
                tft.print("Pump total: ");
                tft.print(pumpRunTimePerDay/ 1000);   //print counter for total pump run
                tft.print(" L");
                break;
          default: tft.setTextColor(ST7735_GREEN); // Normal text color 
                break;  
    }      
   tft.setTextColor(ST7735_GREEN); // Normal text color 
  }  
 enqueue(updateDisplay, 0);  //Start Update the display   immediately
 
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
  pinMode(FLOW_SENSOR_PIN, INPUT);
  // Initialize TFT 1.77 inch screen
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
  enqueue(readEncoderButton, 0);  //Start Encoder button   immediately
  enqueue(updateDisplay, 0);  //Start Update the display   immediately  
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
}
