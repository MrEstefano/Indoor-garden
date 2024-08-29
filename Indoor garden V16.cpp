/**************************************************************************************/
/*  Name    : Indoor Garden V16                                                       */
/*  Author  : Stefan Zakutansky ATU.ie student                                        */
/*  Date    : 29. 8.2023                                                              */
/*  Notes   : The code operates on Queuing architecture to faciliate a "mutex" for    */
/*            global variables. The system measures a soil moisture by capacitive    	*/
/*            sensor. The values are computed by PID alghoritm to calculate           */
/*            a pump runtime acordingly. The sampling rate of 4 minutes provides      */
/*            favorable soil condition to mantain healthy growth of the plant.        */
/*            The pump is active LOW. The Countdown for each task currently running is*/
/*            displayed, same as soil moisture, setpoint and total pump runtime       */
/*            per day on a TFT display. User can intercat with encoder push button,   */
/*            in order to orientate in short menu to adjust the treashold value and   */
/*            dayly water limit  and PID cooeficients                                 */
/**************************************************************************************/

#include <Adafruit_GFX.h>  
#include <Adafruit_Sensor.h> 
#include <Adafruit_ST7735.h>
#include <Wire.h>
#include <SPI.h>
#include <ClickEncoder.h>
#include <TimerOne.h>

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
#define FLOW_SENSOR_PIN 9
#define relayPin 2           /* Pump is contrpolled via PUMP+ on PCB*/
#define moisturePin A0        /* Capacitive moisture sensor is connected to SOIL on PCB*/
#define drySoil 800         /* dry soil moisture value from calibration*/
#define wetSoil 410          /* wet soil moisture value from calibration*/

//TFT screen entity
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

enum Page { MAIN_MENU, SUB_MENU };
Page page = MAIN_MENU;
   

int menuitem = 1;
int frame = 1;
int lastMenuItem = 1;

String menuItem1 = "Setpoint ";
String menuItem2 = "Flow lim ";
String menuItem3 = "Proport  ";
String menuItem4 = "Integral ";
String menuItem5 = "PUMP: OFF";
String menuItem6 = "Reset    ";


int flow_limit = 50;



bool up = false;
bool down = false;
bool middle = false;
bool lastMiddleState = false;

ClickEncoder *encoder;
int16_t last, value;



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
float kp = 2.00;      // Proportional gain
float ki = 0.10;      // Integral gain
float kd = 1.00;      // Derivative gain
float lastError = 0;
float integral = 0;
unsigned long lastTime = 0;
 




// Flow sensor pin and variables

volatile unsigned int flowPulseCount = 0;
float flowRate = 0.0;
float waterUsedToday = 0.0; // Liters used today
float dailyLimit = 1.5; // Daily limit in liters
float tempDailyLimit= 1.5;

//data logging variables
unsigned long startingTimeStamp;
const unsigned long resetInterval = 86400000UL; // 24 hours in milliseconds

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

void navigateInMenu();
void drawSubMenu();
void displayMenuItem();    
  

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
  isCountingDown=false; //make sure we have actual data before printing to screent   
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

  
  //tft.fillRect(65,5,25,25,ST7735_BLACK);  //refresh moisture field
  //tft.fillRect(5,30,59,25,ST7735_BLACK);  //refresh task block 
 
  if (pidOutput > 0 && !pumpState ) {
    //isCountingDown = true;  //now print new data to screen
    pumpRunTime = ((unsigned long)pidOutput);   // cast to positive number from PID-calculation
    pumpRunTimePerDay += pumpRunTime;         //accumulate the total runtim
    pumpRunTimePerDay = dataLoggingForTimeTotal(millis()); //check if 24h overflow happened
    Serial.println("Moisture low. Turning on the pump...");    
    digitalWrite(relayPin, LOW); // Turn on the pump, which is active LO
    pumpState = true;    
    Serial.print("pump runtime:");
    Serial.println(pumpRunTime);   
    isCountingDown = true;  //now print new data to screen
    //attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), flowSensorISR, RISING);  //Enable ISR
    //enqueue(calculateWaterFlow,0);  //monitor flow meter while pump is on
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
    //isCountingDown = false; // Stop the countdown after task execution 
    Serial.print("Total pump run time in 24h:");
    Serial.println(pumpRunTimePerDay);
    Serial.println("Moisture low. Turning on the pump...");
    
    Serial.println("Turning off the pump...");
    digitalWrite(relayPin, HIGH); // Turn off the relay
    //detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN)); //Disable ISR
    pumpState = false;
    //enqueue(dataLoggingForTimeTotal,0); //Keep checking if 24h has passed
    //tft.fillRect(5,30,59,25,ST7735_BLACK);  //refresh task block 
  }
}
/*
// Interrupt service routine for flow sensor
void flowSensorISR() {
  flowPulseCount++;
}

// Function to calculate water flow and usage
void calculateWaterFlow() {
  isCountingDown = true; // Stop the countdown after task execution 
  static unsigned long lastCalcTime = 0;
  unsigned long currentTime = millis(); 
  unsigned long timeElapsed = currentTime - lastCalcTime;  
    if (timeElapsed >= 1000) {                               // Calculate flow rate every second
    flowRate = (flowPulseCount / 7.5) / (timeElapsed / 1000.0); // Liters per second
    waterUsedToday += (flowRate * (timeElapsed / 1000.0)); // Update total water used
    flowPulseCount = 0;
    lastCalcTime = currentTime;
    }  
  waterUsedToday = dataLoggingForTimeTotal(millis()); //check if 24h time period overflown   
}
*/
// Function to update the TFT screen with the countdown and moisture value

  

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
  Timer1.initialize(500);
  Timer1.attachInterrupt(timerIsr); 
  
  //encoder entity
   encoder = new ClickEncoder(ENCODER_CLK, ENCODER_DT, ENCODER_SW, 2, false); // (uint8_t A, uint8_t B, uint8_t BTN = -1, uint8_t stepsPerNotch = 1, bool active = LOW);
  encoder->setAccelerationEnabled(false);

  last = encoder->getValue();
  // Schedule the first moisture check
  enqueue(checkMoisture, 0); // Start meassuring soil moisture immediately
 
  //lastTime = millis(); //Trigger the PID contrall
  startingTimeStamp = millis();
}

//24h log refresh
int dataLoggingForTimeTotal(unsigned long currentTimePump){
  int reset;
  if (currentTimePump - startingTimeStamp > resetInterval) {
    reset = 0;
    startingTimeStamp = currentTimePump;
  }
  return reset;
}

void loop() {  
  dequeue(); // Execute tasks as their time comes
  drawMenu();
  readRotaryEncoder();  
  navigateInMenu();   
}

void timerIsr() {
  encoder->service();
}

void readRotaryEncoder(){    
  ClickEncoder::Button buttonState = encoder->getButton();
  if (buttonState == ClickEncoder::Clicked && !lastMiddleState) {
      middle = true;
  }

  lastMiddleState = ( buttonState == ClickEncoder::Clicked); 
   value += encoder->getValue();
  
  if (value > last) {
    last = value;
    down = true;
    
    delay(15);
  }else   if (value < last) {
    last = value;
    up = true;
    
    delay(15);
  }
}

 
void drawMenu() {
  //3nd line
  
  /*else if (taskQueue[queueStart].function == calculateWaterFlow) {
    tft.fillRect(58,80,40,25,ST7735_BLACK);  //refresh countdown field
    tft.setCursor(5, 55);
    tft.println("Flow check");
  }
  */
  unsigned long currentTime = millis();
  unsigned long timeRemaining = (taskQueue[queueStart].executeTime > currentTime) ?
                                (taskQueue[queueStart].executeTime - currentTime) : 0;
  char buff[6]; // 3 characters + NUL
  sprintf(buff, "%3d", timeRemaining / 1000);       // Right-justified long converted to seconds
  if (taskQueue[queueStart].function == checkMoisture) {
    //tft.fillRect(70,30,40,25,ST7735_BLACK);  //refresh countdown field  
    tft.setCursor(5, 30);
    tft.println("Check");
  } 
  else if (taskQueue[queueStart].function == stopPump) {
    //tft.fillRect(70,30,40,25,ST7735_BLACK);  //refresh countdown field
    tft.setCursor(5, 30);
    tft.println("Pump");
  }
  if (isCountingDown) {   
    //1st line
    tft.setTextSize(2);
    tft.setCursor(5, 5);
    tft.println("SOIL:");
    tft.setCursor(65, 5);
    tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
    tft.println(lastMoistureValue);
    tft.setCursor(105, 5);
    tft.println("%");     
    //2nd line    
    tft.setCursor(70, 30);
    tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
    tft.print(buff); // Display countdown in seconds right side justified
    tft.print("s");
  }   
  //3rd half of screen
  if (page==MAIN_MENU) {   
    drawMainMenu(); 
  }
  else if(page==SUB_MENU ){
    drawSubMenu();
  }
}

void drawMainMenu(){      
  tft.setTextSize(2);  
  tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
  tft.setCursor(3, 55);
  tft.print("MAIN MENU");
  tft.fillRect(0,72,128,2,ST7735_GREEN);  //draw a line    

  if(frame == 1){
    if(menuitem==1 ){          
      displayMenuItem(menuItem1, 80,true);
      displayMenuItem(menuItem2, 105,false);
      displayMenuItem(menuItem3, 130,false);
    }
    else if(menuitem == 2 ){        
      displayMenuItem(menuItem1, 80,false);
      displayMenuItem(menuItem2, 105,true);
      displayMenuItem(menuItem3, 130,false);
    }
    else if(menuitem == 3){         
      displayMenuItem(menuItem1, 80,false);
      displayMenuItem(menuItem2, 105,false);
      displayMenuItem(menuItem3, 130,true);
    }
  }
  
  else if (frame == 2){
    if (menuitem == 2 ){ 
      displayMenuItem(menuItem2, 80,true);
      displayMenuItem(menuItem3, 105,false);
      displayMenuItem(menuItem4, 130,false);
    }
    else if(menuitem == 3){        
      displayMenuItem(menuItem2, 80,false);
      displayMenuItem(menuItem3, 105,true);
      displayMenuItem(menuItem4, 130,false);
    }
    else if(menuitem == 4 ){       
      displayMenuItem(menuItem2, 80,false);
      displayMenuItem(menuItem3, 105,false);
      displayMenuItem(menuItem4, 130,true);
    }      
  }

  else if (frame == 3){
    if(menuitem == 3){
      displayMenuItem(menuItem3, 80,true);
      displayMenuItem(menuItem4, 105,false);
      displayMenuItem(menuItem5, 130,false);
    }
    else if(menuitem == 4){
      displayMenuItem(menuItem3, 80,false);
      displayMenuItem(menuItem4, 105,true);
      displayMenuItem(menuItem5, 130,false);
    }
    else if(menuitem == 5){
      displayMenuItem(menuItem3, 80,false);
      displayMenuItem(menuItem4, 105,false);
      displayMenuItem(menuItem5, 130,true);
    }   
  }

  else if (frame == 4){
    if(menuitem == 4 ){
      displayMenuItem(menuItem4, 80,true);
      displayMenuItem(menuItem5, 105,false);
      displayMenuItem(menuItem6, 130,false);
    }    
    else if(menuitem == 5){
      displayMenuItem(menuItem4, 80,false);
      displayMenuItem(menuItem5, 105,true);
      displayMenuItem(menuItem6, 130,false);
    }
    else if(menuitem == 6) {
      displayMenuItem(menuItem4, 80,false);
      displayMenuItem(menuItem5, 105,false);
      displayMenuItem(menuItem6, 130,true);
    }
  }       
} 

void drawSubMenu(){
  if(menuitem == 1){       
    displayIntMenuPage(menuItem1, setpoint);
  }
  else if ( menuitem == 2){
    displayIntMenuPage(menuItem2, flow_limit);
  }
  else if (menuitem == 3){    
    displayIntMenuPage(menuItem3, kp);
  }
  else if ( menuitem == 4)  {      
    displayIntMenuPage(menuItem4, ki);
  }
  else if (menuitem == 5)   {          
    displayIntMenuPage(menuItem4, (int)pumpState);
  }
}  

void resetDefaults()  {
  setpoint = 70;
  flow_limit = 50;
  kp = 2;
  ki = 0.1;    
  pumpState = true;
  menuItem5 = "Pump: OFF";    
}



void displayIntMenuPage(String menuItem, float value){    
  tft.setTextSize(2);  
  tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
  tft.setCursor(5, 55);
  tft.print(menuItem);
  tft.fillRect(0,72,128,2,ST7735_GREEN);  //draw a line
  tft.setCursor(5, 80);
  tft.print("Value     ");    
  tft.setTextSize(3);
  tft.setCursor(10, 105);
  if(menuitem <= 2){    //for first two items in menu - adjustable value is in whole numbers
    int number = (int)value;
    tft.print(number);
  }
  else{      
    tft.print(value);
  }        
  tft.setTextSize(2);     
}


void displayMenuItem(String item, int position, boolean selected){
  if(selected){
      tft.setTextColor(ST7735_BLACK, ST7735_GREEN);
      tft.setCursor(0, position);
      tft.print(">"+item);
    }
    else{
      tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
      tft.setCursor(0, position);
      tft.print(" "+item);
    }
  tft.setTextColor(ST7735_GREEN, ST7735_BLACK);  
}

void navigateInMenu(){
  if(page == MAIN_MENU){
    if (up) {     
      up = false;
      frameAligmentUP();
    }
    if (down ) {//We have turned the Rotary Encoder Clockwise
      frameAligmentDown();
      down = false;
    }
  }
  if(page == SUB_MENU){
    valueAdjustment();
  }  

  if (middle) {//Middle Button is Pressed
    middle = false;  
    pressButtonAction();
  }  
}

void frameAligmentUP(){
  //--------------------------------------
  if(menuitem==3 && lastMenuItem == 2){
    frame ++;
  }
  else  if(menuitem==4 && lastMenuItem == 3){
    frame ++;
  }
  else  if(menuitem==5 && lastMenuItem == 4 && frame!=4) {
    frame ++;
  }
  lastMenuItem = menuitem;
  menuitem++;  
  if (menuitem==7){      
    menuitem--;
  }
  //-------------------  
}

void frameAligmentDown(){
  if(menuitem==2 && frame ==2){
    frame--;
  }
  if(menuitem==4 && frame ==4){
    frame--;
  }
  if(menuitem==3 && frame ==3){
    frame--;
  }
  lastMenuItem = menuitem;
  menuitem--;
  if (menuitem==0){
    menuitem=1;
  } 
}

void valueAdjustment(){
  if(up){
  up=false;
    if (menuitem==1 ) {
      setpoint++;
    }
    else if ( menuitem==2 ) {
      flow_limit++;
    }
    else if ( menuitem==3 ) {
      kp=kp+0.1;
    }
      else if ( menuitem==4 ) {
      ki=ki+0.1;      
    }
  } 

  if(down){
    down = false;
    if (menuitem==1) {
      setpoint--;
    }
    else if ( menuitem==2) {     
      flow_limit--;
    }
    else if (menuitem==3 ) {
      kp=kp-0.1;    
    }
    else if ( menuitem==4 ) {
      ki=ki-0.1;    
    }
  }  
}

void pressButtonAction(){
  if (page == MAIN_MENU){
    if ( menuitem <=4) {
      tft.fillRect(0,55,128,105,ST7735_BLACK);  //refresh half of the screen
      page=SUB_MENU;
    }
    else if( menuitem==5) { // pum manual overwrite    
      if (pumpState){
        pumpState = false;
        tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
        menuItem5 = "Pump: OFF";
        digitalWrite(relayPin, HIGH); // Turn on the pump, which is active LO
        }
      else{
        pumpState = true; 
        tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
        menuItem5 = "Pump: ON ";
        digitalWrite(relayPin, LOW); // Turn on the pump, which is active LO
      }
    }
    else if( menuitem ==6){
      resetDefaults();
    }
  }     
  else if (page == SUB_MENU){
    tft.fillRect(0,55,128,105,ST7735_BLACK);  //refresh 
    page=MAIN_MENU; 
  }
    
}