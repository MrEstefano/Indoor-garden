/**************************************************************************************/
/*  Name    : Indoor Garden V23                                                       */
/*  Author  : Stefan Zakutansky ATU.ie student                                        */
/*  Date    : 2. 9.2023                                                              */
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

#define DEBUG 0  //If defined with Zero disables Serial.print, One activates 

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
#define FLOW_SENSOR_PIN 2    /*hardware interrupt pin*/
#define relayPin 9           /* Pump is contrpolled via PUMP+ on PCB*/
#define moisturePin A0        /* Capacitive moisture sensor is connected to SOIL on PCB*/
#define drySoil 800         /* dry soil moisture value from calibration*/
#define wetSoil 376         /* wet soil moisture value from calibration*/

//TFT screen entity
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

//menu variableds related
enum Menu_item { 
  SETPOINT, 
  FLOW, 
  PROPORT, 
  INTEGRA, 
  PUMP, 
  RESET, 
  TOTAL_ITEMS 
};

enum Page { 
  MAIN_MENU, 
  SUB_MENU 
};

String printChild[TOTAL_ITEMS] = {"SETPOINT ",
                                 "FLOW LIM ",
                                 "PROPORT  ",
                                 "INTEGRAL ",
                                 "PUMP: OFF",
                                 "RESET    "};


typedef struct {
    int frame = 1;
    Page parent = MAIN_MENU;
    Page lastParent = SUB_MENU;
    Menu_item child = SETPOINT; 
    Menu_item lastChild;
    String printChild[TOTAL_ITEMS];
    volatile bool navigateDownFlag = false;
    volatile bool navigateUpFlag = false;
} *context MENU;

/*
Page page = MAIN_MENU;   
Page oldPage = SUB_MENU;
Menu_item menuitem = SETPOINT; 
Menu_item lastMenuItem;
*/

//Rotary encoder variables
ClickEncoder *encoder;
typedef struct {
    volatile bool up = false;
    volatile bool down = false;
    volatile bool middle = false;
    volatile bool lastMiddleState = false;
    int16_t last, value;
} *encoder ROTARY;

//Pump controll 
typedef struct{
    unsigned float runTime = 0; // How long to run the pump (60 seconds)
    volatile bool state = false;
    volatile unsigned float runTimeTotal = 0;
} *waterControll PUMP;

//PID variables - kp,ki,kd were calibrated
typedef struct {
    float kp = 1.97;      // Proportional gain - original = 2;
    float ki = 0.80;      // Integral gain - original 0.1;
    float kd = 1.18;      // Derivative gain - original 1;
    float lastError = 0;
    float integral = 0;
    unsigned long lastTimeChecked = 0;
} *controller PID;

//Soil Moisture sensor 
typedef struct {
    unsigned int setpoint = 70;  // Desired soil moisture level
    unsigned int lastTimeChecked = 0; // Stores the last measured soil moisture value
    const unsigned long checkInterval = 120000; // Interval to check moisture 2 minute)
} *soil MOISTURE;
 
// Flow sensor 
typedef struct {
    const float calibrationFactor = 16.6;//9 at .11 flow  // This factor may vary depending on the sensor
    volatile unsigned int flowPulseCount = 0;
    volatile float usageTotal = 0.0; // Liters used today
    float supplyLimit = 1.0; // 1 liter
} *irrigation WATER;

//global variables
const byte count =10;
volatile bool isCounting = true; // Indicates if a countdown is active
unsigned long countdownTime = 0; // Holds the countdown time for display

//data logging variables
unsigned long startingTimeStamp = 0;
const unsigned long resetInterval = 86400000UL; // 24 hours in milliseconds

// Structure for a task in the queue
struct Task {
  void (*function)(); // Pointer to the function to execute
  unsigned long executeTime; // Time when the task should be executed
  volatile bool isCountingDown; // Start the countdow
};

// Queue for tasks
const int queueSize = 10;
Task taskQueue[queueSize];
int queueStart = 0;
int queueEnd = 0;

//------------------------------------------------------------------------------
// DECLARATION OF SUB-FUNCTIONS
//------------------------------------------------------------------------------

float computePID(int input);
void drawMenu(); 
void navigateInMenuDown();
void navigateInMenuUp();
void drawSubMenu();
void drawMainMenu();
void displayMenuItem();    
void enqueue(void (*function)(), unsigned long delayTime);  
void dequeue();
void checkMoisture();
void stopPump(); 
void calculateWaterFlow();
void startIpDisplay();
void ScreenStartUpSequance();
void resetDefaults();
void drawRefreshMainMenu();
void drawRefreshSubMenu();
void frameAligmentUP();
void frameAligmentDown();
void valueAdjustment();
void pressButtonAction();
void displayItemMenuPage(String menuItem, float value);
void displayMenuItem(String item, int position, boolean selected);
void mainMenuAction(); 
void updateSoilMoistureDisplay();
void updateCountdownDisplay();
void updateMenuDisplay();
void refreshFrame(int frame, int menuitem); 

//------------------------------------------------------------------------------
// SETUP
//------------------------------------------------------------------------------

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
  
#ifndef DEBUG
  ScreenStartUpSequance(); //screen grafic startup
#endif
  
  Timer1.initialize(1000);   //Rotary Encoder is triggered by Timer 1 overflow period
  Timer1.attachInterrupt(timerIsr); 
  
  //encoder entity
  encoder = new ClickEncoder(ENCODER_CLK, ENCODER_DT, ENCODER_SW, 1, false); // (uint8_t A, uint8_t B, uint8_t BTN = -1, uint8_t stepsPerNotch = 1, bool active = LOW);
  encoder->setAccelerationEnabled(true);
  ROTARY.last = encoder->getValue();

  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), flowSensorISR, RISING);  //Enable ISR

  // Schedule the first moisture check
  enqueue(checkMoisture, 0); // Start meassuring soil moisture immediatel
  //menuitem = SETPOINT; //initialize
  startingTimeStamp = millis();
 // page = MAIN_MENU;
}

//------------------------------------------------------------------------------
// MAIN LOOP
//------------------------------------------------------------------------------

void loop() {  
  dequeue(); // Execute tasks as their time comes
  if (MENU.navigateDownFlag) {
    navigateInMenuDown();
    MENU.navigateDownFlag = false;
  }
  if (MENU.navigateUpFlag) {
    navigateInMenuUp();
    MENU.navigateUpFlag = false;
  }
  drawMenu(); //keep TFT screen updated        
}

//------------------------------------------------------------------------------
// SUB-FUNCTIONS
//------------------------------------------------------------------------------

void resetDefaults(){
  MOISTURE.setpoint = 70;
  WATER.supplyLimit = 1.0;
  PID.kp = 1.97;      // Proportional gain - original = 2;
  PID.ki = 0.80;      // Integral gain - original 0.1;
  PUMP.state = true;
  MENU.printChild[PUMP] = "PUMP: OFF";    
}

void timerIsr() {
  encoder->service();
 
  //Check if button pressed
  ClickEncoder::Button buttonState = encoder->getButton();
  if (buttonState == ClickEncoder::Clicked && !ROTARY.lastMiddleState) {
    ROTARY.middle = true;
    pressButtonAction();
  }
  ROTARY.lastMiddleState = ( buttonState == ClickEncoder::Clicked); 

  //Check which direction the rotary is going
  ROTARY.value += encoder->getValue();  
  if (ROTARY.value > ROTARY.last) {
    ROTARY.last = ROTARY.value;
    MENU.navigateDownFlag = true; 
  }
  else   if (ROTARY.value < ROTARY.last) {
    ROTARY.last = ROTARY.value;
    MENU.navigateUpFlag = true;
  }
 
}

void drawMenu() {
  unsigned long currentTime = millis();
  unsigned long timeRemaining = (taskQueue[queueStart].executeTime > currentTime) ?
                                (taskQueue[queueStart].executeTime - currentTime) : 0;
  char buff[6]; // 3 characters + NUL
  sprintf(buff, "%3d", timeRemaining / 1000);       // Right-justified long converted to seconds
    
  if (isCounting) {   //check if new data has been processed 
    //1st line
    tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
    tft.setTextSize(2);
    tft.setCursor(5, 5);
    tft.println(F("SOIL:"));
    tft.setCursor(65, 5);    
    tft.println(MOISTURE.lastTimeChecked);
    tft.setCursor(105, 5);
    tft.println(F("%"));     
    //2nd line    
    tft.setCursor(5, 30);
    if (taskQueue[queueStart].function == checkMoisture) {
      tft.println(F("Check"));
    } 
    else if (taskQueue[queueStart].function == stopPump) {
      tft.println(F("Pump "));
    }
    tft.setCursor(70, 30);
    tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
    tft.print(buff); // Display countdown in seconds right side justified
    tft.print(F("s"));
  }   
  
  //3rd line - half of screen
 
  if (MENU.parent == MAIN_MENU) { 
    drawRefreshMainMenu(); 
  }
  else if(MENU.parent == SUB_MENU ){
    drawRefreshSubMenu();
  }
}

void navigateInMenuUp(){  
  if(MENU.parent == MAIN_MENU){       
    if(MENU.child > SETPOINT){
      MENU.child = static_cast<int>(MENU.child) - 1; //cast enum type to integer in order to increment or decrement the count
    }
    else{
      MENU.child = RESET;
    }     
    frameAligmentUP(); 
  }
  else if(MENU.parent == SUB_MENU){    
    valueAdjustment();
  }  
  ROTARY.up = true; 
}  

void navigateInMenuDown(){  
  if(MENU.parent == MAIN_MENU){      
    if(MENU.child < RESET){
        MENU.child = static_cast<int>(MENU.child) + 1;   //cast enum type to integer in order to increment or decrement the coun
    }
    else{
        MENU.child = SETPOINT;
    }
    frameAligmentDown(); 
  }          
  else if(MENU.parent == SUB_MENU){    
    valueAdjustment();
  }   
  ROTARY.down = true;  
}

void pressButtonAction(){ 
  if (MENU.parent == MAIN_MENU){
    mainMenuAction();       
  }     
  else if(MENU.parent == SUB_MENU){      
    MENU.parent = MAIN_MENU; 
    MENU.lastParent = SUB_MENU;   // flag to be set for csreen refresh when entering one of main screen manus       
  }   
  ROTARY.middle = false;   
}

 void mainMenuAction(){
  if ( MENU.child <= INTEGRA) { 
    MENU.parent = SUB_MENU;
    MENU.lastParent = MAIN_MENU; // flag to be set for csreen refresh when entering one of main screen manus         
  }
  else if( MENU.child == PUMP) { // pum manual overwrite    
    if (PUMP.state){
      PUMP.state = false;
      tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
      MENU.printChild[PUMP] = "PUMP: OFF";
      digitalWrite(relayPin, HIGH); // Turn on the pump, which is active LO
    }
    else{
      PUMP.state = true; 
      tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
      MENU.printChild[PUMP] = "PUMP: ON ";
      digitalWrite(relayPin, LOW); // Turn on the pump, which is active LO
    }
  }
  else if( MENU.child == RESET){
    resetDefaults();
  }
}

//Calculate PID controled for pump runtime by checking if soil moister near setpoint
float computePID(int input) {
  unsigned long currentTime = millis();
  float timeChange = (float)(currentTime - PID.lastTimeChecked) / 1000.0; // Convert to seconds
  
  if(timeChange <= 0.0){
      return 0;
  }
  // Calculate the error
  float error = MOISTURE.setpoint - input;

  // Proportional term
  float pTerm = PID.kp * error;

  // Integral term 
  PID.integral += error * timeChange;
  if (PID.integral > 1000) PID.integral = 1000;  //limit the Integral to prevent creeping up
  else if (PID.integral < -1000) PID.integral = -1000;
  float iTerm = PID.ki * PID.integral;
  
  // Derivative term
  float derivative = (error - PID.lastError) / timeChange;
  float dTerm = PID.kd * derivative;

  // Combine all terms
  float output = pTerm + iTerm + dTerm;

  // Save error and time for next calculation
  PID.lastError = error;
  PID.lastTimeChecked = currentTime;
  return output;  //time for pump to run  
}

// Function to add a task to the queue
void enqueue(void (*function)(), unsigned long delayTime) {
  unsigned long currentTime = millis();
  taskQueue[queueEnd].function = function;
  taskQueue[queueEnd].executeTime = currentTime + delayTime;
  taskQueue[queueEnd].isCountingDown = isCounting;
  queueEnd = (queueEnd + 1) % queueSize;
  countdownTime = delayTime; // Set the countdown time  
}

// Function to execute and remove the first task in the queue
void dequeue() {
  if (queueStart != queueEnd) { // Check if the queue is not empty
    unsigned long currentTime = millis();
    if (currentTime >= taskQueue[queueStart].executeTime) {
      taskQueue[queueStart].function(); // Execute the task
      taskQueue[queueEnd].isCountingDown = isCounting;
      queueStart = (queueStart + 1) % queueSize; // Remove the task from the queue
    }
  }
}

// Task to check soil moisture
void checkMoisture(){    
  isCounting = false; //flag to not let the new values to be printed yet   
  MOISTURE.lastTimeChecked = analogRead(moisturePin); // Read the moisture leve
#ifdef DEBUG
  Serial.print("Raw Soil Moisture Level: ");
  Serial.println(MOISTURE.lastTimeChecked);  
#endif  
  MOISTURE.lastTimeChecked = map(MOISTURE.lastTimeChecked ,drySoil ,wetSoil ,0 ,100); // map the range in percantage
  float pidOutput = computePID(MOISTURE.lastTimeChecked); // Calculate the PID output
#ifdef DEBUG 
  Serial.print("Soil Moisture Level: ");
  Serial.print(MOISTURE.lastTimeChecked);  
  Serial.println(" %");  
  Serial.print("Setpoint: ");
  Serial.println(MOISTURE.setpoint);     
  Serial.print("PID output in ms: ");
  Serial.println(pidOutput); 
#endif  
  //Check if PID output is positive number and Pump is not running and dayly water supply limit is not exceeding dayly limit
  if (pidOutput > 0 && !PUMP.state && WATER.usageTotal < WATER.supplyLimit) {
    isCounting = true;  //now print new data to screen
    PUMP.runTime = pidOutput;   // cast to positive number from PID-calculation
    PUMP.runTimeTotal += PUMP.runTime;         //accumulate the total runtim
    PUMP.runTimeTotal = dataLoggingForTimeTotal(currentTime); //check if 24h overflow happened
#ifdef DEBUG    
    Serial.println("Moisture low. Turning on the pump...");   
    Serial.print("pump runtime: ");
    Serial.println(PUMP.runTime); 
    Serial.print("ml");  
#endif    
    digitalWrite(relayPin, LOW); // Turn on the pump, which is active LOW
    PUMP.state = true;    
    enqueue(calculateWaterFlow,0);  //monitor flow meter while pump is on
    isCounting = true;  //now print new data to screen     
    enqueue(stopPump, PUMP.runTime); // Schedule a task to stop the pump after 30 seconds    
    enqueue(checkMoisture, MOISTURE.checkInterval); // Re-schedule the moisture check task
  }
  else { 
    isCounting = true;  //now print new data to screen
#ifdef DEBUG    
    Serial.println("Soil Moisture above treashold. Lets check again...in 2 min ");
#endif    
    enqueue(checkMoisture, MOISTURE.checkInterval); // Re-schedule the moisture check task 
  }   
}

// Task to stop the pump
void stopPump() {
  if(PUMP.state){  //protection if user decides to increase sampling rate
#ifdef DEBUG
    Serial.print("Total pump run time in day:");
    Serial.println(PUMP.runTimeTotal);    
    Serial.println("Turning off the pump...");
#endif        
    digitalWrite(relayPin, HIGH); // Turn off the relay
    PUMP.state = false;    
  }
}

// Interrupt service routine for flow sensor
void flowSensorISR() {  
    WATER.flowPulseCount++;
}

// Function to calculate water flow and usage
void calculateWaterFlow() {
  isCounting = true; // Stop the countdown after task execution 
  static unsigned long lastCalcTime = 0;  
  unsigned long currentTime = millis();

  if ((currentTime - lastCalcTime) > 1000) {  // Update every second
    // Calculate the flow rate in liters/min
    float flowRate = ((1000.0 / (millis() - lastCalcTime)) * WATER.flowPulseCount) / WATER.calibrationFactor;

    // Total liters passed
    WATER.usageTotal += (flowRate / 60.0);  // Convert flow rate to liters per second
#ifdef DEBUG    
    Serial.print(F("Water used so far: "));
    Serial.print(WATER.usageTotal);
    Serial.println(F(" L"));
#endif    
    // Reset pulse count and update time
    WATER.flowPulseCount = 0;
    lastCalcTime = currentTime;

    // Check and reset if 24 hours have passed
    WATER.usageTotal = dataLoggingForTimeTotal(currentTime); //check if 24h time period overflown and if yes reset value to 0 
    
#ifdef DEBUG        
        Serial.println(F("24 hours passed. Water usage reset."));
#endif 
  }
}  

void drawRefreshSubMenu(){
    if (MENU.parent != MENU.lastParent) { //refresh a menu screen when entering menu page first time
    tft.fillRect(0,55,128,105,ST7735_BLACK);  
    drawSubMenu();
    MENU.lastParent = SUB_MENU;    
  }
  else{
    drawSubMenu();
  }
}

void drawSubMenu(){ 
  switch(MENU.child){
    case SETPOINT:       
      displayItemMenuPage(MENU.printChild[SETPOINT], MOISTURE.setpoint);
      break;
    case FLOW:
      displayItemMenuPage(MENU.printChild[FLOW], WATER.supplyLimit);
      break;
    case PROPORT:    
      displayItemMenuPage(MENU.printChild[PROPORT], PID.kp);
      break;
    case INTEGRA:     
      displayItemMenuPage(MENU.printChild[INTEGRA], PID.ki);
      break;
    case PUMP:           
      displayItemMenuPage(MENU.printChild[PUMP], (float)PUMP.state);
    break;
  }     
}  

void drawRefreshMainMenu(){
  if (MENU.parent != MENU.lastParent) { //refresh a menu screen when entering menu page first time
    tft.fillRect(0,55,128,105,ST7735_BLACK);  
    drawMainMenu(); 
    MENU.lastParent = MAIN_MENU;  
  }
  else{
    drawMainMenu(); 
  }       
}

void drawMainMenu() {    
  tft.setTextSize(2);  
  tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
  tft.setCursor(3, 55);
  tft.print(F("MAIN MENU"));
  tft.fillRect(0, 72, 128, 2, ST7735_GREEN);  // Draw a line   
#ifdef DEBUG
  Serial.print(F("frame: "));
  Serial.println(MENU.frame);
  Serial.print(F("menuitem: "));
  Serial.println(MENU.child);
#endif
  refreshFrame(MENU.frame, MENU.child);
}

void refreshFrame(int frame, int menuitem) {
  int firstItem = frame - 1;
  int lastItem = frame + 1;
  int yPos = 80;

  for (int i = firstItem; i <= lastItem; i++, yPos += 25) {
    bool selected = (i == MENU.child);
    displayMenuItem(MENU.printChild[i], yPos, selected);
  }
}

void displayItemMenuPage(String menuItem, float value){    
  tft.setTextSize(2);  
  tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
  tft.setCursor(5, 55);
  tft.print(menuItem);
  tft.fillRect(0,72,128,2,ST7735_GREEN);  //draw a line
  tft.setCursor(5, 80);
  tft.print(F("Value     "));    
  tft.setTextSize(3);
  tft.setCursor(10, 105);
  if(menuitem < FLOW){    //for first two items in menu - adjustable value is in whole numbers
    int castedValue = (int)value
    char buff[6]; // 3 characters + NUL
    sprintf(buff, "%3d",castedValue);       // Right-justified 3 digits
    tft.print(buff);
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
}

void frameAligmentUP(){
  if(MENU.child == INTEGRA && MENU.frame ==4){
    MENU.frame--;
  }       
  else if(MENU.child == PROPORT  && MENU.frame ==3  ){ 
    MENU.frame--;
  }  
  else if (MENU.child == FLOW &&  MENU.frame == 2 ){ 
    MENU.frame--;  
  }  
  MENU.lastChild   = MENU.child;
  ROTARY.up = false; 
}

void frameAligmentDown(){
  if(MENU.child == PROPORT && MENU.lastChild   == FLOW){
    MENU.frame ++;
  }
  else  if(MENU.child == INTEGRA && MENU.lastChild   == PROPORT){
    MENU.frame ++;
  }
  else  if(MENU.child == PUMP && MENU.lastChild   == INTEGRA && MENU.frame!=4)  {
    MENU.frame ++;
  }      
  MENU.lastChild = MENU.child;
  ROTARY.down =false;
}

void valueAdjustment(){
  if(ROTARY.up){  
    if (MENU.child == SETPOINT ) {
      MOISTURE.setpoint++;
    }
    else if ( MENU.child == FLOW ) {
      WATER.supplyLimit = WATER.supplyLimit + 0.1 ;
    }
    else if ( MENU.child == PROPORT  ) {
      PID.kp=PID.kp+0.1;
    }
      else if ( MENU.child == INTEGRA  ) {
      PID.ki=PID.ki+0.1;      
    }
    ROTARY.up = false;
  } 

  if(ROTARY.down){
    if (MENU.child == SETPOINT) {
      MOISTURE.setpoint--;
    }
    else if ( MENU.child == FLOW) {     
     WATER.supplyLimit = WATER.supplyLimit - 0.1 ;
    }
    else if (MENU.child == PROPORT ) {
      PID.kp=PID.kp-0.1;    
    }
    else if ( MENU.child == INTEGRA ) {
      PID.ki=PID.ki-0.1;    
    }
    ROTARY.down = false;
  }  
}

#ifndef DEBUG
void ScreenStartUpSequance(){   //run screen intro graphics
  tft.drawRoundRect(3, 5, 122, 150,5, ST7735_BLUE);
  tft.setCursor(30, 20);
  tft.setTextColor(ST7735_GREEN);
  tft.setTextSize(2);
  tft.println(F("SYSTEM"));
  tft.setCursor(30, 45);
  tft.println(F("STARTS"));
  tft.setCursor(50, 70);
  tft.println(F("IN"));
  tft.drawRect(8 , 125, 112, 25,ST7735_BLUE);
  tft.fillRect(11, 128, 8, 19, ST7735_RED);
    for (byte i = 10;count > 0;i--){
    count = count - 1;
    startIpDisplay(); 
    delay(250);   
  }
  tft.fillScreen(ST7735_BLACK); // Clear the screen (white background) 
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
  }  
}
#endif

//24h log refresh
int dataLoggingForTimeTotal(unsigned long currentTime){
  int reset;
  if (currentTime - startingTimeStamp > resetInterval) {
    reset = 0;
    startingTimeStamp = currentTime;
  }
  return reset;
}
