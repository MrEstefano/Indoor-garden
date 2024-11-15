/**************************************************************************************/
/*  Name    : Indoor Garden V21                                                   */
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


//
#define DEBUG
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
#define FLOW_SENSOR_PIN 2
#define relayPin 9           /* Pump is contrpolled via PUMP+ on PCB*/
#define moisturePin A0        /* Capacitive moisture sensor is connected to SOIL on PCB*/
#define drySoil 1023         /* dry soil moisture value from calibration*/
#define wetSoil 200         /* wet soil moisture value from calibration*/

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

Page page = MAIN_MENU;   
Page oldPage ;
Menu_item menuitem ; 
Menu_item lastMenuItem ;

String menuItems[TOTAL_ITEMS] = {"SETPOINT ",
                                 "FLOW LIM ",
                                 "PROPORT  ",
                                 "INTEGRAL ",
                                 "PUMP: OFF",
                                 "RESET    "};

//Rotary encoder variables
volatile bool up = false;
volatile bool down = false;
volatile bool middle = false;
volatile bool lastMiddleState = false;

int frame = 1;


ClickEncoder *encoder;
int16_t last = NULL;
int16_t value;

//global variables
byte count =10;
volatile bool pumpState = false;
volatile bool navigateDownFlag = false;
volatile bool navigateUpFlag = false;
unsigned long moistureCheckInterval = 120000; // Interval to check moisture 2 minute)
unsigned long pumpRunTime = 0; // How long to run the pump (60 seconds)
volatile unsigned int pumpRunTimePerDay =0;
unsigned long countdownTime = 0; // Holds the countdown time for display
volatile bool isCountingDown = true; // Indicates if a countdown is active
int lastMoistureValue = 0; // Stores the last measured soil moisture value
volatile unsigned long lastFlowPulseTime = 0;

//PID variables - kp,ki,kd were calibrated 
int setpoint = 70;  // Desired soil moisture level
int tempSetpoint = 70; // Temporary variable for the setpoint during adjustment
float kp = 1.97;      // Proportional gain - original = 2;
float ki = 0.80;      // Integral gain - original 0.1;
float kd = 1.18;      // Derivative gain - original 1;
float lastError = 0;
float integral = 0;
unsigned long lastTime = 0;
 
// Flow sensor pin and variables
const float calibrationFactor = 16.6;//9 at .11 flow  // This factor may vary depending on the sensor
volatile unsigned int flowPulseCount = 0;
//float flowRate = 0.0;
volatile float waterUsedToday = 0.0; // Liters used today
float flow_limit = 1.0;

//data logging variables
unsigned long startingTimeStamp = 0;
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


//------------------------------------------------------------------------------
// DECLARATION OF SUB-FUNCTIONS
//------------------------------------------------------------------------------
//Declaring Sub-Functions
void flowSensorISR();
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
void refreshFrameOne();
void refreshFrameTwo();
void refreshFrameThree();
void refreshFrameFour();
void drawRefreshMainMenu();
void drawRefreshSubMenu();
void frameAligmentUP();
void frameAligmentDown();
void valueAdjustment();
void pressButtonAction();
void displayIntMenuPage(String menuItem, float value);
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
//#ifndef DEBUG  
  ScreenStartUpSequance(); //screen grafic startup
 //#endif 

  Timer1.initialize(1000);   //Rotary Encoder is triggered by Timer 1 overflow period
  Timer1.attachInterrupt(timerIsr); 
  
  //encoder entity
  encoder = new ClickEncoder(ENCODER_CLK, ENCODER_DT, ENCODER_SW, 1, false); // (uint8_t A, uint8_t B, uint8_t BTN = -1, uint8_t stepsPerNotch = 1, bool active = LOW);
  encoder->setAccelerationEnabled(true);
  last = encoder->getValue();

  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), flowSensorISR, RISING);  //Enable ISR


  // Schedule the first moisture check
  enqueue(checkMoisture, 2000); // Start meassuring soil moisture immediatel
  delay(200);
  menuitem = SETPOINT;
  startingTimeStamp = millis();
  //page = MAIN_MENU;
}

//------------------------------------------------------------------------------
// MAIN LOOP
//------------------------------------------------------------------------------

void loop() {  
  dequeue(); // Execute tasks as their time comes
 
    

  if (navigateDownFlag) {
    navigateInMenuDown();
    navigateDownFlag = false;
  }
  if (navigateUpFlag) {
    navigateInMenuUp();
    navigateUpFlag = false;
  }
  drawMenu(); //keep TFT screen updated 
}

//------------------------------------------------------------------------------
// SUB-FUNCTIONS
//------------------------------------------------------------------------------

void resetDefaults(){
  setpoint = 70;
  flow_limit = 1.0;
  kp = 1.97;      // Proportional gain - original = 2;
  ki = 0.80;      // Integral gain - original 0.1;
  pumpState = true;
  menuItems[PUMP] = "PUMP: OFF";    
}

void timerIsr() {
  encoder->service();
 
  //Check if button pressed
  ClickEncoder::Button buttonState = encoder->getButton();
  if (buttonState == ClickEncoder::Clicked && !lastMiddleState) {
    middle = true;
    pressButtonAction();
  }
  lastMiddleState = ( buttonState == ClickEncoder::Clicked); 

  //Check which direction the rotary is going
  value += encoder->getValue();  
  if (value > last) {
    last = value;
    navigateDownFlag = true; 
  }
  else   if (value < last) {
    last = value;
    navigateUpFlag = true;
  }
 
}

void drawMenu() {
  unsigned long currentTime = millis();
  unsigned long timeRemaining = (taskQueue[queueStart].executeTime > currentTime) ?
                                (taskQueue[queueStart].executeTime - currentTime) : 0;
  char buff[6]; // 3 characters + NUL
  sprintf(buff, "%3d", timeRemaining / 1000);       // Right-justified long converted to seconds
    
  if (isCountingDown) {   
    //1st line
    tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
    tft.setTextSize(2);
    tft.setCursor(5, 5);
    tft.println("SOIL:");
    tft.setCursor(65, 5);    
    tft.println(lastMoistureValue);
    tft.setCursor(105, 5);
    tft.println("%");     
    //2nd line    
    tft.setCursor(5, 30);
    if (taskQueue[queueStart].function == checkMoisture) {
      tft.println("Check");
    } 
    else if (taskQueue[queueStart].function == stopPump) {
      tft.println("Pump ");
    }
    tft.setCursor(70, 30);
    tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
    tft.print(buff); // Display countdown in seconds right side justified
    tft.print("s");
  }   
  
  //3rd line - half of screen
 
  if (page == MAIN_MENU) { 
    drawRefreshMainMenu(); 
  }
  else if(page == SUB_MENU ){
    drawRefreshSubMenu();
  }
}

void navigateInMenuUp(){  
  if(page == MAIN_MENU){   
    if(menuitem > SETPOINT){
      menuitem = static_cast<int>(menuitem) - 1; //cast enum type to integer in order to increment or decrement the count
    }
    else{
      menuitem = SETPOINT;
    }     
    
    frameAligmentUP(); 
  }
  else if(page == SUB_MENU){    
    valueAdjustment();
  }  
  up = true; 
}  

void navigateInMenuDown(){  
  if(page == MAIN_MENU){ 
     if(menuitem < RESET){
        menuitem = static_cast<int>(menuitem) + 1;   //cast enum type to integer in order to increment or decrement the coun
    }
    else{
        menuitem = RESET;
    }   
    
    frameAligmentDown(); 
  }          
  else if(page == SUB_MENU){    
    valueAdjustment();
  }   
   down = true;  
}

void pressButtonAction(){ 
  if (page == MAIN_MENU){
    
    mainMenuAction();       
  }     
  else if(page == SUB_MENU){      
    page = MAIN_MENU; 
    oldPage = SUB_MENU;          
  }   
  middle = false;   
}

 void mainMenuAction(){
  if ( menuitem <= INTEGRA) { 
    page = SUB_MENU;
    oldPage = MAIN_MENU;   
  }
  else if( menuitem == PUMP) { // pum manual overwrite    
    if (pumpState){
      pumpState = false;
      tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
      menuItems[PUMP] = "PUMP: OFF";
      digitalWrite(relayPin, HIGH); // Turn on the pump, which is active LO
    }
    else{
      pumpState = true; 
      tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
      menuItems[PUMP] = "PUMP: ON ";
      digitalWrite(relayPin, LOW); // Turn on the pump, which is active LO
    }
  }
  else if( menuitem == RESET){
    resetDefaults();
  }
}

//Calculate PID controled for pump runtime by checking if soil moister near setpoint
float computePID(int input) {
  unsigned long currentTime = millis();
  float timeChange = (float)(currentTime - lastTime) / 1000.0; // Convert to seconds
  
  if(timeChange <= 0.0){
      return 0;
  }
  // Calculate the error
  float error = setpoint - input;

  // Proportional term
  float pTerm = kp * error;

  // Integral term
  integral += error * timeChange;
  if (integral > 1000) integral = 1000;
  else if (integral < -1000) integral = -1000;
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
  int MoistureValue   = analogRead(moisturePin); // Read the moisture level
  delay(100);
  lastMoistureValue = map(MoistureValue ,drySoil ,wetSoil ,0 ,100); // map the range in percantage
  float pidOutput = computePID(MoistureValue); // Calculate the PID output
///#ifdef DEBUG
  Serial.print("Raw Soil Moisture Level: ");
  Serial.println(   MoistureValue);  
  Serial.print("Soil Moisture Level: ");
  Serial.print(lastMoistureValue);  
  Serial.println(" %");  
  Serial.print("Setpoint: ");
  Serial.println(setpoint);     
  Serial.print("PID output in ms: ");
  Serial.println(pidOutput); 
//#endif  
  if (pidOutput > 0 && !pumpState && waterUsedToday < flow_limit) {
    isCountingDown = true;  //now print new data to screen
    pumpRunTime = ((unsigned long)pidOutput);   // cast to positive number from PID-calculation
    pumpRunTimePerDay += pumpRunTime;         //accumulate the total runtim
    pumpRunTimePerDay = dataLoggingForTimeTotal(millis()); //check if 24h overflow happened
//#ifdef DEBUG    
    Serial.println("Moisture low. Turning on the pump...");   
    Serial.print("pump runtime: ");
    Serial.println(pumpRunTime); 
    Serial.print("ml");  
//#endif    
    digitalWrite(relayPin, LOW); // Turn on the pump, which is active LO
    pumpState = true;    
    enqueue(calculateWaterFlow,0);  //monitor flow meter while pump is on
    isCountingDown = true;  //now print new data to screen     
    enqueue(stopPump, pumpRunTime); // Schedule a task to stop the pump after 30 seconds    
    enqueue(checkMoisture, moistureCheckInterval); // Re-schedule the moisture check task
  }
  else { 
    isCountingDown = true;  //now print new data to screen
//#ifdef DEBUG    
    Serial.println("Soil Moisture above treashold. Lets check again...in 2 min ");
//#endif    
    enqueue(checkMoisture, moistureCheckInterval); // Re-schedule the moisture check task 
  }   
}

// Task to stop the pump
void stopPump() {
  if(pumpState){  //protection if user decides to increase sampling rate
////#ifdef DEBUG
    Serial.print("Total pump run time in day:");
    Serial.println(pumpRunTimePerDay);    
    Serial.println("Turning off the pump...");
//#endif        
    digitalWrite(relayPin, HIGH); // Turn off the relay
    pumpState = false;    
  }
}

// Interrupt service routine for flow sensor
void flowSensorISR() {
  unsigned long currentPulseTime = millis();
  if (currentPulseTime - lastFlowPulseTime > 5) { // Debounce time in ms
    flowPulseCount++;
    lastFlowPulseTime = currentPulseTime;
  }
}

// Function to calculate water flow and usage
void calculateWaterFlow() {
  isCountingDown = true; // Stop the countdown after task execution 
  static unsigned long lastCalcTime = 0;  
  unsigned long currentTime = millis();

  if ((currentTime - lastCalcTime) > 1000) {  // Update every second
    // Calculate the flow rate in liters/min
    float flowRate = ((1000.0 / (millis() - lastCalcTime)) * flowPulseCount) / calibrationFactor;

    // Total liters passed
    waterUsedToday += (flowRate / 60.0);  // Convert flow rate to liters per second
//#ifdef DEBUG    
    Serial.print("Water used so far: ");
    Serial.print(waterUsedToday);
    Serial.println(" L");
//#endif    
    // Reset pulse count and update time
    flowPulseCount = 0;
    lastCalcTime = currentTime;

    // Check and reset if 24 hours have passed
   waterUsedToday = dataLoggingForTimeTotal(currentTime); //check if 24h time period overflown and if yes reset value to 0 
    
  }
  //waterUsedToday = dataLoggingForTimeTotal(millis()); //check if 24h time period overflown and if yes reset value to 0 
}  

//#ifndef DEBUG  
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

void ScreenStartUpSequance(){
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
    delay(250);   
  }
  tft.fillScreen(ST7735_BLACK); // Clear the screen (white background) 
}

//#endif
void drawRefreshSubMenu(){
    if (page != oldPage) {
    tft.fillRect(0,55,128,105,ST7735_BLACK);  //refresh half of the screen when entering menu page first time
    drawSubMenu();
     oldPage = SUB_MENU;    
  }
  else{
    drawSubMenu();
  }
}

void drawSubMenu(){ 
  switch(menuitem){
    case SETPOINT:       
      displayIntMenuPage(menuItems[SETPOINT], setpoint);
      break;
    case FLOW:
      displayIntMenuPage(menuItems[FLOW], flow_limit);
      break;
    case PROPORT:    
      displayIntMenuPage(menuItems[PROPORT], kp);
      break;
    case INTEGRA:     
      displayIntMenuPage(menuItems[INTEGRA], ki);
      break;
    case PUMP:           
      displayIntMenuPage(menuItems[PUMP], (int)pumpState);
    break;
  }     
}  

void drawRefreshMainMenu(){
  if (page != oldPage) {
    tft.fillRect(0,55,128,105,ST7735_BLACK);  //refresh half of the screen when entering menu page first time
    drawMainMenu(); 
    oldPage = MAIN_MENU;  
  }
  else{
    drawMainMenu(); 
  }       
}

void drawMainMenu() {    
  tft.setTextSize(2);  
  tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
  tft.setCursor(3, 55);
  tft.print("MAIN MENU");
  tft.fillRect(0, 72, 128, 2, ST7735_GREEN);  // Draw a line   
//#ifdef DEBUG 
  Serial.print("frame: ");
  Serial.println(frame);
  Serial.print("menuitem: ");
  Serial.println(menuitem);
//#endif
  refreshFrame(frame, menuitem);
}

void refreshFrame(int frame, int menuitem) {
  int firstItem = frame - 1;
  int lastItem = frame + 1;
  int yPos = 80;

  for (int i = firstItem; i <= lastItem; i++, yPos += 25) {
    bool selected = (i == menuitem);
    displayMenuItem(menuItems[i], yPos, selected);
  }
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
 if(menuitem < FLOW){    //for first two items in menu - adjustable value is in whole numbers
    int castedValue = (int)value;
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
  if(menuitem == INTEGRA && frame ==4){
    frame--;
  }       
  else if(menuitem == PROPORT  && frame ==3  ){ 
    frame--;
  }  
  else if (menuitem == FLOW &&  frame == 2 ){ 
    frame--;  
  }  
   /*
    if(menuitem > SETPOINT){
      menuitem = static_cast<int>(menuitem) - 1;
    }
    else{
      menuitem = SETPOINT;
    }    
    */ 
    lastMenuItem = menuitem;
    up = false; 
   
   
}

void frameAligmentDown(){
  if(menuitem==PROPORT && lastMenuItem == FLOW){
    frame ++;
  }
  else  if(menuitem == INTEGRA && lastMenuItem == PROPORT){
    frame ++;
  }
  else  if(menuitem == PUMP && lastMenuItem == INTEGRA && frame!=4)  {
    frame ++;
  }    
  /*  
  if(menuitem < RESET){
        menuitem = static_cast<int>(menuitem) + 1;
    }
    else{
        menuitem = RESET;
    }  
*/
  lastMenuItem = menuitem;
  down =false;
 
}

void valueAdjustment(){
  if(up){  
    if (menuitem == SETPOINT ) {
      setpoint++;
    }
    else if ( menuitem == FLOW ) {
      flow_limit = flow_limit + 0.1 ;
    }
    else if ( menuitem == PROPORT  ) {
      kp=kp+0.1;
    }
      else if ( menuitem == INTEGRA  ) {
      ki=ki+0.1;      
    }
    up=false;
  } 

  if(down){
    if (menuitem == SETPOINT) {
      setpoint--;
    }
    else if ( menuitem == FLOW) {     
     flow_limit = flow_limit - 0.1 ;
    }
    else if (menuitem == PROPORT ) {
      kp=kp-0.1;    
    }
    else if ( menuitem == INTEGRA ) {
      ki=ki-0.1;    
    }
    down = false;
  }  
}





//24h log refresh
int dataLoggingForTimeTotal(unsigned long currentTime){
  int reset;
  if (currentTime - startingTimeStamp > resetInterval) {
    reset = 0;
    startingTimeStamp = currentTime;
  }
  return reset;
}
