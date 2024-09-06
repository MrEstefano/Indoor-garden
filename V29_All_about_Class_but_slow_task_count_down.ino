
/**************************************************************************************/
/*  Name    : Indoor Garden V29 Menu Class                                                 */
/*  Author  : Stefan Zakutansky ATU.ie student                                        */
/*  Date    : 4. 9.2023                                                              */
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



//#define DEBUG 0  //If defined with Zero disables Serial.print, One activates 

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

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);   


class Water{
private:  
  unsigned int flowPulseCount;  
public:
  float calibrationFactor;//9 at .11 flow  // This factor may vary depending on the sensor  
  float usageTotal; // Liters used today
  float supplyLimit; // 1 liter
  //prototypes
  void begin(int pin, void (*callback)());
  void flowSensorISR();
  int updateFlowPulseCount(int);
  int flowPulseCountIs();
};

//Constructor
void Water::begin(int pin, void (*callback)()){
  calibrationFactor = 16.6;//9 at .11 flow  // This factor may vary depending on the sensor
  usageTotal = 0.0; // Liters used today
  supplyLimit = 1.0; // 1 liter
  flowPulseCount = 0;
  attachInterrupt(digitalPinToInterrupt(pin), callback, RISING);
}

void Water::flowSensorISR(){
  flowPulseCount++;
}

//Instance
Water flowSensor;

int Water::flowPulseCountIs(){
    return  flowPulseCount;
  }


int Water::updateFlowPulseCount(int new_value){
    return  flowPulseCount = new_value;
  }

//Rotary encoder variables
ClickEncoder *encoder;
typedef struct encoderButton {    
    volatile bool middle = false;
    volatile bool lastMiddleState = false;
    int16_t last;
    int16_t value;
};encoderButton ROTARY;

//Pump controll 
typedef struct waterControll {
    float runTime = 0; // How long to run the pump (60 seconds)
    volatile bool state = false;
    float runTimeTotal = 0;
}; waterControll WATERPUMP;

//PID variables - kp,ki,kd were calibrated
typedef struct controller {
    float kp = 1.97;      // Proportional gain - original = 2;
    float ki = 0.80;      // Integral gain - original 0.1;
    float kd = 1.18;      // Derivative gain - original 1;
    float lastError = 0;
    float integral = 0;
    unsigned long lastTimeChecked = 0;
};controller PID;

enum Menu_item { SETPOINT, FLOW, PROPORT, INTEGRA, PUMP, RESET, TOTAL_ITEMS};
enum Page { MAIN_MENU, SUB_MENU};

//------------------------------------------------------------------------------
// DECLARATION OF SUB-FUNCTIONS
//------------------------------------------------------------------------------

float computePID(int input);
void printSoilMoisture();  
void enqueue(void (*function)(), unsigned long delayTime);  
void dequeue();
void checkMoisture();
void stopPump(); 
void calculateWaterFlow();
void startIpDisplay();
void ScreenStartUpSequance();
void updateSoilMoistureDisplay();
int  dataLoggingForTimeTotal(unsigned long currentTime);
void displayMenuItem(String item, int position, boolean selected);
void displayItemMenuPage(String menuItem, float value);
void resetDefaults();
void refreshFrame(); 
void drawMainMenu();
void drawSubMenu();
void updatePumpstatus();
void refreshMainMenu();
void refreshSubMenu();
void refreshmenu();

void newSetpoint();
void mainMenuAction();


char buffer[6]; // 3 characters + NUL 
unsigned long curTime;
unsigned long timeRemaining;

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

String printChild[TOTAL_ITEMS] = {"SETPOINT    ",
                                  "WATER LIMIT ",
                                  "PID Kp VALUE",
                                  "PID Ki VALUE",
                                  "PUMP: OFF   ",
                                  "RESET       "};


class Menu{
 
private:   
  void frameAligmentUP(); //private
  void frameAligmentDown(); //private
  bool upFlag;    //private
  bool downFlag; //private  
  
public:
  Menu(); 
  char frame;
  
  byte set_setpoint;
  Menu_item _child;
  Menu_item _lastChild; 
  Page _parent;
  Page _lastParent;
  void valueAdjustment();
  void pressButtonAction();
  void navigateInMenuUp();
  void navigateInMenuDown(); 
  //bool navigateDownFlag;
  //bool navigateUpFlag;
  ////////
  char setpointIs(); 
  };
 
Menu::Menu(){
  _parent = MAIN_MENU;
  _lastParent = SUB_MENU;
  _child = SETPOINT;
  _lastChild = SETPOINT;
  set_setpoint = 70;
  frame = 1;  
  //navigateDownFlag = false;
  //navigateUpFlag = false;
  upFlag = false;
  downFlag = false;
}

Menu menu;

void Menu::navigateInMenuUp(){  
  if(_parent == MAIN_MENU){       
    if(_child > SETPOINT){
       _child = static_cast<int>(menu._child) -1; //cast enum type to integer in order to increment or decrement the count
    }
    else{
      _child = SETPOINT;
    }     
    frameAligmentUP(); 
  }
  else if(menu._parent == SUB_MENU){    
    valueAdjustment();
  }  
 
  upFlag = true; 
} 

void Menu::navigateInMenuDown(){  
  if(_parent == MAIN_MENU){      
    if(_child < RESET){
        _child = static_cast<int>(_child) + 1;  //cast enum type to integer in order to increment or decrement the coun
    }
    else{
      _child = RESET;
    }
    frameAligmentDown(); 
  }          
  else if(_parent == SUB_MENU){    
    valueAdjustment();
  }   
  downFlag = true;  
}

void Menu::pressButtonAction(){ 
  if (_parent == MAIN_MENU){
    mainMenuAction();       
  }     
 // menu.updateLastParent()
  else if(menu._parent == SUB_MENU){      
    _parent = MAIN_MENU; 
    _lastParent = SUB_MENU;   // flag to be set for csreen refresh when entering one of main screen manus       
  }   
  
  ROTARY.middle = false;   
}

void Menu::frameAligmentUP(){
  if(_child == INTEGRA && frame ==4){
    frame--;
  }       
  else if(_child == PROPORT  && frame ==3  ){ 
    frame--;
  }  
  else if (_child == FLOW &&  frame == 2 ){ 
    frame--;  
  }  
  _lastChild = _child;
  upFlag = false; 
}

void Menu::frameAligmentDown(){
  if(_child == PROPORT && _lastChild == FLOW){
    frame ++;
  }
  else  if(_child == INTEGRA && _lastChild == PROPORT){
    frame ++;
  }
  else  if(_child == PUMP && _lastChild == INTEGRA && frame!=4)  {
    frame ++;
  }      
  _lastChild = _child;
  downFlag = false;
}



void Menu::valueAdjustment(){
  if(upFlag){  
    if (_child == SETPOINT ) {
        set_setpoint += 1;
    }
    else if ( _child == FLOW ) {
      flowSensor.supplyLimit = flowSensor.supplyLimit + 0.1 ;
    }
    else if ( _child == PROPORT  ) {
      PID.kp=PID.kp+0.1;
    }
      else if ( _child == INTEGRA  ) {
      PID.ki=PID.ki+0.1;      
    }
    upFlag = false;
  } 

  if(downFlag){
    if (_child == SETPOINT) {
      set_setpoint -= 1; 
    }
    else if (_child == FLOW) {     
     flowSensor.supplyLimit = flowSensor.supplyLimit - 0.1 ;
    }
    else if (_child == PROPORT ) {
      PID.kp=PID.kp-0.1;    
    }
    else if ( _child == INTEGRA ) {
      PID.ki=PID.ki-0.1;    
    }
    downFlag = false;
  }  
}



char Menu::setpointIs(){
  return set_setpoint;
}



//Soil Moisture sensor 
class Soil {
private:
    int _pin;      //hardware pin number    
    int drySoil;
    int wetSoil;
    int constrainedMoistureValue;
    unsigned int moistureValue; // Stores the last measured soil moisture value
    unsigned long checkInterval; // Interval to check moisture 2 min
    
public:  
    //constructor prototype
    Soil(int pin); 
    //functions ptototypes     
    int check();// Read the moisture leve        
    void updateSetpoint();         
    int percentValu();      
    unsigned long checkIntervalIs();
};
          
                                 
//constructor aka initialize
Soil::Soil(int pin) {
  _pin = pin ;   
  drySoil = 1023;
  wetSoil = 260;
  constrainedMoistureValue;
  moistureValue = 0; // Stores the last measured soil moisture value
  checkInterval = 120000; // Interval to check moisture 2 min
  pinMode(pin, INPUT);
}

const int pin_A0 = A0;  //A0 
//instances
 
Soil soilMoisture(pin_A0);

int Soil::check(){              
  return moistureValue = analogRead(_pin); // Read the moisture leve      
  
}
  
int Soil::percentValu(){
  return constrainedMoistureValue = map(moistureValue ,drySoil ,wetSoil ,0 ,100); // map the range in percantage
}
  
unsigned long Soil::checkIntervalIs() {
  return checkInterval;
}
//end of class Soil



//global variables
byte count =11;

//volatile bool isCounting = true; // Indicates if a countdown is active
unsigned long countdownTime = 0; // Holds the countdown time for display

//data logging variables
unsigned long startingTimeStamp = 0;
const unsigned long resetInterval = 86400000UL; // 24 hours in milliseconds
unsigned long start;



byte value = 9;
volatile bool flagComplete = false;
volatile bool timer_One_flag = false;



//------------------------------------------------------------------------------
// SETUP
//------------------------------------------------------------------------------

void setup() {
  Serial.begin(9600);
#ifdef DEBUG  
  Serial.begin(9600);
#endif  
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH); // Initially turn off the pump
  pinMode(ENCODER_SW, INPUT_PULLUP); // Set encoder button as input with internal pull-up
  pinMode(FLOW_SENSOR_PIN, INPUT);

  // Initialize TFT 1.77 inch screen
  tft.initR(INITR_BLACKTAB);    
  tft.fillScreen(ST7735_BLACK);
  tft.setRotation(1);   //make screen horisontal
  
#ifndef DEBUG
  ScreenStartUpSequance(); //screen grafic startup
#endif
  
 
  

  noInterrupts();                       // disable all interrupts
  

  Timer1.initialize(1000);   //Rotary Encoder is triggered by Timer 1 overflow period 50ms
  Timer1.attachInterrupt(timerIsr); 

   flowSensor.begin(FLOW_SENSOR_PIN, []() {flowSensor.flowSensorISR();});

  interrupts();                         // enable all interrupts
  //encoder entity
  encoder = new ClickEncoder(ENCODER_CLK, ENCODER_DT, ENCODER_SW, 1, false); // (uint8_t A, uint8_t B, uint8_t BTN = -1, uint8_t stepsPerNotch = 1, bool active = LOW);
  encoder->setAccelerationEnabled(true);
  ROTARY.last = encoder->getValue();

 

  // Schedule the first moisture check
  enqueue(checkMoisture, 0); // Start meassuring soil moisture immediatel 
  startingTimeStamp = millis();
}

//------------------------------------------------------------------------------
// MAIN LOOP
//------------------------------------------------------------------------------

void loop() {      
   
  dequeue(); // Execute tasks as their time comes
  start = millis();
  printSoilMoisture(); //keep TFT screen updated   
  refreshomeMenu();         
}

//------------------------------------------------------------------------------
// SUB-FUNCTIONS
//------------------------------------------------------------------------------
void timerIsr() {  
  encoder->service();    
   //Check if button pressed
  ClickEncoder::Button buttonState = encoder->getButton();
  if (buttonState == ClickEncoder::Clicked && !ROTARY.lastMiddleState) {
    ROTARY.middle = true;
    menu.pressButtonAction();
  }
  ROTARY.lastMiddleState = ( buttonState == ClickEncoder::Clicked); 

  //Check which direction the rotary is going
  ROTARY.value += encoder->getValue();  
  if (ROTARY.value/2 > ROTARY.last) {
    ROTARY.last = ROTARY.value/2;
    //menu.navigateDownFlag = true; 
    //menu.navigation();
    menu.navigateInMenuDown();
  }
  else   if (ROTARY.value/2 < ROTARY.last) {
    ROTARY.last = ROTARY.value/2;
    //menu.navigateUpFlag = true;
    //menu.navigation();
    menu.navigateInMenuUp();
  }     
}

void refreshomeMenu(){
  if (menu._parent == MAIN_MENU) { 
    refreshMainMenu(); 
  }
  else if(menu._parent == SUB_MENU){
    refreshSubMenu();
  }
  
  
}

void refreshMainMenu(){
  if  (menu._parent != menu._lastParent) { //refresh a menu screen when entering menu page first time
    tft.fillRect(0,62,160,66,ST7735_BLACK);  
    drawMainMenu(); 
    menu._lastParent = MAIN_MENU ;  
  }
  else{
    drawMainMenu(); 
  }       
   
}


void mainMenuAction(){
  if ( menu._child <= INTEGRA) { 
    menu._parent= SUB_MENU;
    menu._lastParent = MAIN_MENU; // flag to be set for csreen refresh when entering one of main screen manus         
  }
  else if(menu._child == PUMP) { // pum manual overwrite    
    updatePumpstatus();
  }
  else if( menu._child == RESET){
    resetDefaults();
  }
}


void refreshSubMenu(){
    if (menu._parent != menu._lastParent) { //refresh a menu screen when entering menu page first time
    tft.fillRect(0,62,160,66,ST7735_BLACK);  
    drawSubMenu();
    menu._lastParent = SUB_MENU;    
  }
  else{
    drawSubMenu();
  }
}



void updatePumpstatus(){
  if (WATERPUMP.state){
    WATERPUMP.state = false;
    tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
    printChild[PUMP] = "PUMP: OFF  ";
    digitalWrite(relayPin, HIGH); // Turn on the pump, which is active LO
  }
  else{
    WATERPUMP.state = true; 
    tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
    printChild[PUMP] = "PUMP: ON   ";
    digitalWrite(relayPin, LOW); // Turn on the pump, which is active LO
  }
}

void drawSubMenu(){ 
  switch(menu._child){
    case SETPOINT:       
      displayItemMenuPage(printChild[SETPOINT], menu.set_setpoint );
      break;
    case FLOW:
      displayItemMenuPage(printChild[FLOW], flowSensor.supplyLimit);
      break;
    case PROPORT:    
      displayItemMenuPage(printChild[PROPORT], PID.kp);
      break;
    case INTEGRA:     
      displayItemMenuPage(printChild[INTEGRA], PID.ki);
      break;
    case PUMP:           
      displayItemMenuPage(printChild[PUMP], (float)WATERPUMP.state);
    break;
  }     
} 

void drawMainMenu() {    
  
  tft.setTextSize(2);  
  tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
  tft.setCursor(5, 40);
  tft.print(F(" MAIN MENU  "));
              
  tft.fillRect(0, 56, 160, 2, ST7735_GREEN);  // Draw a line   
#ifdef DEBUG
  Serial.print(F("frame: "));
  Serial.println(menu.frame);
  Serial.print(F("menuitem: "));
  Serial.println(menu.child);
#endif
  refreshFrame();
}

void displayItemMenuPage(String menuItem, float value){    
  tft.setTextSize(2);
  tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
  tft.setCursor(10, 40);
  tft.print(menuItem);
  tft.fillRect(0,56,160,2,ST7735_GREEN);  //draw a line
  tft.setCursor(15, 70);
  tft.print(F("Value     "));    
  tft.setTextSize(3);
  tft.setCursor(25, 94);
  if(menu._child < FLOW){    //for first two items in menu - adjustable value is in whole numbers
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

void refreshFrame() {
  char firstItem = menu.frame - 1;
  char lastItem = menu.frame + 1;
  char yPos = 62;

  for (int i = firstItem; i <= lastItem; i++, yPos += 22) {
    bool selected = (i == menu._child);
    displayMenuItem(printChild[i], yPos, selected);
    //Serial.println(yPos);
  }
  long finish = millis() - start;
  Serial.print("finished Refreshing the frame in :");  
  Serial.println(finish);  
}


void resetDefaults(){
  menu.set_setpoint = 70;
  flowSensor.supplyLimit = 1.0;
  PID.kp = 1.97;      // Proportional gain - original = 2;
  PID.ki = 0.80;      // Integral gain - original 0.1;
  WATERPUMP.state = true;
  printChild[PUMP] = "PUMP: OFF";    
} 

void printSoilMoisture() {
  curTime = millis();
  timeRemaining = (taskQueue[queueStart].executeTime > curTime) ?
                              (taskQueue[queueStart].executeTime - curTime) : 0;
    
                            
  sprintf(buffer, "%3d", timeRemaining / 1000);       // Right-justified long converted to seconds  

  tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.println(F("Soil dew:"));    
  tft.setCursor(105, 0);    
  tft.println(soilMoisture.percentValu());
  tft.setCursor(145, 0);
  tft.println(F("%"));     
  //2nd line        
  tft.setCursor(0, 20);
  if (taskQueue[queueStart].function == checkMoisture) {
    tft.println("Check in:");
  } 
  else if (taskQueue[queueStart].function == stopPump) {
    tft.println("Pump run:");
  }
  
    tft.setCursor(105, 20);
  tft.setTextColor(ST7735_GREEN, ST7735_BLACK);  
                              
  
  tft.println(buffer); // Display countdown in seconds right side justified
  Serial.println(buffer);
  tft.setCursor(145, 20);
  tft.print(F("s"));     
}
 

//Calculate PID controled for pump runtime by checking if soil moister near setpoint
float computePID(int input) {
  unsigned long currentTime = millis();
  float timeChange = (float)(currentTime - PID.lastTimeChecked) / 1000.0; // Convert to seconds
  
  //if(timeChange <= 0.0){
  //    return 0;
  //}
  // Calculate the error
  float error =  menu.setpointIs() - input;

  // Proportional term
  float pTerm = PID.kp * error;

  // Integral term 
  PID.integral += error * timeChange;
  //if (PID.integral > 1000) PID.integral = 1000;  //limit the Integral to prevent creeping up
  //else if (PID.integral < -1000) PID.integral = -1000;
  float iTerm = PID.ki * PID.integral;
  
  // Derivative term
  float derivative = (error - PID.lastError) / timeChange;
  float dTerm = PID.kd * derivative;

  // Combine all terms
  float output = pTerm + iTerm + dTerm;

  // Save error and time for next calculation
  PID.lastError = error;
  PID.lastTimeChecked = currentTime;
  long finish = millis() - start;
  Serial.print("finished Calculating PID in :");  
  Serial.println(finish);  

  return output;  //time for pump to run  
}

// Function to add a task to the queue
void enqueue(void (*function)(), unsigned long delayTime) {
  unsigned long currentTime = millis();
  taskQueue[queueEnd].function = function;
  taskQueue[queueEnd].executeTime = currentTime + delayTime;
  //taskQueue[queueEnd].isCountingDown = isCounting;
  queueEnd = (queueEnd + 1) % queueSize;
  countdownTime = delayTime; // Set the countdown time  
}

// Function to execute and remove the first task in the queue
void dequeue() {
  if (queueStart != queueEnd) { // Check if the queue is not empty
    unsigned long currentTime = millis();
    if (currentTime >= taskQueue[queueStart].executeTime) {
      taskQueue[queueStart].function(); // Execute the task
      //taskQueue[queueEnd].isCountingDown = isCounting;
      queueStart = (queueStart + 1) % queueSize; // Remove the task from the queue
    }
  }
   
  long finish = millis() - start;
  Serial.print("finished Dequing in :");  
  Serial.println(finish); 
}

// Task to check soil moisture
void checkMoisture(){    
  long currentTime = millis();
   
#ifdef DEBUG
  Serial.print(F("Raw Soil Moisture Level: "));
  Serial.println(soilMoisture.check());  
#endif  
 
  float pidOutput = computePID(soilMoisture.percentValu()); // Calculate the PID output
#ifdef DEBUG 
  Serial.print(F("Soil Moisture Level: "));
  Serial.print(soilMoisture.percentValu());  
  Serial.println(" %");  
  Serial.print(F("Setpoint: "));
  Serial.println(menu.setpointIs());     
  Serial.print(F("PID output in ms: "));
  Serial.println(pidOutput); 
#endif  
  //Check if PID output is positive number and Pump is not running and dayly water supply limit is not exceeding dayly limit
  if (pidOutput > 0 && !WATERPUMP.state && flowSensor.usageTotal < flowSensor.supplyLimit) {
    //isCounting = true;  //now print new data to screen
    WATERPUMP.runTime = pidOutput;   // cast to positive number from PID-calculation
    WATERPUMP.runTimeTotal += WATERPUMP.runTime;         //accumulate the total runtim
    WATERPUMP.runTimeTotal = dataLoggingForTimeTotal(currentTime); //check if 24h overflow happened
#ifdef DEBUG    
    Serial.println(F("Moisture low. Turning on the pump..."));   
    Serial.print(F("pump runtime: "));
    Serial.println(WATERPUMP.runTime); 
    Serial.print(F("ml"));  
#endif    
    digitalWrite(relayPin, LOW); // Turn on the pump, which is active LOW
    WATERPUMP.state = true;    
    enqueue(calculateWaterFlow,0);  //monitor flow meter while pump is on
    //isCounting = true;  //now print new data to screen     
    enqueue(stopPump, WATERPUMP.runTime); // Schedule a task to stop the pump after 30 seconds    
    enqueue(checkMoisture, soilMoisture.checkIntervalIs()); // Re-schedule the moisture check task
  }
  else { 
    //isCounting = true;  //now print new data to screen
#ifdef DEBUG    
    Serial.println(F("Soil Moisture above treashold. Lets check again...in 2 min "));
#endif    
    enqueue(checkMoisture, soilMoisture.checkIntervalIs());  //enter valu in seconds); // Re-schedule the moisture check task 
  }   
}

// Task to stop the pump
void stopPump() {
  if(WATERPUMP.state){  //protection if user decides to increase sampling rate
#ifdef DEBUG
    Serial.print(F("Total pump run time in day:"));
    Serial.println(WATERPUMP.runTimeTotal);    
    Serial.println(F("Turning off the pump..."));
#endif        
    digitalWrite(relayPin, HIGH); // Turn off the relay
    WATERPUMP.state = false;    
  }
}


// Function to calculate water flow and usage
void calculateWaterFlow() {
  //isCounting = true; // Stop the countdown after task execution 
  static unsigned long lastCalcTime = 0;  
  unsigned long currentTime = millis();

  if ((currentTime - lastCalcTime) > 1000) {  // Update every second
    // Calculate the flow rate in liters/min
    float flowRate = ((1000.0 / (millis() - lastCalcTime)) * flowSensor.flowPulseCountIs() / flowSensor.calibrationFactor);

    // Total liters passed
    flowSensor.usageTotal += (flowRate / 60.0);  // Convert flow rate to liters per second
#ifdef DEBUG    
    Serial.print(F("Water used so far: "));
    Serial.print(WATER.usageTotal);
    Serial.println(F(" L"));
#endif    
    // Reset pulse count and update time
    flowSensor.updateFlowPulseCount(0);
    lastCalcTime = currentTime;

    // Check and reset if 24 hours have passed
    flowSensor.usageTotal = dataLoggingForTimeTotal(currentTime); //check if 24h time period overflown and if yes reset value to 0 
    
#ifdef DEBUG        
        Serial.println(F("24 hours passed. Water usage reset."));
#endif 
  }
    
}  

void displayMenuItem(String item, int position, boolean selected){
  if(selected){
    //tft.setTextColor(ST7735_BLACK, ST7735_GREEN);
    tft.setCursor(0, position);
    tft.print(">"+item);
  }
  else{
    //tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
    tft.setCursor(0, position);
    tft.print(" "+item);
  }    
}


#ifndef DEBUG

void ScreenStartUpSequance(){   //run screen intro graphics
  tft.drawRoundRect(0, 0, 158, 126,5, ST7735_BLUE);
  tft.setCursor(8, 10);
  tft.setTextColor(ST7735_GREEN);
  tft.setTextSize(2);
  tft.println(F("SMART GARDEN"));
  tft.setCursor(20, 35);
  tft.println(F("STARTS"));
  tft.setCursor(100, 35);
  tft.println(F("IN"));
  tft.drawRect(6 , 90, 145, 25,ST7735_BLUE);
  
    for (byte i = 11;count > 1;i--){
    count = count - 1;
    startIpDisplay(); 
    delay(250);   
  }
  tft.fillScreen(ST7735_BLACK); // Clear the screen (white background) 
}

//Graphics for screen startup
void startIpDisplay(){
  //tft.drawRoundRect(55, 70, 40,25,5, ST7735_BLACK);
  tft.fillRect(65, 60, 13, 20, ST7735_BLACK);
  tft.setCursor(65, 60);
  int printCount = count-1;
  tft.println(printCount);
  switch (count){
    case 10:tft.fillRect(9, 94, 12, 19, ST7735_RED);
      break;
    case 9:tft.fillRect(23, 94, 12, 19,ST7735_RED);
      break;
    case 8: tft.fillRect(37, 94, 12, 19,ST7735_YELLOW);
      break;
    case 7:  tft.fillRect(51, 94, 12, 19,ST7735_YELLOW);
      break;
    case 6: tft.fillRect(65, 94, 12, 19,ST7735_GREEN);
      break;
    case 5: tft.fillRect(79, 94, 12, 19,ST7735_GREEN);
      break;
    case 4: tft.fillRect(93, 94, 12, 19,ST7735_GREEN);
      break;
    case 3: tft.fillRect(107, 94, 12, 19,ST7735_GREEN);
      break;
    case 2: tft.fillRect(121, 94, 12, 19,ST7735_GREEN);
      break;  
    case 1: tft.fillRect(135, 94, 12, 19,ST7735_GREEN);
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
