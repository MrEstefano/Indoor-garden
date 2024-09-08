
/**************************************************************************************/
/*  Name    : Indoor Garden V33 Menu Class_struct                                                 */
/*  Author  : Stefan Zakutansky ATU.ie student                                        */
/*  Date    : 7. 9.2023                                                              */
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
#define moisturePin A0        /* Capacitive moisture sensor is connected to SOIL on PCB*/
#define drySoil 1023         /* dry soil moisture value from calibration*/
#define wetSoil 250         /* wet soil moisture value from calibration*/

#define F_CPU 16000000UL  // Ensure clock frequency is set properly

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);   


// Flow sensor struct - ugly to read but efective
typedef struct irrigation { const float calibrationFactor = 16.6; volatile unsigned int flowPulseCount = 0; volatile float usageTotal = 0.0; float supplyLimit = 1.0;};irrigation Water_pointer;
irrigation* Water = &Water_pointer;

//Rotary encoder variables
ClickEncoder *encoder;

//Rotary encoder - ugly to read but efective
typedef struct encoderButton {volatile bool middle = false; volatile bool lastMiddleState = false; int last; int value; };encoderButton Rotary_pointer;
encoderButton* Rotary = &Rotary_pointer;

//Pump controll - ugly to read but efective
typedef struct waterControll{ float runTime = 0; volatile bool state = false; float runTimeTotal = 0;}; waterControll  WaterPump_pointer;
waterControll* WaterPump = &WaterPump_pointer;

//PID variables - kp,ki,kd were calibrated
typedef struct controller{ float kp = 1.97; float ki = 0.80; float kd = 1.18; float lastError = 0; float integral = 0; unsigned long lastTimeChecked = 0; };controller PID_pointer;
controller* PID = &PID_pointer;

//Soil Moisture sensor 
typedef struct soil { char buffer[4]; unsigned int moisture = 0; volatile unsigned long checkInterval = 120000;};soil Soil_pointer;
soil* Soil = &Soil_pointer;

enum Menu_item  { SETPOINT, FLOW, PROPORT, INTEGRA, PUMP, RESET, TOTAL_ITEMS};
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
int  dataLoggingForTimeTotal(unsigned long currentTime);
void displayMenuItem();
void displayItemMenuPage(String menuItem, float value);
void pumpOverwrite();
void  resetTodefault();


// Structure for a task in the queue
struct Task {void (*function)(); unsigned long executeTime; volatile bool isCountingDown;};

// Queue for tasks
const int queueSize = 10;
Task taskQueue[queueSize];
int queueStart = 0;
int queueEnd = 0;
unsigned long timeRemaining;


//leave the spaces as they over lay previouse item printed on screen
String printChild[TOTAL_ITEMS]  = {"SETPOINT    ", "WATER LIMIT ", "PID Kp VALUE","PID Ki VALUE","PUMP: OFF   ","RESET       "}; 

class Menu{ 
private:   
  void frameAligmentUP(); //private
  void frameAligmentDown(); //private
  bool upFlag;    //private
  bool downFlag; //private    
public:
  Menu(){}
  char frame = 1;  
  bool flag;
  byte set_setpoint = 90;
  Menu_item _child = SETPOINT;
  Menu_item _lastChild = SETPOINT; 
  Page _parent = MAIN_MENU;
  Page _lastParent = SUB_MENU;
  void valueAdjustment();
  void pressButtonAction();
  void navigateInMenuUp();
  void navigateInMenuDown(); 
  char setpointIs(); 
  char updatePosition(char ); 
};

Menu menu;

void Menu::navigateInMenuUp(){  
  menu.flag = true;  // /raise the flag to update the screen 
  if(_parent == MAIN_MENU){       
    if(_child > SETPOINT){
       _child = static_cast<int>(_child) -1; //cast enum type to integer in order to increment or decrement the count
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
  menu.flag = true;  // /raise the flag to update the screen  
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
  menu.flag = true;  // /raise the flag to update the screen 
  if (_parent == MAIN_MENU){    
    if ( _child <= INTEGRA) { 
      _parent= SUB_MENU;
      _lastParent = MAIN_MENU; // flag to be set for csreen refresh when entering one of main screen manus         
    }
    else if(_child == PUMP) { // pum manual overwrite    
      pumpOverwrite();
    }
    else if( _child == RESET){
      resetTodefault();
        
    }       
  }     
 // menu.updateLastParent()
  else if(_parent == SUB_MENU){      
    _parent = MAIN_MENU; 
    _lastParent = SUB_MENU;   // flag to be set for csreen refresh when entering one of main screen manus       
  }   
   
  Rotary->middle = false;   
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
     Water->supplyLimit  += 0.1 ;
    }
    else if ( _child == PROPORT  ) {
      PID->kp += 0.1;
    }
      else if ( _child == INTEGRA  ) {
      PID->ki += 0.1;      
    }
    upFlag = false;
  } 

  if(downFlag){
    if (_child == SETPOINT) {
      set_setpoint -= 1; 
    }
    else if (_child == FLOW) {     
     Water->supplyLimit -= 0.1 ;
    }
    else if (_child == PROPORT ) {
      PID->kp -= 0.1;    
    }
    else if ( _child == INTEGRA ) {
      PID->ki -= 0.1;    
    }
    downFlag = false;
  }  
  
}
char Menu::setpointIs(){
  return set_setpoint;
}
//End of Menu Class

//global variables
byte count =11;
int value;

//volatile bool isCounting = true; // Indicates if a countdown is active
unsigned long countdownTime = 0; // Holds the countdown time for display

//data logging variables
unsigned long startingTimeStamp = 0;
const unsigned long resetInterval = 86400000UL; // 24 hours in milliseconds
unsigned long start;
unsigned long lastStartTime = 0;


//------------------------------------------------------------------------------
// SETUP
//------------------------------------------------------------------------------

void setup() {
  Serial.begin(9600);
#ifdef DEBUG  
  Serial.begin(9600);
#endif  
  pinMode(relayPin, OUTPUT);
  PORTB |= (1 << 1); // Set pin 13 high
  pinMode(ENCODER_SW, INPUT_PULLUP); // Set encoder button as input with internal pull-up
  pinMode(FLOW_SENSOR_PIN, INPUT);

  // Initialize TFT 1.77 inch screen
  tft.initR(INITR_BLACKTAB);    
  tft.fillScreen(ST7735_BLACK);
  tft.setRotation(1);   //make screen horisontal
  
#ifndef DEBUG
  ScreenStartUpSequance(); //screen grafic startup
#endif  

  noInterrupts();                       // disable all interrupts while setting up 
  Timer1.initialize(2000);   //Rotary Encoder is triggered by Timer 1 overflow period 50ms
  Timer1.attachInterrupt(timerIsr); 

  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), flowSensorISR, RISING);  //Enable ISR
  interrupts();                         // enable all interrupts after setup

  //encoder entity
  encoder = new ClickEncoder(ENCODER_CLK, ENCODER_DT, ENCODER_SW, 1, false); // (uint8_t A, uint8_t B, uint8_t BTN = -1, uint8_t stepsPerNotch = 1, bool active = LOW);
  encoder->setAccelerationEnabled(true);
  Rotary->last = encoder->getValue();

  // Schedule the first moisture check
  enqueue(checkMoisture, 0); // Start meassuring soil moisture immediatel 
  startingTimeStamp = millis();
  menu.flag=true;
}

//------------------------------------------------------------------------------
// MAIN LOOP
//------------------------------------------------------------------------------

void loop() {      
    //Check if button pressed
  ClickEncoder::Button buttonState = encoder->getButton();
  if (buttonState == ClickEncoder::Clicked && !Rotary->lastMiddleState) {  //
    Rotary->middle = true;
    menu.pressButtonAction(); //manipulate variables 
  }
  Rotary->lastMiddleState = ( buttonState == ClickEncoder::Clicked); 
  dequeue(); // Execute tasks as their time comes
  checkRotaryEncoder();  //keep checking rotary encoder If any changes happened
  start = millis();   

   //-------------------------
  unsigned long startedNow = millis(); 
  if ((startedNow - lastStartTime) > 1000) {  // Update every second
    //Keep calculating time remining for a task to executr
    ///unsigned long curTime = millis();
    timeRemaining = (taskQueue[queueStart].executeTime > startedNow) ?
                                (taskQueue[queueStart].executeTime - startedNow) : 0;
    char Elbuff[4]; //for storing count down                         
    sprintf(Elbuff, "%3d", timeRemaining / 1000);       // Right-justified long converted to seconds   
    //print the time remining
    tft.setCursor(105, 20);
    tft.setTextColor(ST7735_GREEN, ST7735_BLACK);     
    tft.println(Elbuff); // Display countdown in seconds right side justified
    Serial.println(Elbuff);
    tft.setCursor(145, 20);
    tft.print(F("s"));    
    lastStartTime = startedNow;
  }
  //Keep calculating time remining for a task to executr
 

  //Only if button is pressed or turned left or right refresh the screen
  if(menu.flag){
    
    if (menu._parent == MAIN_MENU) {  
      if(menu._parent != menu._lastParent) { //refresh a menu screen when entering menu page first time
        tft.fillRect(0,62,160,66,ST7735_BLACK);   
        drawParentpage();
        menu._lastParent = MAIN_MENU ;
      }
      else{
        drawParentpage();
      }
           
    }
    else if(menu._parent == SUB_MENU){    
      if (menu._parent != menu._lastParent) { //refresh a menu screen when entering menu page first time
        tft.fillRect(0,62,160,66,ST7735_BLACK);  
        drawChildpage();  
        menu._lastParent = SUB_MENU;    
      }
      else{
        drawChildpage();  
      }
    }   
      menu.flag = false;   
  }  
}

//------------------------------------------------------------------------------
// SUB-FUNCTIONS
//------------------------------------------------------------------------------


void timerIsr() {  
  encoder->service();   
}

void checkRotaryEncoder(){


  //Check which direction the rotary is going
  Rotary->value += encoder->getValue();  
  //value = Rotary->value>>1; //bittshift right costs less computation then devide by 2
  if (Rotary->value > Rotary->last) {  
    Rotary->last = Rotary->value;
    menu.navigateInMenuDown();  //manipulate variables
    
  }
  else if (Rotary->value < Rotary->last) {
    Rotary->last = Rotary->value;
    menu.navigateInMenuUp();  //manipulate variables
    
  }     
}


// Interrupt service routine for flow sensor
void flowSensorISR() {  
    Water->flowPulseCount++;
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


void pumpOverwrite(){
  if (WaterPump->state){
    WaterPump->state = false;
    tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
    printChild[PUMP] = "PUMP: OFF  ";
    PORTB |= (1 << 1); // // Turn off the pump, which is active LO    
  }
  else{
    WaterPump->state = true; 
    tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
    printChild[PUMP] = "PUMP: ON   ";
    PORTB &= ~(1 << 1); // Turn on the pump, which is active LO
  }  
  

}
 

//Calculate PID controled for pump runtime by checking if soil moister near setpoint
float computePID(int input) {
  unsigned long currentTime = millis();
  float timeChange = (float)(currentTime - PID->lastTimeChecked) / 1000.0; // Convert to seconds
  
  //if(timeChange <= 0.0){
  //    return 0;
  //}
  // Calculate the error
  float error =  menu.setpointIs() - input;

  // Proportional term
  float pTerm = PID->kp * error;

  // Integral term 
  PID->integral += error * timeChange;
  if (PID->integral > 1000) PID->integral = 1000;  //limit the Integral to prevent creeping up
  else if (PID->integral < -1000) PID->integral = -1000;
  float iTerm = PID->ki * PID->integral;
  
  // Derivative term
  float derivative = (error - PID->lastError) / timeChange;
  float dTerm = PID->kd * derivative;

  // Combine all terms
  float output = pTerm + iTerm + dTerm;

  // Save error and time for next calculation
  PID->lastError = error;
  PID->lastTimeChecked = currentTime;
  return output;  //time for pump to run  
}

// Function to add a task to the queue
void enqueue(void (*function)(), unsigned long delayTime) {
  unsigned long currentTime = millis();
  taskQueue[queueEnd].function = function;
  taskQueue[queueEnd].executeTime = currentTime + delayTime;
      tft.setCursor(0, 20);
      if (taskQueue[queueStart].function == checkMoisture) {
        tft.println("Check in:");
      } 
      else if (taskQueue[queueStart].function == stopPump) {
        tft.println("Pump run:");
      }
  queueEnd = (queueEnd + 1) % queueSize;
  countdownTime = delayTime; // Set the countdown time  
}

// Function to execute and remove the first task in the queue
void dequeue() {
  if (queueStart != queueEnd) { // Check if the queue is not empty
    unsigned long currentTime = millis();
    if (currentTime >= taskQueue[queueStart].executeTime) {
      taskQueue[queueStart].function(); // Execute the task
      //print to screen what task is executed        
  
      //taskQueue[queueEnd].isCountingDown = isCounting;
      queueStart = (queueStart + 1) % queueSize; // Remove the task from the queue
    }
  }
 
#ifdef DEBUG   
  long finish = millis() - start;
  Serial.print("finished Dequing in :");  
  Serial.println(finish); 
#endif  
}

// Task to check soil moisture
void checkMoisture(){    
  long currentTime = millis();  
  Soil->moisture = analogRead(moisturePin); // Read the moisture leve
#ifdef DEBUG
  Serial.print(F("Raw Soil Moisture Level: "));
  Serial.println(Soil->moisture);  
#endif  
  Soil->moisture = map(Soil->moisture ,drySoil ,wetSoil ,0 ,100); // map the range in percantage                             
  sprintf(Soil->buffer, "%3d",Soil->moisture);  
  float pidOutput = computePID(Soil->moisture); // Calculate the PID output
#ifdef DEBUG 
  Serial.print("Soil Moisture Level: ");
  Serial.print(Soil->moisture);  
  Serial.println(" %");  
  Serial.print(F("Setpoint: "));
  Serial.println(menu.setpointIs());     
  Serial.print(F("PID output in ms: "));
  Serial.println(pidOutput); 
#endif  
  //print updated soil moisture    
  tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.println(F("Soil dew:"));    
  tft.setCursor(105, 0);      
  tft.println(Soil->buffer);
  tft.setCursor(145, 0);
  tft.println(F("%"));   
  //Check if PID output is positive number and Pump is not running and dayly water supply limit is not exceeding dayly limit
  if (pidOutput > 0 && !WaterPump->state && Water->usageTotal < Water->supplyLimit) {
    //isCounting = true;  //now print new data to screen
    WaterPump->runTime = pidOutput;   // cast to positive number from PID-calculation
    WaterPump->runTimeTotal += WaterPump->runTime;         //accumulate the total runtim
    WaterPump->runTimeTotal = dataLoggingForTimeTotal(currentTime); //check if 24h overflow happened
#ifdef DEBUG    
    Serial.println(F("Moisture low. Turning on the pump..."));   
    Serial.print(F("pump runtime: "));
    Serial.println(WaterPump->runTime); 
    Serial.print(F("ml"));  
#endif    
    PORTB &= ~(1 << 1); // Set pin 13 low
    WaterPump->state = true;    
    enqueue(calculateWaterFlow,0);  //monitor flow meter while pump is on
    //isCounting = true;  //now print new data to screen     
    enqueue(stopPump, WaterPump->runTime); // Schedule a task to stop the pump after 30 seconds    
    enqueue(checkMoisture, Soil->checkInterval); // Re-schedule the moisture check task
  }
  else { 
    //isCounting = true;  //now print new data to screen
#ifdef DEBUG    
    Serial.println(F("Soil Moisture above treashold. Lets check again...in 2 min "));
#endif    
    enqueue(checkMoisture, Soil->checkInterval);  /// Re-schedule the moisture check task 
  }   
}

// Task to stop the pump
void stopPump() {
  if(WaterPump->state){  //protection if user decides to increase sampling rate
#ifdef DEBUG
    Serial.print(F("Total pump run time in day:"));
    Serial.println(WATERPUMP.runTimeTotal);    
    Serial.println(F("Turning off the pump..."));
#endif        
    PORTB |= (1 << 1); // Set pin 13 high
    WaterPump->state = false;    
  }
}

void resetTodefault(){
  menu.set_setpoint = 90;
  Water->supplyLimit = 1.0;
  PID->kp = 1.97;      // Proportional gain - original = 2;
  PID->ki = 0.80;      // Integral gain - original 0.1;
  WaterPump->state = true;
  printChild[PUMP] = "PUMP: OFF";   
  
}


// Function to calculate water flow and usage
void calculateWaterFlow() {
  //isCounting = true; // Stop the countdown after task execution 
  static unsigned long lastCalcTime = 0;  
  unsigned long currentTime = millis();

  if ((currentTime - lastCalcTime) > 1000) {  // Update every second
    // Calculate the flow rate in liters/min
    float flowRate = ((1000.0 / (millis() - lastCalcTime)) * Water->flowPulseCount / Water->calibrationFactor);

    // Total liters passed
    Water->usageTotal += (flowRate / 60.0);  // Convert flow rate to liters per second
#ifdef DEBUG    
    Serial.print(F("Water used so far: "));
    Serial.print(Water->usageTotal);
    Serial.println(F(" L"));
#endif    
    // Reset pulse count and update time
    Water->flowPulseCount = 0;
    lastCalcTime = currentTime;

    // Check and reset if 24 hours have passed
    Water->usageTotal = dataLoggingForTimeTotal(currentTime); //check if 24h time period overflown and if yes reset value to 0 
    
#ifdef DEBUG        
        Serial.println(F("24 hours passed. Water usage reset."));
#endif 
  }
    
}  

void drawParentpage(){   

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
  displayMenuItem();  
}  

void drawChildpage(){  
  switch(menu._child){
    case SETPOINT:       
      displayItemMenuPage(printChild[SETPOINT], menu.set_setpoint );
      break;
    case FLOW:
      displayItemMenuPage(printChild[FLOW], Water->supplyLimit);
      break;
    case PROPORT:    
      displayItemMenuPage(printChild[PROPORT], PID->kp);
      break;
    case INTEGRA:     
      displayItemMenuPage(printChild[INTEGRA], PID->ki);
      break;
    case PUMP:           
      displayItemMenuPage(printChild[PUMP], (float)WaterPump->state);
    break;
  }     
} 

void displayMenuItem(){
  char firstItem = menu.frame - 1;
  char lastItem = menu.frame + 1;
  char yPos = 62;
  for (int i = firstItem; i <= lastItem; i++, yPos += 22) {
    
    if(i == menu._child){
      //tft.setTextColor(ST7735_BLACK, ST7735_GREEN);
      tft.setCursor(0, yPos);
      tft.print(">"+printChild[i]);
    }
    else{
      //tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
      tft.setCursor(0, yPos);
      tft.print(" "+printChild[i]);
    }
  }   
#ifndef DEBUG  
  
  long finish = millis() - start;
  Serial.print("finished where cursor is :");
  Serial.println(finish); 
  
#endif  
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
  startIpDisplay(); 
  tft.fillScreen(ST7735_BLACK); // Clear the screen (white background) 
}

//Graphics for screen startup
void startIpDisplay(){
  for (byte i = 11;count > 1;i--){
    count = count - 1;
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
    delay(250);
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
