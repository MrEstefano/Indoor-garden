#include <Arduino_FreeRTOS.h>
#include <timers.h>
#include <event_groups.h> 
#include <Adafruit_ST7735.h>
//#include <FreeRTOSConfig.h> 




#define SOLENOID_ONE A4
#define SOLENOID_TWO A5

#define TFT_CS  6
#define TFT_RST 11  
#define TFT_DC  10
#define TFT_SCLK 7  
#define TFT_MOSI 8

#define GET_POT1   ( 1 << 0 ) //bit 0 used
#define GET_POT2   ( 1 << 1 ) //bit 0 used

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST); 

const uint8_t relayPin = 9;
const uint8_t moisturePin_1 = A0;
const uint8_t moisturePin_2 = A1;

/* dry soil moisture value from calibration*/
const uint16_t drySoil = 450;         
const uint8_t wetSoil  = 185; 
const uint16_t span = drySoil - wetSoil;

/*
percentage = 100 * (sensorValue - 200) / (1000 - 200.0);

const int min = 200, max = 1000;
const float span = max - min, hundred = 100.0;
percentage = hundred * (sensorValue - min) / span;
*/

//QueueHandle_t queue1;  // Create handle
struct controller {
  float kp;
  float ki;
  float kd;
  int setpoint = 95;  
}; controller PID_pointer; // Declare the controller instance
controller* PID = &PID_pointer; // Create a pointer to the controller

struct PIDState {
  char lastError;
  float integral;
  unsigned long lastTimeChecked;
  //TickType_t lastTimeChecked;
  float pidOutput;
  int moisture; 
}; 

PIDState pot_1_sensor;
PIDState pot_2_sensor;

// define Tasks 
void TaskPot1( void *pvParameters );
void TaskPot2( void *pvParameters );
void TaskTFT( void *pvParameters );
void TaskFree( void *pvParameters );


const TickType_t xTimeIncrement = pdMS_TO_TICKS(12000); // 1 minutes in ticks

// Declare Timer Handles
TimerHandle_t pumpTimer1, pumpTimer2;

// Declare Event Handles
EventGroupHandle_t event;
   
// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);

  pinMode(moisturePin_1, INPUT);
  pinMode(moisturePin_2, INPUT);
  pinMode(SOLENOID_ONE, OUTPUT);
  pinMode(SOLENOID_TWO, OUTPUT);
  pinMode(relayPin, OUTPUT);  // Set the relay pin as output

  // Initialize pump to be off (HIGH means off for active low)
  PORTB |= (1 << 1);  
 
  //initialize solenoids
  PORTC |= (1 << 4);   //digitalWrite(SOLENOID_ONE, HIGH);  // Turn off the solenoid
  PORTC |= (1 << 5);

  // Initialize your PID controller, menu, etc.
  // You can also set these values here or keep them in the struct constructor
  PID->kp = 1.97;
  PID->ki = 0.80;
  PID->kd = 1.18;

  tft.initR(INITR_BLACKTAB);    
  tft.fillScreen(ST7735_BLACK);
  tft.setRotation(1);   //make screen horisontal
  tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
  tft.setTextSize(2);

  tft.setCursor(0, 0);
  tft.println(F("Pot one:")); 

  tft.setCursor(145, 0);
  tft.println(F("%"));
   
  tft.setCursor(0, 20);
  tft.println(F("Pot two:"));    

  tft.setCursor(145, 20);
  tft.println(F("%"));
  
  pot_1_sensor.pidOutput = 0;    
  pot_2_sensor.pidOutput = 0;   

  pot_1_sensor.moisture = 100 * (drySoil  - analogRead(moisturePin_1) ) / span; //saving memory, rather using map() func           
  pot_2_sensor.moisture = 100 * (drySoil  - analogRead(moisturePin_2) ) / span; //saving memory, rather using map() func         
  //Create a event 
  event = xEventGroupCreate(); // get an event group handle
  // Create timers for pump 1 and pump 2
  pumpTimer1 = xTimerCreate("PumpTimer1", pdMS_TO_TICKS(1), pdFALSE, (void *)1, pumpTimerCallback1);
  pumpTimer2 = xTimerCreate("PumpTimer2", pdMS_TO_TICKS(1), pdFALSE, (void *)2, pumpTimerCallback2);
  
  xTaskCreate(
    TaskFree
    ,  "Free Ram"  // A name just for humans
    ,  48 // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL //Parameters for the task
    ,  0  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ); //Task Handle

   xTaskCreate(
    TaskTFT
    ,  "TFT Screen"  // A name just for humans
    ,  168 // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL //Parameters for the task
    ,  0  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ); //Task Handle

    
  // Now set up two Tasks to run independently.
  xTaskCreate(
    TaskPot1
    ,  "TaskreadSoilMoistureONE"  // A name just for humans
    ,  168  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL //Parameters for the task
    ,  0  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL  // Task handle 
    ); //Task Handle
  
  xTaskCreate(
    TaskPot2
    ,  "TaskreadSoilMoistureTWO" // A name just for humans
    ,  168  // Stack size 168 was good
    ,  NULL //Parameters for the task
    ,  0  // Priority
    ,  NULL 
    ); //Task Handle

  vTaskStartScheduler();
}


float computePID(int input, PIDState* pidState, float kp, float ki, float kd, int setpoint){ 

  unsigned long currentTime = millis();
  float timeChange = (float)(currentTime - pidState->lastTimeChecked) / 1000.0; // Convert to seconds

  // Calculate the error
  char error = setpoint - input;

  // Proportional term
  float pTerm = kp * error;

  // Integral term
  pidState->integral += error * timeChange;
  //if (pidState->integral > 1000) pidState->integral = 1000;  // limit the Integral to prevent creeping up
  //else if (pidState->integral < -1000) pidState->integral = -1000;
  float iTerm = ki * pidState->integral;

  // Derivative term
  float derivative = (error - pidState->lastError) / timeChange;
  float dTerm = kd * derivative;

  // Combine all terms
  float output = pTerm + iTerm + dTerm;

  // Save error and time for next calculation
  pidState->lastError = error;
  pidState->lastTimeChecked = currentTime;

  return output;  // time for pump to run
}

// Callback function for Pump 1
void pumpTimerCallback1(TimerHandle_t xTimer) {
  PORTC |= (1 << 4);   //digitalWrite(SOLENOID_ONE, HIGH);  // Turn off the solenoid
  PORTB |= (1 << 1);
}

// Callback function for Pump 2
void pumpTimerCallback2(TimerHandle_t xTimer) {
  PORTC |= (1 << 5); //digitalWrite(SOLENOID_TWO, HIGH);  // Turn off the solenoid
  PORTB |= (1 << 1); //digitalWrite(pumpRelay, HIGH);  // Turn off the solenoid
}

int freeRam() {
  //If __brkval is zero nothing has been allocated on the heap - so use heap start
  //If __brkval is non zero it indicates the top of the heap
  extern int __heap_start, *__brkval;

  //Calculate the free RAM between the top of the heap and top of the stack
  //This new variable is on the top of the stack
  int l_total = 0;
  if (__brkval == 0)
	l_total = (int) &l_total - (int) &__heap_start;
  else
	l_total = (int) &l_total - (int) __brkval;
  //
  l_total -= sizeof(l_total); //Because free RAM starts after this local variable
  return l_total;
}

/*----------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------- Tasks -----------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------*/
void TaskFree(void *pvParameters) {
  (void*) pvParameters;
  while (1) {   
    freeRam();
    vTaskDelay(pdMS_TO_TICKS(100000)); // 100 s delay 
  }
}

void TaskTFT(void *pvParameters) {
  (void*) pvParameters;
  while (1) {   
    xEventGroupWaitBits(event,GET_POT1 | GET_POT2, pdTRUE, pdTRUE, portMAX_DELAY);      
    // do not interrupt SPI communication with TFT
    noInterrupts();
    // POT 1
    tft.setCursor(105, 0); 
    tft.println(pot_1_sensor.moisture);
    // POT2
    tft.setCursor(105, 20); 
    tft.println(pot_2_sensor.moisture);
    //flush the TFT pipe
    tft.flush();
    //allow interrupts back for use by timeras
    interrupts();
    vTaskDelay(pdMS_TO_TICKS(1000)); // 100 ms delay for smoother updates
  }
}

void TaskPot1( void *pvParameters ) { 
  (void*) pvParameters; 
  TickType_t xLastWakeTime = xTaskGetTickCount(); // Initialize with current tick count  
  while (1){   
    TickType_t currentTime = xTaskGetTickCount(); 
    if (((currentTime / pdMS_TO_TICKS(12000)) % 2) == 0) {  // even minute check
      //xSemaphoreTake( mutex_0, portMAX_DELAY );
      pot_1_sensor.moisture = 100 * (drySoil  - analogRead(moisturePin_1) ) / span; //saving memory, rather using map() func                    
      pot_1_sensor.pidOutput = computePID(pot_1_sensor.moisture, &pot_1_sensor, PID->kp, PID->ki, PID->kd, PID->setpoint);  
      Serial.println("pot 1:");
      Serial.println(pot_1_sensor.pidOutput);
      xEventGroupSetBits( event, GET_POT1 );       
      if (pot_1_sensor.pidOutput > 0){               
        PORTC &= ~(1 << 4); //digitalWrite(SOLENOID_ONE, LOW);  // Turn on the solenoid (active low)
        PORTB &= ~(1 << 1); 
        // Set timer duration to the PID runtime output
        xTimerChangePeriod(pumpTimer1, pdMS_TO_TICKS(pot_1_sensor.pidOutput), 0);  
        xTimerStart(pumpTimer1, 0);  // Start the timer for pump 1
      }   
    }  
  vTaskDelayUntil(&xLastWakeTime, xTimeIncrement);  
  }
}

void TaskPot2(void *pvParameters){  // This is a Task.
  TickType_t xLastWakeTime = xTaskGetTickCount(); // Initialize with current tick count; // Initialize with 2 min delay current tick count 
  (void*) pvParameters;
  while (1){   
    TickType_t currentTime = xTaskGetTickCount(); 
    if (((currentTime / pdMS_TO_TICKS(12000)) % 2) == 1) {  // even minute check
      pot_2_sensor.moisture =  100 * (drySoil  - analogRead(moisturePin_2)) / span;   
      pot_2_sensor.pidOutput = computePID(pot_2_sensor.moisture, &pot_1_sensor, PID->kp, PID->ki, PID->kd, PID->setpoint);  
      Serial.println("pot 2:");
      Serial.println(pot_2_sensor.pidOutput);
      xEventGroupSetBits( event, GET_POT2 );      
      if (pot_2_sensor.pidOutput > 0){
        PORTC &= ~(1 << 5); 
        PORTB &= ~(1 << 1); 
        xTimerChangePeriod(pumpTimer2, pdMS_TO_TICKS(pot_2_sensor.pidOutput), 0);  
        xTimerStart(pumpTimer2, 0);  // Start the timer for pump 2
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xTimeIncrement);  
  }
}

void loop(){ /*Empty. Things are done in Tasks*/ }

