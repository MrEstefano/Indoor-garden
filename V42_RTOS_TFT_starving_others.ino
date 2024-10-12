#include <Arduino_FreeRTOS.h>
//#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).
#include <timers.h>
#include <event_groups.h>
#include <Adafruit_GFX.h>  
#include <Adafruit_Sensor.h> 
#include <Adafruit_ST7735.h>
#include <Wire.h>
#include <SPI.h>

#define SOLENOID_ONE A4
#define SOLENOID_TWO A5

#define TFT_CS  6
#define TFT_RST 11  
#define TFT_DC  10
#define TFT_SCLK 7  
#define TFT_MOSI 8

#define DISPLAY_UPDATE_FLAG   ( 1 << 0 ) //bit 0 used

EventGroupHandle_t event;

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST); 


#define LOW_STACK_SIZE   64  // Stack size in words (128 bytes)
#define MEDIUM_STACK_SIZE 128 //// Stack size in words (256 bytes)
#define HIGH_STACK_SIZE 256 //// Stack size in words (256 bytes)

const uint8_t relayPin = 9;
const uint8_t moisturePin_1 = A0;
const uint8_t moisturePin_2 = A1;

/* dry soil moisture value from calibration*/
const uint16_t drySoil = 450;         
const uint8_t wetSoil  = 185; 
//const uint16_t span = drySoil - wetSoil;

/*
percentage = 100 * (sensorValue - 200) / (1000 - 200.0);

const int min = 200, max = 1000;
const float span = max - min, hundred = 100.0;
percentage = hundred * (sensorValue - min) / span;
*/

//QueueHandle_t queue1;  // Create handle
struct pid {
  float kp;
  float ki;
  float kd;
  bool flag;
  int set_setpoint = 95;  
}; pid PID;

struct PIDState {
  bool solenoid_flag;
  int lastError;
  float integral;
  unsigned long lastTimeChecked;
  float pidOutput;
  int moisture; 
}; 

PIDState pot_1_sensor;
PIDState pot_2_sensor;

// define Tasks 
void TaskreadSoilMoistureONE( void *pvParameters );
void TaskreadSoilMoistureTWO( void *pvParameters );
void TaskUpdateTFTscreen( void *pvParameters );
void TaskpumpControll( void *pvParameters );

const TickType_t xTimeIncrement = pdMS_TO_TICKS(12000); // 1 minutes in ticks

// Declare Timer Handles
TimerHandle_t pumpTimer1, pumpTimer2;


   
// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);
  pinMode(moisturePin_1, INPUT);
  pinMode(moisturePin_2, INPUT);

  pinMode(SOLENOID_ONE, OUTPUT);
  pinMode(SOLENOID_TWO, OUTPUT);
  pinMode(relayPin, OUTPUT);  // Set the relay pin as output
  digitalWrite(relayPin, HIGH);  // Initialize pump to be off (HIGH means off for active low)
 
  //initialize solenoids
  digitalWrite(SOLENOID_ONE, HIGH); 
  digitalWrite(SOLENOID_TWO, HIGH); 

  // Initialize your PID controller, menu, etc.
  // You can also set these values here or keep them in the struct constructor
  PID.kp = 1.97;
  PID.ki = 0.80;
  PID.kd = 1.18;

  tft.initR(INITR_BLACKTAB);    
  tft.fillScreen(ST7735_BLACK);
  tft.setRotation(1);   //make screen horisontal
  tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
  tft.setTextSize(2);

  //Create a mutex object  
  //mutex_0 = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port

  //Create a event 
  event = xEventGroupCreate(); // get an event group handle
  // Create timers for pump 1 and pump 2
  pumpTimer1 = xTimerCreate("PumpTimer1", pdMS_TO_TICKS(1), pdFALSE, (void *)1, pumpTimerCallback1);
  pumpTimer2 = xTimerCreate("PumpTimer2", pdMS_TO_TICKS(1), pdFALSE, (void *)2, pumpTimerCallback2);
  
  xTaskCreate(
    TaskUpdateTFTscreen
    ,  "TFT Screen"  // A name just for humans
    ,  128 // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL //Parameters for the task
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ); //Task Handle

  // Now set up two Tasks to run independently.
  xTaskCreate(
    TaskreadSoilMoistureONE
    ,  "TaskreadSoilMoistureONE"  // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL //Parameters for the task
    ,  0  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL  // Task handle 
    ); //Task Handle
  
  xTaskCreate(
    TaskreadSoilMoistureTWO
    ,  "TaskreadSoilMoistureTWO" // A name just for humans
    ,  128  // Stack size
    ,  NULL //Parameters for the task
    ,  0  // Priority
    ,  NULL 
    ); //Task Handle

  vTaskStartScheduler();
}



float computePID(int input, PIDState* pidState) {
  unsigned long currentTime = millis();
  float timeChange = (float)(currentTime - pidState->lastTimeChecked) / 1000.0; // Convert to seconds

  // Calculate the error
  int error = PID.set_setpoint - input;

  // Proportional term
  float pTerm = PID.kp * error;

  // Integral term
  pidState->integral += error * timeChange;
  //if (pidState->integral > 1000) pidState->integral = 1000;  // limit the Integral to prevent creeping up
  //else if (pidState->integral < -1000) pidState->integral = -1000;
  float iTerm = PID.ki * pidState->integral;

  // Derivative term
  float derivative = (error - pidState->lastError) / timeChange;
  float dTerm = PID.kd * derivative;

  // Combine all terms
  float output = pTerm + iTerm + dTerm;

  // Save error and time for next calculation
  pidState->lastError = error;
  pidState->lastTimeChecked = currentTime;

  return output;  // time for pump to run
}

// Callback function for Pump 1
void pumpTimerCallback1(TimerHandle_t xTimer) {
  //vTaskDelay(pdMS_TO_TICKS(6));
  digitalWrite(SOLENOID_ONE, HIGH);  // Turn off the solenoid
  PORTB |= (1 << 1);
  // Reset any state variables related to the pump if necessary
}

// Callback function for Pump 2
void pumpTimerCallback2(TimerHandle_t xTimer) {
  //vTaskDelay(pdMS_TO_TICKS(6));
  digitalWrite(SOLENOID_TWO, HIGH);  // Turn off the solenoid
  PORTB |= (1 << 1);
  // Reset any state variables related to the pump if necessary
}

/*----------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------- Tasks -----------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------*/


void TaskUpdateTFTscreen(void *pvParameters) {
  EventBits_t wait_for_Pot_1_update_event;
  wait_for_Pot_1_update_event = xEventGroupWaitBits(event,DISPLAY_UPDATE_FLAG, pdTRUE, pdFALSE, portMAX_DELAY);
  while (1) {   
    //vTaskDelay(pdMS_TO_TICKS(1000)); // 100 ms delay for smoother updates   
    //if(wait_for_Pot_1_update_event & DISPLAY_UPDATE_FLAG != 0){          
       // POT 1
      tft.setCursor(0, 0);
      tft.println(F("Pot one:")); 
      //event POT 1 
      tft.setCursor(105, 0); 
      tft.println(pot_1_sensor.moisture);
      tft.setCursor(145, 0);
      tft.println(F("%"));
      // POT2
      tft.setCursor(0, 20);
      tft.println(F("Pot two:"));    
      //event POT 2
      tft.setCursor(105, 20); 
      tft.println(pot_2_sensor.moisture);
      tft.setCursor(145, 20);
      tft.println(F("%"));
   // }
  vTaskDelay(pdMS_TO_TICKS(3000)); // 100 ms delay for smoother updates
  }
}

void TaskreadSoilMoistureONE( void *pvParameters ) { 
  TickType_t xLastWakeTime = xTaskGetTickCount(); // Initialize with current tick count  
  while (1){   
    unsigned long currentTime = millis();
    if (((currentTime / 12000) % 2) == 0) {  // even minute check
      //xSemaphoreTake( mutex_0, portMAX_DELAY );
      //pot_1_sensor.moisture = 100 * (drySoil  - analogRead(moisturePin_1) ) / span;                    
      pot_1_sensor.moisture = map(analogRead(moisturePin_1) ,drySoil ,wetSoil ,0 ,100); // map the rang     
      pot_1_sensor.pidOutput = computePID(pot_1_sensor.moisture, &pot_1_sensor);  
      //xEventGroupSetBits( event, DISPLAY_UPDATE_FLAG );
      
      Serial.println("pot 1:");
      Serial.println(pot_1_sensor.pidOutput);
      if (pot_1_sensor.pidOutput > 0){        
        digitalWrite(SOLENOID_ONE, LOW);  // Turn on the solenoid (active low)
        PORTB &= ~(1 << 1); 
        // Set timer duration to the PID runtime output
        xTimerChangePeriod(pumpTimer1, pdMS_TO_TICKS(pot_1_sensor.pidOutput), 0);  
        xTimerStart(pumpTimer1, 0);  // Start the timer for pump 1
        vTaskDelay(pdMS_TO_TICKS(10));
      }   
      //xSemaphoreGive( mutex_0 ); // Now free or "Give" the Serial Port for others.     
    }  
  vTaskDelayUntil(&xLastWakeTime, xTimeIncrement);  
  }
}

void TaskreadSoilMoistureTWO(void *pvParameters){  // This is a Task.
  TickType_t xLastWakeTime = xTaskGetTickCount(); // Initialize with current tick count; // Initialize with 2 min delay current tick count 
  while (1){   
    unsigned long currentTime = millis();
    if (((currentTime / 12000) % 2) == 1) {  // Odd minute check
      //pot_2_sensor.moisture =  100 * (drySoil  - analogRead(moisturePin_2)) / span;   
       pot_2_sensor.moisture = map(analogRead(moisturePin_2) ,drySoil ,wetSoil ,0 ,100); // map the rang                          
      pot_2_sensor.pidOutput = computePID(pot_2_sensor.moisture, &pot_2_sensor); 
      //xEventGroupSetBits( event, DISPLAY_UPDATE_FLAG );
      
      Serial.println("pot 2:");
      Serial.println(pot_2_sensor.pidOutput);
      if (pot_2_sensor.pidOutput > 0){
        digitalWrite(SOLENOID_TWO, LOW);
        PORTB &= ~(1 << 1); 
        xTimerChangePeriod(pumpTimer2, pdMS_TO_TICKS(pot_2_sensor.pidOutput), 0);  
        xTimerStart(pumpTimer2, 0);  // Start the timer for pump 2
        vTaskDelay(pdMS_TO_TICKS(10));
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xTimeIncrement);  
  }
}

void loop(){ /*Empty. Things are done in Tasks*/}

