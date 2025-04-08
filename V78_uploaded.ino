/**************************************************************************************************************/
/********************** Event-driven Active Object (AO) pattern with FreeRTOS *********************************/
/**************************************************************************************************************/
/*************************************************************************************************************/
/*                     Name    : Indoor Garden V78                                                           */
/*                     Author  : Stefan Zakutansky ATU.ie student                                            */
/*                     Date    : 8. 4.2025                                                                 */
/*                     Notes   : The code architecture accomodates an Active object pattern faciliating      */
/*                               RTOS features, Queues, Timers, Schedulling. The Event generator task        */
/*                               Sens_AO measures a soil moisture via sensor with RS485 interface, and       */
/*                               Enc_AO likewise is acquiring data by checking the rotary encoder,           */   
/*                               at period of 3 ms, to maintain responsiveness when User interacts with.     */
/*                               The periodic timer sends Encoder events every 100 ms, to prevent saturating */
/*                               queue blocks with fast incoming Encoder task data, For human 100 ms is fast */
/*                               enough to see the values being refreshed on TFT screen. User can intercat   */
/*                               with encoder push button, in order to adjust the treashold value            */
/*                               The receiving tasks are Yielding (giving CPU) while waiting for event       */  
/*                               When Event occures, assigned to particular task and section in task, then   */
/*                               Event is proccessed. The receivers like Pump_AO task, gets soil moiture     */
/*                               curent value, calculates the Pump run time with PID helper funcion,         */
/*                               which adapts to enviroment.The sampling rate of 4 minutes provides          */
/*                               favorable soil condition to mantain healthy growth of the plant.            */
/*                               The pump is active LOW driven. The receiving TFT_AO task displayes          */
/*                               the current sensor readings and Target value. Config file was tuned to      */
/*                               optimize a memory usage and prevent unwanted memory leaks                   */       
/*                                                                                                           */
/*************************************************************************************************************/
/*

General Key Principles of the Active Object Pattern:
'''''''''''''''''''''''''''''''''''''''''''''
-Encapsulation: Each AO has its own data and state, only modified by the AO's methods.
-Event Handling and Queues: Events are delivered to AOs through queues.
-Each AO has its own queue, which decouples the tasks from direct data access.
-Asynchronous Execution: AOs execute asynchronously, receiving events in their own task context.

Explanation
''''''''''''
-Queue-driven: Each task operates independently, reacting to incoming events from its queue. 
-This approach maximizes modularity and responsiveness by separating concerns.
- Each Queue block carries a data, which are encapsulated for particular block.
- This protects variables from being used at same time with other task

*/

#include <Arduino_FreeRTOS.h>
#include <timers.h>
#include <queue.h>
#include <task.h>
#include <SoftwareSerial.h>
#include <Adafruit_ST7735.h>
#include <ClickEncoder.h>


// Constants defining Rotary encoder pins
#define ENCODER_CLK     2   // TTL A-chanel
#define ENCODER_DT      3   // TTL B-Chanel
#define ENCODER_SW      4   // Push-Button 

// Constants defining SPI connection to TFT screen
#define TFT_CS   10
#define TFT_RST  12
#define TFT_DC    14
#define TFT_SCLK   13
#define TFT_MOSI   11

#define SENSOR_INTERVAL_MS 500000

// Creating TFT screen object
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST); 

// Software serial object needed for RS485 communication
SoftwareSerial mod(9, 8); // (RX - A4 , TX - A5)

// Rotary Encoder object
//ClickEncoder *encoder;
//encoder = new ClickEncoder(ENCODER_CLK, ENCODER_DT, ENCODER_SW, 1, false); // (uint8_t A, uint8_t B, uint8_t BTN = -1, uint8_t stepsPerNotch = 1, bool active = LOW);
  ClickEncoder encoder(ENCODER_CLK, ENCODER_DT, ENCODER_SW, 4, false);

struct PIDContext {
  float integral;
  int lastError;
  TickType_t lastTimeChecked ;
};


// Active Object Events
typedef enum { 
    EVENT_SENSOR_UPDATE,
    EVENT_ENCODER_UPDATE,
} EventType;

// Event Struct: 
// Event holds the type and data, 
// sent to each AO's queue.
struct Event {
    EventType type;
    void  *data;
}; 

// Rotary Encoder data
struct EncoderData {
  bool adjustMode = false;
  int setpoint = 72;
};EncoderData encoderData;

// Soil sensor data 
struct SensorData {
  int moisture;
  int temperature;
  int setpoint;
};SensorData sensorData;

// Tasks definitions - Active Object Prototypes
void TFT_AO ( void *pvParameters );
void Enc_AO ( void *pvParameters );
void Sens_AO( void *pvParameters );
void Pump_AO( void *pvParameters );

// Declare Timer Handles
TimerHandle_t sensorTimer, pumpTimer;

// Event Queues Handles
QueueHandle_t  sensorQueue, displayQueue;

// the setup function runs once when you press reset or power the board
void setup() {
  // Initialize software serial communication at 4800 baud rate
  mod.begin(4800);      // RS485
 
  //Print once - this does not change
  tft.initR(INITR_BLACKTAB);    
  tft.fillScreen(ST7735_BLACK);  
  tft.setRotation(1);   //make screen horisontal
  tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
  tft.setTextSize(2);

  tft.setCursor(0, 0);
  tft.println(F("Soil dew:")); 

  tft.setCursor(145, 0);
  tft.println(F("%"));
   
  tft.setCursor(0, 20);
  tft.println(F("Soil Tem:"));    

  tft.setCursor(145, 20);
  tft.println(F("C"));

  tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
  tft.setCursor(5, 40);
  tft.print(F("Target"));
  tft.fillRect(0,60,160,2,ST7735_GREEN);  //draw a line
  tft.setCursor(15, 70);
  tft.print(F("Value     "));  
  
  // Create event queues
  sensorQueue = xQueueCreate(3, sizeof(Event));
  displayQueue = xQueueCreate(6, sizeof(Event));
 

  // Single shot Timer - this timer is called by Pump Task, passing a time period calculated by PID controller
  pumpTimer = xTimerCreate("PumpTimer", pdMS_TO_TICKS(1), pdFALSE, (void *)1, pumpTimerCallback1);
  
  // Recursive Timer - triggering the event every 5 minutes
  sensorTimer = xTimerCreate("sensortimer", pdMS_TO_TICKS(SENSOR_INTERVAL_MS), pdTRUE, (void *)1, sensorTimerCallback);  

  //encoderData.setpoint = 55;  // Or whatever your default value is
  Event displayEvent = {EVENT_ENCODER_UPDATE, &encoderData};
  xQueueSend(displayQueue, &displayEvent, 0);  // Ensure initial setpoint is shown
  // Initialize Encoder Object
  //encoder = new ClickEncoder(ENCODER_CLK, ENCODER_DT, ENCODER_SW, 1, false); // (uint8_t A, uint8_t B, uint8_t BTN = -1, uint8_t stepsPerNotch = 1, bool active = LOW);
  ClickEncoder encoder(ENCODER_CLK, ENCODER_DT, ENCODER_SW, 1, false);
  // Encoder Input Event: EncoderAO reads the encoder’s value and sends the target setpoint updates to PumpControlAO.
  xTaskCreate(
    Enc_AO
  ,  "Check encoder"  // A name just for humans
  ,  60     // This stack size 
  ,  NULL   // Parameters for the task
  ,  3        // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
  ,  NULL   // Task Handle
  ); 

  //Display Event: DisplayAO updates the display based on new sensor data.
  xTaskCreate(
    TFT_AO
    ,  "TFT Screen"  // A name just for humans
    ,  155 // This stack size 
    ,  NULL //Parameters for the task
    ,  3   // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL //Task Handle
    ); 

  //Sensor Read and Process Event Flow: The SensorHandlerAO will read sensor data and send events to DisplayAO and PumpControlAO.
  xTaskCreate(
    Sens_AO
    ,  "Sens the soil moisture"  // A name just for humans
    ,  70  // This stack size 
    ,  NULL //Parameters for the task
    ,  3   // Priority, with 3 being the highest, and 0 being the lowest.
    ,  NULL //Task Handle
    ); 

  //Pump Control Event: PumpControlAO adjusts the pump operation based on the setpoint and sensor feedback.
  xTaskCreate(
    Pump_AO
    ,  "TaskreadSoilMoireONE"  // A name just for humans
    ,  70  // This stack size 
    ,  NULL //Parameters for the task
    ,  3    // Priority, with 3 being the highest, and 0 being the lowest.
    ,  NULL // Task handle 
    ); 
  
  xTimerStart(sensorTimer, 0);  // Start the timer imedietly
 
  
  vTaskStartScheduler();
}

// Proportional Integral derivate Calculation for time to run the pump
float computePID(PIDContext* ctx, int input, int setpoint) {
    // Tuned PID values (consider making these configurable)
    static const float kp = 1.30;
    static const float ki = 0.15; 
    static const float kd = 0.45;
    
    // Calculate time difference in seconds
    TickType_t currentTime = xTaskGetTickCount();
    float timeChange = (float)(currentTime - ctx->lastTimeChecked) / configTICK_RATE_HZ;
    if(timeChange <= 0) timeChange = 0.001; // Prevent division by zero
    
    // Calculate error
    int error = setpoint - input;
    
    // Proportional term
    float pTerm = kp * error;
    
    // Integral term with clamping
    ctx->integral += error * timeChange;
    ctx->integral = constrain(ctx->integral, -100.0, 100.0); // Much smaller range
    float iTerm = ki * ctx->integral;
    
    // Derivative term
    float derivative = (error - ctx->lastError) / timeChange;
    float dTerm = kd * derivative;
    
    // Combine terms and limit output
    float output = pTerm + iTerm + dTerm;
    output = constrain(output, 0, 10.0); // Max 10 seconds runtime
    
    // Save state
    ctx->lastError = error;
    ctx->lastTimeChecked = currentTime;
    
    return output;
}


// Timer Callback function for Pump to turn of, after the calculated period expires
void pumpTimerCallback1(TimerHandle_t xTimer) {
  // Turn off the pump by setting the bit high (active low)
  PORTD |= (1 << 7);  
}

// The Pump is triggered every 5 minutes
void sensorTimerCallback(TimerHandle_t xTimer) {
    // Dynamically allocate memory for the data
  SensorData *dataCopy = (SensorData *) pvPortMalloc(sizeof(SensorData));
  if (dataCopy == NULL) return;  // Check allocation

  // Copy values safely
  dataCopy->moisture = sensorData.moisture;
  dataCopy->temperature = sensorData.temperature;
  dataCopy->setpoint = encoderData.setpoint;

  Event sensorEvent = {EVENT_SENSOR_UPDATE, dataCopy};

  // Try sending to the queue
  if (xQueueSend(sensorQueue, &sensorEvent, 0) != pdTRUE) {
    vPortFree(dataCopy); // Free memory if queue is full
  }

}

void printPaddedNumber(int number) {
  taskENTER_CRITICAL();  // Only one critical section needed for TFT
  if (number < 10) {
    tft.print(F("  "));
  } else if (number < 100) {
    tft.print(F(" "));
  }
  tft.print(number);
  taskEXIT_CRITICAL();  // End critical section after all TFT writes
}

// Polling helper function
void delayMs(uint32_t ms) {
  TickType_t start = xTaskGetTickCount();
  while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(ms)) {
    // Busy wait
     __asm__ __volatile__("nop"); // Minimal CPU stall
  }
}
/*----------------------------------------------------------------------------------------------------------*/
/*-------------------------------Event-driven Active Object (AO) -------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------*/

// Task acquires data from Sensor and sends Event to dedicated receiver, with aid of timers
// Polling is used instead of vTaskdelay() - Intentionaly the CPU is not being released, 
// to keep the request and respons flowless, otherwise transmition will be broken
void Sens_AO(void *pvParameters) {
  (void*) pvParameters;
  
  static const byte soilSensorRequest[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B};
  byte soilSensorResponse[9];
  while (1) {         
    // send request to RS485 device
    mod.write(soilSensorRequest, sizeof(soilSensorRequest));
    //poll untoil software serial buffer is full of 9 bytes
    //delayMs(10);   
    // have the 1 second timeout pre-calculated using RTOS timer tick 
    TickType_t startTime = xTaskGetTickCount();
    while (mod.available() < 9) {
        if (xTaskGetTickCount() - startTime > pdMS_TO_TICKS(1000)) {
            break; // Timeout reached, abort read
        }
        delayMs(1); // Keep task execution atomic
    }
    // When buffer is already full, store the data to an array
    if (mod.available() >= 9) {
      for (byte i = 0; i < 9; i++) {
        soilSensorResponse[i] = mod.read();
        delayMs(1);  //poll to give the serial port a chance to store byte values for each index
      } 
      //mod.flush(); 
      // Extract the moisturefrom array
      int moistureRaw = int(soilSensorResponse[3] << 8 | soilSensorResponse[4]) / 10;
      // limit the moisture to range 0 - 100
      if (moistureRaw > 100){ sensorData.moisture = 100;}  
      else if(moistureRaw < 0){ sensorData.moisture = 0;}
      else{sensorData.moisture = moistureRaw;}

      // Extract the temperature from array
      int temperatureRaw = int(soilSensorResponse[5] << 8 | soilSensorResponse[6]);
      sensorData.temperature =  temperatureRaw / 10  ;
      // (temperatureRaw > 0xFE6F) ? -(0x10000 - temperatureRaw) / 10 :
      // Create event
      Event sensorEvent = {EVENT_SENSOR_UPDATE, &sensorData};
      // Send the event to disply task Imedietly at 400ms rate
      xQueueSend(displayQueue, &sensorEvent,(TickType_t) 0);  
      // Second event is sent tu pump task by periodic timer every 5 min
    }
    // 450 ms delay for smoother updates  
    vTaskDelay(pdMS_TO_TICKS(1000));  
  }
}   

// The task acquires data, Timer sends the Event
void Enc_AO(void *pvParameters) {
  (void*) pvParameters;
  encoder.setAccelerationEnabled(true); 
  int value  ;
  int last = encoder.getValue() ; 
  while (1) {   
    // Update encoder state
    encoder.service();  
    // Check for button click to toggle adjust mode
    if (encoder.getButton() == ClickEncoder::Clicked) {
      encoderData.adjustMode = !encoderData.adjustMode;
      Event event = {EVENT_ENCODER_UPDATE, &encoderData};
      xQueueSend(displayQueue, &event, 0);
    }      
    // Check button press to toggle adjust mode 
    if (encoderData.adjustMode) {   
      value += encoder.getValue();
      if (value != last ){      
        if ( value > last) { 
          encoderData.setpoint--;   
        }  
        else if (value < last)  {
          encoderData.setpoint++; 
        }
        // limit the setpoint to range 0 - 100  
        encoderData.setpoint = constrain(encoderData.setpoint, 0, 100);  
        last = value; 
        // Create the event - in other words pickup variables from structure and store them in Event structure
        Event encoderEvent = {EVENT_ENCODER_UPDATE, &encoderData};
        // send Even every 100 milliseconds (readinc encoder is 10x faster, no need to overload the event stream)
        xQueueSend(displayQueue, &encoderEvent, 0);  

      } 
    }
    // 10ms delay for responsivenes
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}

// Task becomes active when Event arrives
void TFT_AO(void *pvParameters) {
  (void*) pvParameters; 
  Event event;  
  while (1) {  


    if (xQueueReceive(displayQueue, &event, 0) == pdTRUE) {
      configASSERT(event.data != nullptr);

      taskENTER_CRITICAL();  // Only one critical section needed for TFT

      if (event.type == EVENT_ENCODER_UPDATE) {
        EncoderData *data = (EncoderData*) event.data;

        tft.setTextSize(3);
        tft.setCursor(25, 94);
        tft.setTextColor(data->adjustMode ? ST7735_BLACK : ST7735_GREEN,
                         data->adjustMode ? ST7735_GREEN : ST7735_BLACK);

        // Draw with highlighting if in adjust mode
        tft.print(data->adjustMode ? ">" : " ");
        printPaddedNumber(data->setpoint);
        tft.print(data->adjustMode ? "<" : " ");
      }

      else if (event.type == EVENT_SENSOR_UPDATE) {
        SensorData *data = (SensorData*) event.data;

        tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
        tft.setTextSize(2);

        tft.setCursor(105, 0);
        printPaddedNumber(data->moisture);

        tft.setCursor(105, 20);
        printPaddedNumber(data->temperature);
      }
      taskEXIT_CRITICAL();  // End critical section after all TFT writes
    }
    taskYIELD();  // Yield CPU
  }
}

// Task becomes active when Event arrives
void Pump_AO(void *pvParameters) {
    (void*) pvParameters;    
    Event event;
    static PIDContext ctx = {0};
    
    // Pump control setup
    DDRD |= 0x80;
    PORTD |= (1 << 7); // Start with pump off
    
    // Minimum/maximum pump run times (in ms)
    const TickType_t MIN_PUMP_TIME = pdMS_TO_TICKS(100);  // 100ms minimum
    const TickType_t MAX_PUMP_TIME = pdMS_TO_TICKS(10000); // 10s maximum
    
    while (1) {
        if (xQueueReceive(sensorQueue, &event, 0) == pdTRUE) {
            if (event.type == EVENT_SENSOR_UPDATE) {
                SensorData *data = (SensorData*) event.data;
                
                // Only activate pump if significantly below setpoint
                if(data->moisture < (data->setpoint - 2)) { // 2% deadband
                    float pidOutput = computePID(&ctx, data->moisture, data->setpoint);
                    
                    // Convert to ticks and constrain
                    TickType_t pumpTime = pdMS_TO_TICKS(pidOutput * 1000);
                    pumpTime = constrain(pumpTime, MIN_PUMP_TIME, MAX_PUMP_TIME);
                    
                    // Activate pump
                    PORTD &= ~(1 << 7); // Turn on
                    xTimerChangePeriod(pumpTimer, pumpTime, 0);
                    xTimerStart(pumpTimer, 0);
                }
                vPortFree(data);
            }
        }  
        taskYIELD();
    }
}

void loop(){ /*Empty. Things are done in Tasks*/ }

/*----------------------------------------------------------
Sketch uses 24938 bytes (77%) of program storage space. Maximum is 32256 bytes.
Global variables use 833 bytes (40%) of dynamic memory, leaving 1215 bytes for local variables. Maximum is 2048 bytes.

Adjustment done in RTOSConfig.h file

#define configTICK_RATE_HZ   ( ( TickType_t ) 1000 )

#define configHEAP_CLEAR_MEMORY_ON_FREE     0


#define configUSE_PORT_DELAY                1


#define configUSE_PREEMPTION                0//1

#define configCPU_CLOCK_HZ                  ( ( uint32_t ) F_CPU )          // This F_CPU variable set by the environment
#define configTICK_TYPE_WIDTH_IN_BITS       TICK_TYPE_WIDTH_16_BITS

#define configMAX_PRIORITIES                4
#define configMAX_TASK_NAME_LEN             8 //16


#define configSTACK_DEPTH_TYPE              uint16_t

#define configMINIMAL_STACK_SIZE            80 //192
#define configCHECK_FOR_STACK_OVERFLOW      1
#define configUSE_TRACE_FACILITY            0

#define configUSE_MUTEXES                   0//1
#define configUSE_RECURSIVE_MUTEXES         0//1
#define configUSE_COUNTING_SEMAPHORES       0//1
#define configUSE_TIME_SLICING              0//1

#define configUSE_QUEUE_SETS                0
#define configUSE_APPLICATION_TASK_TAG      0
#define configUSE_MALLOC_FAILED_HOOK        1
#define configQUEUE_REGISTRY_SIZE           0
#define configSUPPORT_DYNAMIC_ALLOCATION    1
#define configSUPPORT_STATIC_ALLOCATION     0

#define configUSE_IDLE_HOOK                 1
#define configIDLE_SHOULD_YIELD             1
#define configUSE_TICK_HOOK                 0

///* Timer definitions. 
#define configUSE_TIMERS                    1
#define configTIMER_TASK_PRIORITY           ( configMAX_PRIORITIES )
#define configTIMER_TASK_STACK_DEPTH        80
#define configTIMER_QUEUE_LENGTH            5//10

#define INCLUDE_vTaskPrioritySet               0
#define INCLUDE_uxTaskPriorityGet              0
#define INCLUDE_vTaskDelete                    0
#define INCLUDE_vTaskSuspend                   0
#define INCLUDE_xTaskResumeFromISR             0
#define INCLUDE_xTaskDelayUntil                1   // needed for precise timing
#define INCLUDE_vTaskDelay                     1   // standard delay
#define INCLUDE_xTaskGetSchedulerState         0
#define INCLUDE_xTaskGetCurrentTaskHandle      1
#define INCLUDE_uxTaskGetStackHighWaterMark    0   // enable during development
#define INCLUDE_xTaskGetIdleTaskHandle         0
#define INCLUDE_eTaskGetState                  0
#define INCLUDE_xEventGroupSetBitFromISR       1   // only if using event groups
#define INCLUDE_xTimerPendFunctionCall         0
#define INCLUDE_xTaskAbortDelay                0
#define INCLUDE_xTaskGetHandle                 0

*/
