/**************************************************************************************************************/
/********************** Event-driven Active Object (AO) pattern with FreeRTOS *********************************/
/**************************************************************************************************************/
/*************************************************************************************************************/
/*                     Name    : Indoor Garden V82 static memory improved                                     */
/*                     Author  : Stefan Zakutansky ATU.ie student                                            */
/*                     Date    : 15. 4.2025                                                                 */
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


#define SENSOR_INTERVAL_MS 180000
#define QUEUE_SIZE 3

// Creating TFT screen object
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST); 

// Software serial object needed for RS485 communication
SoftwareSerial mod(9, 8); // (RX - A4 , TX - A5)

volatile int lastEncoded = 0;

struct PIDContext {
  float integral;
  int lastError;
  TickType_t lastTimeChecked;
};

// Active Object Events
typedef enum { 
  EVENT_SENSOR_UPDATE,
  EVENT_ENCODER_UPDATE,
} EventType;

// Event Struct
struct Event {
  EventType type;
  void* data;
}; 

// Rotary Encoder data
struct EncoderData {
  bool adjustMode = false;
  int setpoint = 72;
  int encoderValue = 0;
};
static EncoderData encoderData;

// Soil sensor data 
struct SensorData {
  int moisture;
  int temperature;
  int setpoint;
};
static SensorData sensorData;

// Static memory for queues
static Event sensorQueueStorage[QUEUE_SIZE];
static StaticQueue_t sensorQueueStruct;
static QueueHandle_t sensorQueue;

static Event displayQueueStorage[QUEUE_SIZE];
static StaticQueue_t displayQueueStruct;
static QueueHandle_t displayQueue;

// Tasks definitions
void TFT_AO(void *pvParameters);
void Enc_AO(void *pvParameters);
void Sens_AO(void *pvParameters);
void Pump_AO(void *pvParameters);

// Timer handles
static TimerHandle_t sensorTimer;
static StaticTimer_t sensorTimerBuffer;

// Timer callbacks
//void pumpTimerCallback1(TimerHandle_t xTimer);
void sensorTimerCallback(TimerHandle_t xTimer);

void setup() {
  // Initialize software serial communication at 4800 baud rate
  mod.begin(4800);      // RS485
  Serial.begin(9600);
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_DT), updateEncoder, CHANGE);
 
  // Initialize TFT
  tft.initR(INITR_BLACKTAB);    
  tft.fillScreen(ST7735_BLACK);  
  tft.setRotation(1);
  tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
  tft.setTextSize(2);

  // Static UI elements
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
  tft.fillRect(0,60,160,2,ST7735_GREEN);
  tft.setCursor(15, 70);
  tft.print(F("Value     "));  
  
  // Create static queues
  sensorQueue = xQueueCreateStatic(QUEUE_SIZE, sizeof(Event), 
                                 (uint8_t*)sensorQueueStorage, &sensorQueueStruct);
  displayQueue = xQueueCreateStatic(QUEUE_SIZE, sizeof(Event), 
                                  (uint8_t*)displayQueueStorage, &displayQueueStruct);
 
  sensorTimer = xTimerCreateStatic("sensortimer", pdMS_TO_TICKS(SENSOR_INTERVAL_MS), pdTRUE, 
                                 (void*)1, sensorTimerCallback, &sensorTimerBuffer);

  // Initial encoder event to display value on screen
  Event encoderEvent = {EVENT_ENCODER_UPDATE, &encoderData}; 
  xQueueSend(displayQueue, &encoderEvent, 0);

  // Create tasks
  xTaskCreate(Enc_AO, "Check encoder", 88, NULL, 3, NULL); 
  xTaskCreate(TFT_AO, "TFT Screen", 168, NULL, 3, NULL);
  xTaskCreate(Sens_AO, "Sens the soil moisture", 98, NULL, 3, NULL);
  xTaskCreate(Pump_AO, "Pump control", 108, NULL, 4, NULL);
  //checkMemory();
  xTimerStart(sensorTimer, 0);
  vTaskStartScheduler();
}

// ISR for encoder
void updateEncoder() {
  int MSB = digitalRead(ENCODER_CLK);
  int LSB = digitalRead(ENCODER_DT);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderData.encoderValue++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderData.encoderValue--;

  lastEncoded = encoded;
}

bool buttonPressed() {
  static TickType_t lastDebounceTime = 0;
  const TickType_t debounceDelay = pdMS_TO_TICKS(50);

  if (digitalRead(ENCODER_SW) == HIGH) {
    return false;
  }

  TickType_t now = xTaskGetTickCount();
  if ((now - lastDebounceTime) < debounceDelay) {
    return false;
  }

  lastDebounceTime = now;
  return true;
}

float computePID(PIDContext* ctx, int input, int setpoint) {
  //static const float kp = 1.30;
  //static const float ki = 0.15; 
  //static const float kd = 0.45;

  static const float kp = 2.0;  // Increased from 1.30
  static const float ki = 0.05; // Reduced from 0.15
  static const float kd = 0.25; // Reduced from 0.45
  
  TickType_t currentTime = xTaskGetTickCount();
  float timeChange = (float)(currentTime - ctx->lastTimeChecked) / configTICK_RATE_HZ;
  if(timeChange <= 0) timeChange = 0.001;
  
  int error = setpoint - input;
  
  // Reset integral when error is small
  //if(abs(error) < 1) {
  //    ctx->integral = 0;
  //}
  
  float pTerm = kp * error;
  ctx->integral += error * timeChange;
  //ctx->integral = constrain(ctx->integral, -1000.0, 1000.0);
  float iTerm = ki * ctx->integral;
  
  float derivative = (error - ctx->lastError) / timeChange;
  float dTerm = kd * derivative;
  
  float output = pTerm + iTerm + dTerm;
  
  ctx->lastError = error;
  ctx->lastTimeChecked = currentTime;
  
  return output;
}


// Trigger the pump task by timer overflow sendimg the message via queue
void sensorTimerCallback(TimerHandle_t xTimer) {
  sensorData.setpoint = encoderData.setpoint;
   //checkMemory();  // Before queue operation
  Event sensorEvent = {EVENT_SENSOR_UPDATE, &sensorData};
  xQueueSend(sensorQueue, &sensorEvent, 0);

}

void printPaddedNumber(int number) {
  taskENTER_CRITICAL();
  if (number < 10) {
    tft.print(F("  "));
  } else if (number < 100) {
    tft.print(F(" "));
  }
  tft.print(number);
  taskEXIT_CRITICAL();
}

void checkMemory() {
  extern int __heap_start, *__brkval;
  int free_memory = (__brkval == 0) ? 
    (int)&free_memory - (int)&__heap_start : 
    (int)&free_memory - (int)__brkval;
    
  Serial.print("[MEM] Free: ");
  Serial.println(free_memory);
  //Serial.print(" | Task: ");
  //Serial.println(pcTaskGetName(NULL)); // Current task name
}

void Sens_AO(void *pvParameters) {
  static const byte soilSensorRequest[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B};
  byte soilSensorResponse[9];
  uint8_t responseIndex = 0;
  TickType_t lastRequestTime = 0;
  
  
  while (1) {         

    // Send request every 2 seconds
    if ((xTaskGetTickCount() - lastRequestTime) >= pdMS_TO_TICKS(2000)) {
      //checkMemory();  
      mod.write(soilSensorRequest, sizeof(soilSensorRequest));
      responseIndex = 0;
      lastRequestTime = xTaskGetTickCount();
    }
    
    // Non-blocking read
    while (mod.available() > 0 && responseIndex < 9) {
      soilSensorResponse[responseIndex++] = mod.read();
    }

    if (responseIndex >= 9) {

      int moistureRaw = int(soilSensorResponse[3] << 8 | soilSensorResponse[4]) / 10;
      sensorData.moisture = constrain(moistureRaw, 0, 100);

      int temperatureRaw = int(soilSensorResponse[5] << 8 | soilSensorResponse[6]);
      sensorData.temperature = temperatureRaw / 10;
      //checkMemory();  // Before queue operation
      Event sensorEvent = {EVENT_SENSOR_UPDATE, &sensorData};
      xQueueSend(displayQueue, &sensorEvent, 0);
      responseIndex = 0;  // Reset for next reading
      //Serial.println("sent: ");
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}   


// Modify your Enc_AO task:
void Enc_AO(void *pvParameters) {
  bool lastAdjustMode = false;
  int lastValue = 0;
  
  while (1) {
    if (buttonPressed()) {
      encoderData.adjustMode = !encoderData.adjustMode;     
      Event encoderEvent = {EVENT_ENCODER_UPDATE, &encoderData}; 
      xQueueSend(displayQueue, &encoderEvent, 0);
      
    }

    if (encoderData.adjustMode) {
      int currentValue = encoderData.encoderValue;
      
      if (currentValue != lastValue) {
        encoderData.setpoint += (currentValue > lastValue) ? 1 : -1;
        encoderData.setpoint = constrain(encoderData.setpoint, 0, 100);
        lastValue = currentValue;

        // Reset encoder value periodically to prevent overflow
        if (abs(encoderData.encoderValue) > 1000) {
          encoderData.encoderValue = 0;
        }

        Event encoderEvent = {EVENT_ENCODER_UPDATE, &encoderData}; 
        xQueueSend(displayQueue, &encoderEvent, 0);

      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


void TFT_AO(void *pvParameters) {
  Event event;  
  while (1) {
    if (xQueueReceive(displayQueue, &event, portMAX_DELAY) == pdTRUE) {
      taskENTER_CRITICAL();

      if (event.type == EVENT_ENCODER_UPDATE) {
        EncoderData *data = static_cast<EncoderData*>(event.data);

        tft.setTextSize(3);
        tft.setCursor(25, 94);
        tft.setTextColor(data->adjustMode ? ST7735_BLACK : ST7735_GREEN,
                         data->adjustMode ? ST7735_GREEN : ST7735_BLACK);

        tft.print(data->adjustMode ? ">" : " ");
        printPaddedNumber(data->setpoint);
        tft.print(data->adjustMode ? "<" : " ");
      }
      else if (event.type == EVENT_SENSOR_UPDATE) {
        SensorData *data = static_cast<SensorData*>(event.data);

        tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
        tft.setTextSize(2);

        tft.setCursor(105, 0);
        printPaddedNumber(data->moisture);

        tft.setCursor(105, 20);
        printPaddedNumber(data->temperature);
      }
      taskEXIT_CRITICAL();
    }
  }
}

void Pump_AO(void *pvParameters) {
  static PIDContext ctx = {0};
  Event event;
  
  DDRD |= 0x80;
  PORTD |= (1 << 7); // Start with pump off
  
  while (1) {
    if (xQueueReceive(sensorQueue, &event, portMAX_DELAY) == pdTRUE) {
      SensorData *data = static_cast<SensorData*>(event.data);
      Serial.print("Moisture: ");
      Serial.println(data->moisture);
      Serial.print("Setpoint: ");
      Serial.println(data->setpoint);
      if (event.type == EVENT_SENSOR_UPDATE && 
         data->moisture < data->setpoint) {
        checkMemory();  
        float pidOutput = computePID(&ctx, data->moisture, 
                                   data->setpoint);

        TickType_t pumpTime = pdMS_TO_TICKS(pidOutput * 100); // Min 100ms                           
        //TickType_t pumpTime = pdMS_TO_TICKS(fmax(pidOutput * 100, 100)); // Min 100ms
        //pumpTime = constrain(pumpTime, MIN_PUMP_TIME, MAX_PUMP_TIME);
        Serial.print("Pump Time: ");
        Serial.println(pumpTime);
        PORTD &= ~(1 << 7); // Turn on
        vTaskDelay(pumpTime);  // Blocking delay (okay at high priority)
        PORTD |= (1 << 7);  // Turn off
      }
    }
  }
}

void loop() {}


/*----------------------------------------------------------
Sketch uses 25006 bytes (77%) of program storage space. Maximum is 32256 bytes.
Global variables use 1241 bytes (60%) of dynamic memory, leaving 807 bytes for local variables. Maximum is 2048 bytes.

Adjustment done in RTOSConfig.h file

#define configTICK_RATE_HZ   ( ( TickType_t ) 1000 )

#define configHEAP_CLEAR_MEMORY_ON_FREE     0


#define configUSE_PORT_DELAY                1


#define configUSE_PREEMPTION                1

#define configCPU_CLOCK_HZ                  ( ( uint32_t ) F_CPU )          // This F_CPU variable set by the environment
#define configTICK_TYPE_WIDTH_IN_BITS       TICK_TYPE_WIDTH_16_BITS

#define configMAX_PRIORITIES                4
#define configMAX_TASK_NAME_LEN             8 //16


#define configSTACK_DEPTH_TYPE              uint16_t

#define configMINIMAL_STACK_SIZE            88 //192
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
#define configSUPPORT_STATIC_ALLOCATION     1

#define configUSE_IDLE_HOOK                 1
#define configIDLE_SHOULD_YIELD             1
#define configUSE_TICK_HOOK                 0

///* Timer definitions. 
#define configUSE_TIMERS                    1
#define configTIMER_TASK_PRIORITY           ( configMAX_PRIORITIES )
#define configTIMER_TASK_STACK_DEPTH        88
#define configTIMER_QUEUE_LENGTH            5

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
