/**************************************************************************************************************/
/********************** Event-driven Active Object (AO) pattern with FreeRTOS *********************************/
/**************************************************************************************************************/

/*

Key Principles of the Active Object Pattern:
'''''''''''''''''''''''''''''''''''''''''''''
-Encapsulation: Each AO has its own data and state, only modified by the AO's methods.
-Event Handling and Queues: Events are delivered to AOs through queues. Each AO has its own queue, which decouples the tasks from direct data access.
-Asynchronous Execution: AOs execute asynchronously, receiving events in their own task context.

Explanation
''''''''''''
-Event Struct: Event holds the type and data, sent to each AO's queue.
-AO Tasks: SensorHandlerAO: Reads sensor data, sends events to displayQueue and pumpQueue.
-DisplayAO: Receives sensor data and updates the display.
-EncoderAO: Monitors encoder changes, sends new setpoints to pumpQueue.
-PumpControlAO: Adjusts pump runtime based on the setpoint and sensor data.
-Queue-driven: Each task operates independently, reacting to incoming events from its queue. This approach maximizes modularity and responsiveness by separating concerns.

Adjustment done in RTOSConfig.h file

 /*----------------------------------------------------------
#define configTICK_RATE_HZ   ( ( TickType_t ) 1000 )
#define configTOTAL_HEAP_SIZE  ( ( size_t ) ( 1280 ) )

//#define configUSE_TASK_NOTIFICATIONS 1
//#define configTASK_NOTIFICATION_ARRAY_ENTRIES 2

* Delay definition - here, the user can choose which delay implementation is required.
 * The default is to change nothing. *
#define configUSE_PORT_DELAY                1

/* And on to the things the same no matter the AVR type... *
#define configUSE_PREEMPTION                1

#define configCPU_CLOCK_HZ                  ( ( uint32_t ) F_CPU )          // This F_CPU variable set by the environment
#define configTICK_TYPE_WIDTH_IN_BITS       TICK_TYPE_WIDTH_16_BITS

#define configMAX_PRIORITIES                4
#define configMAX_TASK_NAME_LEN             8 //16

/* Set the stack depth type to be uint16_t, otherwise it defaults to StackType_t *
#define configSTACK_DEPTH_TYPE              uint16_t

#define configMINIMAL_STACK_SIZE            68 //192
#define configCHECK_FOR_STACK_OVERFLOW      1
#define configUSE_TRACE_FACILITY            0

#define configUSE_MUTEXES                   0//1
#define configUSE_RECURSIVE_MUTEXES         0//1
#define configUSE_COUNTING_SEMAPHORES       0//1
#define configUSE_TIME_SLICING              1

#define configUSE_QUEUE_SETS                0
#define configUSE_APPLICATION_TASK_TAG      0
#define configUSE_MALLOC_FAILED_HOOK        1
#define configQUEUE_REGISTRY_SIZE           0
#define configSUPPORT_DYNAMIC_ALLOCATION    1
#define configSUPPORT_STATIC_ALLOCATION     0

#define configUSE_IDLE_HOOK                 1
#define configIDLE_SHOULD_YIELD             1
#define configUSE_TICK_HOOK                 0

/* Timer definitions. *
#define configUSE_TIMERS                    1
#define configTIMER_TASK_PRIORITY           ( configMAX_PRIORITIES-1 )
#define configTIMER_TASK_STACK_DEPTH        68
#define configTIMER_QUEUE_LENGTH            10

 Current Compiled version at:
Sketch uses 24258 bytes (75%) of program storage space. Maximum is 32256 bytes.
Global variables use 648 bytes (31%) of dynamic memory, leaving 1400 bytes for local variables. Maximum is 2048 bytes.

*/

#include <Arduino_FreeRTOS.h>
#include <timers.h>
#include <queue.h>
#include <SoftwareSerial.h>
#include <Adafruit_ST7735.h>
#include <ClickEncoder.h>

// Rotary encoder pins
#define ENCODER_CLK 3
#define ENCODER_DT  4
#define ENCODER_SW  5 

//Rotary encoder variables
ClickEncoder *encoder;

#define TFT_CS  6
#define TFT_RST 11  
#define TFT_DC  10
#define TFT_SCLK 7
#define TFT_MOSI 8


Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST); 


SoftwareSerial mod(19, 18); // Software serial for RS485 communication

// Active Object Events
typedef enum { 
    EVENT_SENSOR_UPDATE,
    EVENT_ENCODER_UPDATE,
    EVENT_PUMP_CONTROL,
    EVENT_DISPLAY_UPDATE 
} EventType;

struct Event {
    EventType type;
    void  *data;
}; Event event;

struct EncoderData {
  bool adjustMode = false;
  int setpoint = 60;
};EncoderData encoderData;

struct SensorData {
  int moisture;
  int temperature;
};SensorData sensorData;

// define Tasks - Active Object Prototypes
void TFT_AO ( void *pvParameters );
void Enc_AO ( void *pvParameters );
void Sens_AO( void *pvParameters );
void Pump_AO( void *pvParameters );

// Declare Timer Handles
TimerHandle_t pumpTimer;

// Event Queues  
QueueHandle_t  sensorQueue, displayQueue, encoderQueue, pumpQueue;

   
// the setup function runs once when you press reset or power the board
void setup() {
  // Initialize software serial communication at 4800 baud rate
  mod.begin(4800);    
 
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
  tft.setCursor(15, 40);
  tft.print(F("Target"));
  tft.fillRect(0,56,160,2,ST7735_GREEN);  //draw a line
  tft.setCursor(15, 70);
  tft.print(F("Value     "));  
  
  // Create event queues
  sensorQueue = xQueueCreate(5, sizeof(SensorData));
  displayQueue = xQueueCreate(5, sizeof(Event));
  encoderQueue = xQueueCreate(5, sizeof(Event));
  pumpQueue = xQueueCreate(20, sizeof(Event));

  //Create timers for pump 1
  pumpTimer = xTimerCreate("PumpTimer", pdMS_TO_TICKS(1), pdFALSE, (void *)1, pumpTimerCallback1);
 
  encoder = new ClickEncoder(ENCODER_CLK, ENCODER_DT, ENCODER_SW, 1, false); // (uint8_t A, uint8_t B, uint8_t BTN = -1, uint8_t stepsPerNotch = 1, bool active = LOW);
  //priority 3 same as timer

  //Encoder Input Event: EncoderAO reads the encoder’s value and sends the target setpoint updates to PumpControlAO.
  xTaskCreate(
    Enc_AO
  ,  "Check encoder"  // A name just for humans
  ,  68 // This stack size 
  ,  NULL //Parameters for the task
  ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
  ,  NULL//&xTaskEnc 
  ); //Task Handle

  //Display Event: DisplayAO updates the display based on new sensor data.
  xTaskCreate(
    TFT_AO
    ,  "TFT Screen"  // A name just for humans
    ,  148 // This stack size 
    ,  NULL //Parameters for the task
    ,  3 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL//&xTaskTFT 
    ); //Task Handle

  //Sensor Read and Process Event Flow: The SensorHandlerAO will read sensor data and send events to DisplayAO and PumpControlAO.
  xTaskCreate(
    Sens_AO
    ,  "Sens the soil moisture"  // A name just for humans
    ,  108  // This stack size 
    ,  NULL //Parameters for the task
    ,  3 //1  // Priority, with 3 being the highest, and 0 being the lowest.
    ,  NULL//N&xTaskSens  // Task handle 
    ); //Task Handle

  //Pump Control Event: PumpControlAO adjusts the pump operation based on the setpoint and sensor feedback.
  xTaskCreate(
    Pump_AO
    ,  "TaskreadSoilMoireONE"  // A name just for humans
    ,  98  // This stack size 
    ,  NULL //Parameters for the task
    ,  3 //1  // Priority, with 3 being the highest, and 0 being the lowest.
    ,  NULL//&xTaskPump  // Task handle 
    ); //Task Handle

  vTaskStartScheduler();
}

//Proportional Integral derivate Calculation for time to run the pump
float computePID(int input,  int setpoint){ 
  // Initialize your PID controller, menu, etc.
  // You can also set these values here or keep them in the struct constructor
  static const float kp = 1.97;
  static const float ki = 0.80;
  static const float kd = 1.18;
  static int lastError;
  static float integral;
  static TickType_t lastTimeChecked = xTaskGetTickCount(); // Initialize with current tick count 
  TickType_t currentTime = xTaskGetTickCount();  
  float timeChange = (float)(currentTime - lastTimeChecked) / 1000; // Convert to seconds

  // Calculate the error
  int error = setpoint - input;

  // Proportional term
  float pTerm = kp * error;

  // Integral term
  integral += error * timeChange;
  //if (pidState->integral > 1000) pidState->integral = 1000;  // limit the Integral to prevent creeping up
  //else if (pidState->integral < -1000) pidState->integral = -1000;
  float iTerm = ki * integral;

  // Derivative term
  float derivative = (error - lastError) / timeChange;
  float dTerm = kd * derivative;

  // Combine all terms
  float output = pTerm + iTerm + dTerm;

  // Save error and time for next calculation
  lastError = error;
  lastTimeChecked = currentTime;

  return output;  // time for pump to run
}

// Callback function for Pump 1
void pumpTimerCallback1(TimerHandle_t xTimer) {
  PORTB |= (1 << 1);
}


void delayMs(uint32_t ms) {
  TickType_t start = xTaskGetTickCount();
  while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(ms)) {
    // Busy wait
  }
}


/*----------------------------------------------------------------------------------------------------------*/
/*-------------------------------Event-driven Active Object (AO) -------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------*/
void Sens_AO(void *pvParameters) {
  (void*) pvParameters;
  //TickType_t xLastWakeTime = xTaskGetTickCount(); // 1 second timeout
  const byte soilSensorRequest[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B};
  byte soilSensorResponse[9];
  while (1) {         
    // Read sensor data
    //poll is used instead of vTaskdelay() - no need to give CPU others, otherwise transmition will be broken
    //send request to RS485 device
    mod.write(soilSensorRequest, sizeof(soilSensorRequest));
    //poll untoil software serial buffer is full of 9 bytes
    TickType_t timeout = xTaskGetTickCount() + pdMS_TO_TICKS(1000); // 1 second timeout 
    while (mod.available() < 9 && xTaskGetTickCount() < timeout) {
      delayMs(1);   //poll untoil software serial buffer is full of 9 bytes
    }
    //when full of 9 bytes
    
    byte index = 0;
    if (mod.available() >= 9) {
      
      //Extract elements from string to individual indexex
      while (mod.available() && index < 9) {
        soilSensorResponse[index++] = mod.read();
        delayMs(2);  //poll to give the serial port a chance to store byte values in each index
      }
      // Extract the moisture and temperature values
      sensorData.moisture = int(soilSensorResponse[3] << 8 | soilSensorResponse[4]) / 10;
      int temperatureRaw = int(soilSensorResponse[5] << 8 | soilSensorResponse[6]);
      sensorData.temperature = (temperatureRaw > 0x7FFF) ? -(0x10000 - temperatureRaw) / 10 : temperatureRaw / 10;
    }

    // Create sensor event
    Event sensorEvent;
    sensorEvent.type = EVENT_SENSOR_UPDATE;
    sensorEvent.data = &sensorData;

    // Send event to display and pump control
    xQueueSend(displayQueue, &sensorEvent,(TickType_t) 0);   //send the values with the queue to disply task Imedietly
    xQueueSend(pumpQueue, &sensorEvent, (TickType_t) 0);
    vTaskDelay(pdMS_TO_TICKS(6000)); // 100 ms delay for smoother updates  
  }
}   

void Enc_AO(void *pvParameters) {
  (void*) pvParameters;
  encoder->setAccelerationEnabled(true);
  int value;
  int last = encoder->getValue();
  bool last_Button_eState = false;
  while (1) {   
    // Update encoder state
    encoder->service(); 
    // Check button press to toggle adjust mode 
    ClickEncoder::Button buttonState = encoder->getButton();
    if (buttonState == ClickEncoder::Clicked && !last_Button_eState ) {    
      encoderData.adjustMode = !encoderData.adjustMode; 
    }
    last_Button_eState = (buttonState == ClickEncoder::Clicked); 
    // Adjust PID.setpoint only when in adjust mode
    if (encoderData.adjustMode) {    
      value += encoder->getValue();  
      if (value != last ){
        if ( value > last) {    
          encoderData.setpoint -= 1;
        }  
        else if (value < last)  {
          encoderData.setpoint += 1; 
        }  
        last = value; 
      }        
    }
    Event encoderEvent;
    encoderEvent.type = EVENT_ENCODER_UPDATE;
    encoderEvent.data = &encoderData;
    xQueueSend(displayQueue, &encoderEvent, (TickType_t) 0);  
    xQueueSend(pumpQueue, &encoderEvent, (TickType_t) 0);        
    vTaskDelay(pdMS_TO_TICKS(10)); // 100 ms delay for smoother updates 
  }
}

void TFT_AO(void *pvParameters) {
  (void*) pvParameters;
  while (1) {   
    // do not interrupt SPI communication with TFT
    if (xQueueReceive(displayQueue, &event, portMAX_DELAY) == pdTRUE) { 
      if (event.type == EVENT_ENCODER_UPDATE ) {
        EncoderData  *data = (EncoderData*) event.data;
        taskENTER_CRITICAL(); // Disable interrupts here if needed
        tft.setTextSize(3);
        tft.setCursor(25, 94);
        if(data->adjustMode ){
          tft.setTextColor(ST7735_BLACK, ST7735_GREEN); 
          tft.print(">"); 
          tft.print(data->setpoint); 
          tft.print("<"); 
        }
        else{
          tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
          tft.print(" "); 
          tft.print(data->setpoint);
          tft.print(" "); 
        }
        tft.setTextColor(ST7735_GREEN, ST7735_BLACK);   
        tft.setTextSize(2);   
        taskEXIT_CRITICAL(); // Re-enable interrupts
      }
      else if (event.type == EVENT_SENSOR_UPDATE) {
        SensorData *data = (SensorData*) event.data;
        taskENTER_CRITICAL(); // Disable interrupts here if needed
        tft.setCursor(115, 0);
        tft.print(data->moisture);
        tft.setCursor(115, 20);
        tft.print(data->temperature);
        taskEXIT_CRITICAL(); // Re-enable interrupts
      }
    } 
        
    vTaskDelay(pdMS_TO_TICKS(80)); // 120 ms delay for smoother updates
  }
}



// Task function using Mutex
void Pump_AO(void *pvParameters) {
  (void*) pvParameters;
  DDRB |= 0x02; 		// XXXXXXXX | 00000010 = XX1XXXXX Set PB1 aka pin (9) as output, ignore the rest
  PORTB |= (1 << 1);   // Initialize pump to be off (HIGH means off for active low)
  //float pidOutput = 0;
  int newSetpoint = 60;
  while (1) {
    if (xQueueReceive(pumpQueue, &event, portMAX_DELAY) == pdTRUE) {
      if (event.type == EVENT_SENSOR_UPDATE) {
        SensorData *data = (SensorData*) event.data;
        float pidOutput = computePID(data->moisture,  newSetpoint);
        if (pidOutput > 0){               
          PORTB &= ~(1 << 1); 
          // Set timer duration to the PID runtime output
          xTimerChangePeriod(pumpTimer, pdMS_TO_TICKS(pidOutput), 0);  
          xTimerStart(pumpTimer, 0);  // Start the timer for pump 1
        }
      }
      else if (event.type == EVENT_ENCODER_UPDATE) {
        //pull the data from Encoder update event
        EncoderData  *data = (EncoderData*) event.data;
        newSetpoint = data->setpoint;
      }
    }
  vTaskDelay(pdMS_TO_TICKS(60)); // 100 ms delay for smoother updates
  }
}

void loop(){ /*Empty. Things are done in Tasks*/ }
