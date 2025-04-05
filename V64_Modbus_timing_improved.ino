/**************************************************************************************************************/
/********************** Event-driven Active Object (AO) pattern with FreeRTOS *********************************/
/**************************************************************************************************************/
/*************************************************************************************************************/
/*                     Name    : Indoor Garden V64                                                           */
/*                     Author  : Stefan Zakutansky ATU.ie student                                            */
/*                     Date    : 27. 11.2024                                                                 */
/*                     Notes   : The code architecture accomodates an Active object pattern faciliating      */
/*                               RTOS features, likes, Queues, Timers, Schedulling. The Event generator task */
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
#define ENCODER_CLK       5
#define ENCODER_DT        6
#define ENCODER_SW      4

// Constants defining SPI connection to TFT screen
#define TFT_CS   10
#define TFT_RST  12
#define TFT_DC    8
#define TFT_SCLK   13
#define TFT_MOSI   11

// Creating TFT screen object
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST); 

// Software serial object needed for RS485 communication
SoftwareSerial mod(18, 19); // (RX, TX)

// Creating Rotary encoder button object
ClickEncoder *encoder;

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
}; Event event;

// Rotary Encoder data
struct EncoderData {
  bool adjustMode = false;
  int setpoint = 80;
};EncoderData encoderData;

// Soil sensor data 
struct SensorData {
  int moisture;
  int temperature;
};SensorData sensorData;

// Tasks definitions - Active Object Prototypes
void TFT_AO ( void *pvParameters );
void Enc_AO ( void *pvParameters );
void Sens_AO( void *pvParameters );
void Pump_AO( void *pvParameters );

// Declare Timer Handles
TimerHandle_t sensorTimer, pumpTimer, encoderTimer;

// Event Queues Handles
QueueHandle_t  sensorQueue, displayQueue, pumpQueue;

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
  tft.setCursor(5, 40);
  tft.print(F("Target"));
  tft.fillRect(0,60,160,2,ST7735_GREEN);  //draw a line
  tft.setCursor(15, 70);
  tft.print(F("Value     "));  
  
  // Create event queues
  sensorQueue = xQueueCreate(3, sizeof(SensorData));
  displayQueue = xQueueCreate(5, sizeof(Event));
  pumpQueue = xQueueCreate(3, sizeof(Event));

  // Single shot Timer - this timer is called by Pump Task, passing a time period calculated by PID controller
  pumpTimer = xTimerCreate("PumpTimer", pdMS_TO_TICKS(1), pdFALSE, (void *)1, pumpTimerCallback1);
  
  // Recursive Timer - triggering the event every 5 minutes
  sensorTimer = xTimerCreate("sensortimer", pdMS_TO_TICKS(13000), pdTRUE, (void *)1, sensorTimerCallback);  

  // Recursive timer - triggering the events every 150 milliseconds worked
  encoderTimer = xTimerCreate("encoderTimer", pdMS_TO_TICKS(100), pdTRUE, (void *)1, encoderTimerCallback);  

  // Initialize Encoder Object
  encoder = new ClickEncoder(ENCODER_CLK, ENCODER_DT, ENCODER_SW, 1, false); // (uint8_t A, uint8_t B, uint8_t BTN = -1, uint8_t stepsPerNotch = 1, bool active = LOW);

  // Encoder Input Event: EncoderAO reads the encoder’s value and sends the target setpoint updates to PumpControlAO.
  xTaskCreate(
    Enc_AO
  ,  "Check encoder"  // A name just for humans
  ,  50     // This stack size 
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
    ,  80  // This stack size 
    ,  NULL //Parameters for the task
    ,  3    // Priority, with 3 being the highest, and 0 being the lowest.
    ,  NULL // Task handle 
    ); 
  
  xTimerStart(sensorTimer, 0);  // Start the timer imedietly
  xTimerStart(encoderTimer, 0);  // Start the timer imedietly
  
  vTaskStartScheduler();
}

//Proportional Integral derivate Calculation for time to run the pump
float computePID(int input,  int setpoint){ 
 
  // Tuned PID values
  static const float kp = 1.30; //"1.2" 23.11.24
  static const float ki = 0.15; 
  static const float kd = 0.45; 


  // Initialize your PID controller.
  static int lastError;
  static float integral;
  static TickType_t lastTimeChecked = xTaskGetTickCount(); // Initialize with current tick count 

  // record current system tick count
  TickType_t currentTime = xTaskGetTickCount();  
  float timeChange = (float)(currentTime - lastTimeChecked) / 1000; // Convert to seconds

  // Calculate the error
  int error = setpoint - input;

  // Proportional term
  float pTerm = kp * error;

  // Integral term
  integral += error * timeChange;

  // calculate iTerm  
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

// Timer Callback function for Pump to turn of, after the calculated period expires
void pumpTimerCallback1(TimerHandle_t xTimer) {
  // Turn off the pump by setting the bit high (active low)
  PORTB |= (1 << 1);  
}

// Timer Callback function sends event every 5 minutes
void sensorTimerCallback(TimerHandle_t xTimer) {
  // Create sensor event and asigne the data from sensor to it
  Event sensorEvent = {EVENT_SENSOR_UPDATE, &sensorData};
  // Send sensor event
  xQueueSend(sensorQueue, &sensorEvent,(TickType_t) 0);
}

// Timer Callback function sends event every 100 ms
void encoderTimerCallback(TimerHandle_t xTimer) {
  // Create the event - in other words pickup variables from structure and store them in Event structure
  Event encoderEvent = {EVENT_ENCODER_UPDATE, &encoderData};
  // send Even every 100 milliseconds (readinc encoder is 10x faster, no need to overload the event stream)
  xQueueSend(displayQueue, &encoderEvent, (TickType_t) 0);  

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
    delayMs(5);
    //vTaskDelay(10);
    // have the 1 second timeout pre-calculated using RTOS timer tick 
    //
    // Stay here until the buffer is filled up
      // Poll for data, but exit if timeout (1 sec)
        TickType_t startTime = xTaskGetTickCount();
       while (mod.available() < 9) {
            if (xTaskGetTickCount() - startTime > pdMS_TO_TICKS(1000)) {
                break; // Timeout reached, abort read
            }
            delayMs(1); // Keep task execution atomic
        }
      /*
    TickType_t timeout = xTaskGetTickCount() + pdMS_TO_TICKS(1000); 
    while (mod.available() < 10 && xTaskGetTickCount() < timeout) {
      delayMs(1);   //poll untoil software serial buffer is full of 9 bytes
    }
    // When buffer is already full, store the data to an array
      */
    if (mod.available() >= 9) {
          for (byte i = 0; i < 9; i++) {
            delayMs(1);
            //delayMs(1);  //poll to give the serial port a chance to store byte values for each index
            soilSensorResponse[i] = mod.read();
          }
      
        // Extract the moisturefrom array
        sensorData.moisture = int(soilSensorResponse[3] << 8 | soilSensorResponse[4]) / 10;
        // limit the moisture to range 0 - 100
        if (sensorData.moisture > 100){ sensorData.moisture = 100;}  
        else if(sensorData.moisture < 0){ sensorData.moisture = 0;}
    
        // Extract the temperature from array
        int temperatureRaw = int(soilSensorResponse[5] << 8 | soilSensorResponse[6]);
        sensorData.temperature = (temperatureRaw > 0x7FFF) ? -(0x10000 - temperatureRaw) / 10 : temperatureRaw / 10;
    
        // Create event
        Event sensorEvent = {EVENT_SENSOR_UPDATE, &sensorData};
    
        // Send the event to disply task Imedietly at 400ms rate
        xQueueSend(displayQueue, &sensorEvent,0);  
        // Second event is sent tu pump task by periodic timer every 5 min
    }
    else{

    }

    // 450 ms delay for smoother updates  
    vTaskDelay(pdMS_TO_TICKS(400)); 
  }
}   

// The task acquires data, Timer sends the Event
void Enc_AO(void *pvParameters) {
  (void*) pvParameters;
  encoder->setAccelerationEnabled(true); 
  //encoder->service();
  int value = encoder->getValue();
  static int last  = encoder->getValue(); 
  while (1) {   
    // Update encoder state
    encoder->service();  
    // Check button press to toggle adjust mode 
    if (encoderData.adjustMode) {   
       //taskENTER_CRITICAL(); // Disable interrupts here if needed
      value += encoder->getValue();
      if (value != last ){   
        
        if ( value > last) { 
          encoderData.setpoint--;   
        }  
        else if (value < last)  {
          encoderData.setpoint++; 
        }
        // limit the setpoint to range 0 - 100  
        if (encoderData.setpoint > 100){ encoderData.setpoint = 100;}  
        else if(encoderData.setpoint < 0){ encoderData.setpoint = 0;}
          
        last = value; 
        // send message only when value has changed  
        Event encoderEvent = {EVENT_ENCODER_UPDATE, &encoderData};
        xQueueSend(pumpQueue, &encoderEvent,(TickType_t) 0);
        // encoder timer triggers posting the event to display and Pump tasks every 100ms
        // to prevent saturation of queue at 3ms rate
      } 
       
      //taskEXIT_CRITICAL(); // Re-enable interrupts   
    }
     ClickEncoder::Button buttonState = encoder->getButton();
    if (buttonState == ClickEncoder::Clicked ) {    
      encoderData.adjustMode = !encoderData.adjustMode;        
    }      
    // 3 ms delay for responsivenes
    vTaskDelay(pdMS_TO_TICKS(5)); 
  }
}

// Task becomes active when Event arrives
void TFT_AO(void *pvParameters) {
  (void*) pvParameters;   
  while (1) {   
    // If queueing block contains update from encoder task, then print assigned data 
    if (xQueueReceive(displayQueue, &event, 0) == pdTRUE) { 
      configASSERT(event != (Event)0);
      if (event.type == EVENT_ENCODER_UPDATE ) {
        EncoderData  *data = (EncoderData*) event.data;
        tft.setTextSize(3);
        tft.setCursor(25, 94);
        if(data->adjustMode){
          // do not interrupt SPI communication with TFT
          tft.setTextColor(ST7735_BLACK, ST7735_GREEN); 
          taskENTER_CRITICAL(); // Disable interrupts here if needed
          tft.print(F(">")); 
          if (data->setpoint < 10) {
              tft.print(F("  "));  // Add two spaces for single-digit numbers
          } else if (data->setpoint < 100) {
              tft.print(F(" "));   // Add one space for two-digit numbers
          }
          tft.print(data->setpoint);       // Print the number itself
          tft.print(F("<")); 
          taskEXIT_CRITICAL(); // Re-enable interrupts
        }
        else{
          tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
          // do not interrupt SPI communication while printing new data
          taskENTER_CRITICAL(); // Disable interrupts here if needed
          tft.print(F(" ")); 
          if (data->setpoint < 10) {
              tft.print(F("  "));  // Add two spaces for single-digit numbers
          } else if (data->setpoint < 100) {
              tft.print(F(" "));   // Add one space for two-digit numbers
          }
          tft.print(data->setpoint);       // Print the number itself
          tft.print(F(" "));          
          taskEXIT_CRITICAL(); // Re-enable interrupts
        }   
      }
      // If queueing block contains update from sensor, then print assigned data 
      else if (event.type == EVENT_SENSOR_UPDATE) {
        SensorData *data = (SensorData*) event.data;  
        // do not interrupt SPI communication with TFT
        tft.setTextColor(ST7735_GREEN, ST7735_BLACK);   
        tft.setTextSize(2);
        tft.setCursor(105, 0);
        taskENTER_CRITICAL(); // Disable interrupts here if needed
        if (data->moisture < 10) {
            tft.print(F("  "));  // Add two spaces for single-digit numbers
        } else if (data->moisture < 100) {
            tft.print(F(" "));   // Add one space for two-digit numbers
        }
        tft.print(data->moisture);       // Print the number itself
        //delayMs(2);
        tft.setCursor(105, 20);
        //taskENTER_CRITICAL(); // Disable interrupts here if needed
        if (data->temperature < 10) {
            tft.print(F("  "));  // Add two spaces for single-digit numbers
        } else if (data->temperature < 100) {
            tft.print(F(" "));   // Add one space for two-digit numbers
        }
        tft.print(data->temperature);       // Print the number itself
        taskEXIT_CRITICAL(); // Re-enable interrupts
      }
    } 
    taskYIELD(); //Give CPU if no queue received
  }
}
// Task becomes active when an event arrives
void Pump_AO(void *pvParameters) {
    (void*) pvParameters;    

    // Set PB1 (pin 9) as output, active LOW
    DDRB |= (1 << 1);  // Set pin 9 as output
    PORTB |= (1 << 1); // Initialize pump to be OFF (HIGH means off)

    int newSetpoint = 0;  // Default setpoint
    SensorData latestSensorData = {0}; // Store latest sensor values

    Event event;

    while (1) {
  // Process any pending events from both queues
        if (xQueueReceive(sensorQueue, &event, 0) == pdTRUE || xQueueReceive(pumpQueue, &event, 0) == pdTRUE) {
            
            switch (event.type) {
                case EVENT_SENSOR_UPDATE:
                    // Update sensor data and compute PID
                    latestSensorData = *(SensorData*) event.data;
                    float pidOutput = computePID(latestSensorData.moisture, newSetpoint);

                    if (pidOutput > 0) {  
                        PORTB &= ~(1 << 1); // Turn pump ON
                        xTimerChangePeriod(pumpTimer, pdMS_TO_TICKS(pidOutput), 0);  
                        xTimerStart(pumpTimer, 0);  
                    }
                    break;

                case EVENT_ENCODER_UPDATE:
                    // Update setpoint from encoder
                    newSetpoint = ((EncoderData*) event.data)->setpoint;
                    // Immediately recompute PID using latest sensor reading
                    float newPidOutput = computePID(latestSensorData.moisture, newSetpoint);

                    if (newPidOutput > 0) {  
                        PORTB &= ~(1 << 1); // Turn pump ON
                        xTimerChangePeriod(pumpTimer, pdMS_TO_TICKS(newPidOutput), 0);  
                        xTimerStart(pumpTimer, 0);  
                    }
                    break;

                default:
                    // Handle unexpected events (optional debugging)
                    break;
            }
        }

        // Yield for FreeRTOS multitasking
        taskYIELD();
    }
}

/*
// Task becomes active when Event arrives
void Pump_AO(void *pvParameters) {
  (void*) pvParameters;    
  // Set PB1 aka pin (9) as output, ignore the rest (for fast execution and memory saving, using C)
  DDRB |= 0x02; 		   // XXXXXXXX | 00000010 = XXXXXX1X 
  PORTB |= (1 << 1);   // Initialize pump to be off (HIGH means off for active low)
  int newSetpoint = encoderData.setpoint;
  while (1) {
    if (xQueueReceive(sensorQueue, &event, portMAX_DELAY) == pdTRUE) {
      // Check if incoming event is valid 
      configASSERT(event != (Event)0);
      // Once event received check the type
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
    }  
    if (xQueueReceive(pumpQueue, &event, portMAX_DELAY) == pdTRUE) {
      // Check if incoming data is valid 
      configASSERT(event != (Event)0);
      if (event.type == EVENT_ENCODER_UPDATE) {
        //pull the data from Encoder update event
        EncoderData  *data = (EncoderData*) event.data;
        // store new value to task local variable, so other Queue can use it
        newSetpoint = data->setpoint;
      }
    }
    taskYIELD();
  }
}
*/
void loop(){ /*Empty. Things are done in Tasks*/ }

/*----------------------------------------------------------
Sketch uses 24740 bytes (76%) of program storage space. Maximum is 32256 bytes.
Global variables use 684 bytes (33%) of dynamic memory, leaving 1364 bytes for local variables. Maximum is 2048 bytes.

Adjustment done in RTOSConfig.h file

#define configTICK_RATE_HZ   ( ( TickType_t ) 1000 )

#define configHEAP_CLEAR_MEMORY_ON_FREE        1


#define configUSE_PORT_DELAY                1


#define configUSE_PREEMPTION                0/1

#define configCPU_CLOCK_HZ                  ( ( uint32_t ) F_CPU )          // This F_CPU variable set by the environment
#define configTICK_TYPE_WIDTH_IN_BITS       TICK_TYPE_WIDTH_16_BITS

#define configMAX_PRIORITIES                4
#define configMAX_TASK_NAME_LEN             8 //16


#define configSTACK_DEPTH_TYPE              uint16_t

#define configMINIMAL_STACK_SIZE            98 //192
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
#define configIDLE_SHOULD_YIELD             0//1
#define configUSE_TICK_HOOK                 0

///* Timer definitions. 
#define configUSE_TIMERS                    1
#define configTIMER_TASK_PRIORITY           ( configMAX_PRIORITIES )
#define configTIMER_TASK_STACK_DEPTH        98
#define configTIMER_QUEUE_LENGTH            5//10

*/
