/**************************************************************************************************************/
/********************** Event-driven Active Object (AO) pattern with FreeRTOS *********************************/
/**************************************************************************************************************/
/*************************************************************************************************************/
/*                     Name    : Indoor Garden V66                                                           */
/*                     Author  : Stefan Zakutansky ATU.ie student                                            */
/*                     Date    : 30. 11.2024                                                                 */
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
//#include <NeoSWSerial.h>
//#include <AltSoftSerial.h>
#include <SoftwareSerial.h>
#include <Adafruit_ST7735.h>
//#include <PinChangeInt.h>
#include <PinChangeInterrupt.h>

//#define PCINT_USE_PORT2 false
// Constants defining Rotary encoder pins
#define PIN_A       5 //needs to be changed to hardware Interrupt pins
#define PIN_B        6
#define ENCODER_SW      4

// Constants defining SPI connection to TFT screen
#define TFT_CS   10
#define TFT_RST  12
#define TFT_DC    14
#define TFT_SCLK   13
#define TFT_MOSI   11

// Creating TFT screen object
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST); 

// Software serial object needed for RS485 communication

//AltSoftSerial is limited to specific pins:

// TX: Pin 9
// RX: Pin 8
SoftwareSerial mod(9, 8); // (RX - A4 , TX - A5)
// Creating Rotary encoder button object
//ClickEncoder *encoder;

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
TimerHandle_t sensorTimer, pumpTimer;

// Event Queues Handles
QueueHandle_t  sensorQueue, displayQueue, pumpQueue;

// Flag from interrupt routine (moved=true)
// bool rotaryEncoder = false;  

TaskHandle_t xEncoderHandle = NULL;

// Shared variable for adjust mode
volatile bool adjustModeFlag = false;
// the setup function runs once when you press reset or power the board
void setup() {
  // Initialize software serial communication at 4800 baud rate
  mod.begin(4800);    
   // Configure the button pin as input with pull-up resistor
  pinMode(ENCODER_SW, INPUT_PULLUP);
  pinMode(PIN_A, INPUT);
  pinMode(PIN_B, INPUT);

  // Attach the interrupt to the rotary encoder button pin
  attachPCINT(digitalPinToPCINT(ENCODER_SW), rotaryPushButton_ISR, FALLING);
  // We need to monitor both pins, rising and falling for all states
  attachInterrupt(digitalPinToInterrupt(PIN_A), rotary_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B), rotary_ISR, CHANGE);
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
  displayQueue = xQueueCreate(8, sizeof(Event));
  pumpQueue = xQueueCreate(3, sizeof(Event));

  // Single shot Timer - this timer is called by Pump Task, passing a time period calculated by PID controller
  pumpTimer = xTimerCreate("PumpTimer", pdMS_TO_TICKS(1), pdFALSE, (void *)1, pumpTimerCallback1);
  
  // Recursive Timer - triggering the event every 5 minutes
  sensorTimer = xTimerCreate("sensortimer", pdMS_TO_TICKS(13000), pdTRUE, (void *)1, sensorTimerCallback);  

 

  // Initialize Encoder Object
  //encoder = new ClickEncoder(ENCODER_CLK, ENCODER_DT, ENCODER_SW, 1, false); // (uint8_t A, uint8_t B, uint8_t BTN = -1, uint8_t stepsPerNotch = 1, bool active = LOW);

  // Encoder Input Event: EncoderAO reads the encoder’s value and sends the target setpoint updates to PumpControlAO.
  xTaskCreate(
    Enc_AO
  ,  "Check encoder"  // A name just for humans
  ,  50     // This stack size 
  ,  NULL   // Parameters for the task
  ,  3        // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
  ,  &xEncoderHandle   // Task Handle
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
  
  
  vTaskStartScheduler();
}

uint8_t checkRotaryEncoder(){
    // Reset the flag that brought us here (from ISR)
    //rotaryEncoder = false;
        

    static uint8_t lrmem = 3;
    static int lrsum = 0;
    static int8_t TRANS[] = {0, -1, 1, 14, 1, 0, 14, -1, -1, 14, 0, 1, 14, 1, -1, 0};

    // Read BOTH pin states to deterimine validity of rotation (ie not just switch bounce)
    int8_t l = digitalRead(PIN_A);
    int8_t r = digitalRead(PIN_B);

    // Move previous value 2 bits to the left and add in our new values
    lrmem = ((lrmem & 0x03) << 2) + 2 * l + r;

    // Convert the bit pattern to a movement indicator (14 = impossible, ie switch bounce)
    lrsum += TRANS[lrmem];

    /* encoder not in the neutral (detent) state */
    if (lrsum % 4 != 0){
      return 0;
    }

    /* encoder in the neutral state - clockwise rotation*/
    if (lrsum == 4){
      lrsum = 0;
      return 1;
      
    }

    /* encoder in the neutral state - anti-clockwise rotation*/
    if (lrsum == -4){
      lrsum = 0;
      return -1;
      //encoderData.setpoint -= 1 * 5;
    }

    // An impossible rotation has been detected - ignore the movement
    lrsum = 0;
    return 0;
    
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
  PORTD |= (1 << 7);  
}

// Timer Callback function sends event every 5 minutes
void sensorTimerCallback(TimerHandle_t xTimer) {
  Event encoderEvent = {EVENT_ENCODER_UPDATE, &encoderData};
  // send Even every 100 milliseconds (readinc encoder is 10x faster, no need to overload the event stream)
  
  xQueueSend(pumpQueue, &encoderEvent,(TickType_t) 0);
  // Create sensor event and asigne the data from sensor to it
  Event sensorEvent = {EVENT_SENSOR_UPDATE, &sensorData};
  // Send sensor event
  xQueueSend(sensorQueue, &sensorEvent,(TickType_t) 0);
}



void rotary_ISR(){

   BaseType_t xHigherPriorityTaskWoken = pdFALSE;  

    /* Notify the task that the transmission is complete. */  
  //vTaskNotifyGiveIndexedFromISR( xTaskToNotify, 0, &xHigherPriorityTaskWoken );  
  xTaskNotifyFromISR( xEncoderHandle,0,eSetBits,&xHigherPriorityTaskWoken);

// Yield to a higher priority task if necessary
  portYIELD_FROM_ISR();
}

// ISR for the rotary encoder button press
void rotaryPushButton_ISR() {
    /* We have not woken a task at the start of the ISR. */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;    

    encoderData.adjustMode = !encoderData.adjustMode;
    /* The transmission ended as expected. */ 
    Event encoderEvent = {EVENT_ENCODER_UPDATE, &encoderData};

    // Send the event to disply task Imedietly at 400ms rate
    xQueueSendFromISR(displayQueue, &encoderData,&xHigherPriorityTaskWoken );  

    // Yield to a higher priority task if necessary
    portYIELD_FROM_ISR();
}

// Polling helper function
void delayMs(uint32_t ms) {
  TickType_t start = xTaskGetTickCount();
  while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(ms)) {
    // Busy wait
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
    // have the 1 second timeout pre-calculated using RTOS timer tick 
    TickType_t timeout = xTaskGetTickCount() + pdMS_TO_TICKS(1000); 
    // Stay here until the buffer is filled up
    while (mod.available() < 10 && xTaskGetTickCount() < timeout) {
      delayMs(1);   //poll untoil software serial buffer is full of 9 bytes
    }
    // When buffer is already full, store the data to an array
    if (mod.available() >= 9) {
      for (byte i = 0; i < 10; i++) {
        delayMs(1);  //poll to give the serial port a chance to store byte values for each index
        soilSensorResponse[i] = mod.read();
      }
    mod.flush();    
    // Extract the moisturefrom array
    sensorData.moisture = int(soilSensorResponse[3] << 8 | soilSensorResponse[4]) / 10;

    // Extract the temperature from array
    int temperatureRaw = int(soilSensorResponse[5] << 8 | soilSensorResponse[6]);
    sensorData.temperature = (temperatureRaw > 0x7FFF) ? -(0x10000 - temperatureRaw) / 10 : temperatureRaw / 10;
    
    // limit the moisture to range 0 - 100
    if (sensorData.moisture > 100){ sensorData.moisture = 100;}  
    else if(sensorData.moisture < 0){ sensorData.moisture = 0;}

    // Create event
    Event sensorEvent = {EVENT_SENSOR_UPDATE, &sensorData};

    // Send the event to disply task Imedietly at 400ms rate
    xQueueSend(displayQueue, &sensorEvent,portMAX_DELAY);  
    // Second event is sent tu pump task by periodic timer every 5 min
    }
    else{
      //otherwise do nothing
    }  
    // 450 ms delay for smoother updates  
    vTaskDelay(pdMS_TO_TICKS(200)); 
  }
}   

// The task acquires data, Timer sends the Event
void Enc_AO(void *pvParameters) {
  (void*) pvParameters;

  //encoder->service();
  Event encoderEvent = {EVENT_ENCODER_UPDATE, &encoderData};
  // send Even every 100 milliseconds (readinc encoder is 10x faster, no need to overload the event stream)
  xQueueSend(displayQueue, &encoderEvent, portMAX_DELAY);  
  uint32_t ulNotificationValue;  
 
  while (1) {   
    /* Wait for the transmission to complete for ever. */  
  
    xTaskNotifyWaitIndexed( 0,         /* Wait for 0th notification. */
                            0x00,      /* Don't clear any notification bits on entry. */
                            ULONG_MAX, /* Reset the notification value to 0 on exit. */
                            &ulNotifiedValue, /* Notified value pass out in
                                                 ulNotifiedValue. */
                            portMAX_DELAY );  /* Block indefinitely. */

    /* Process any events that have been latched in the notified value. */
    if(  ( ulNotifiedValue & 0x01 ) != 0 && encoderData.adjustMode == 1)   {  
        /* Start the transmission by calling the function shown above. */  
        
        // Get the movement (if valid)
        uint8_t rotationValue = checkRotaryEncoder();

        // If valid movement, do something
        if (rotationValue != 0){
            encoderData.setpoint += rotationValue * 5;
            if (encoderData.setpoint > 100){ encoderData.setpoint = 100;}  
            else if(encoderData.setpoint < 0){ encoderData.setpoint = 0;} 

            Event encoderEvent = {EVENT_ENCODER_UPDATE, &encoderData};

            // Send the event to disply task Imedietly at 400ms rate
            xQueueSend(displayQueue, &encoderData,(TickType_t)0);  
            /* At this point xTaskToNotify should be NULL as no transmission is in  
            progress. A mutex can be used to guard access to the peripheral if  
            necessary. */
            configASSERT( xTaskToNotify == NULL );  

            /* Store the handle of the calling task. */  
            xTaskToNotify = xTaskGetCurrentTaskHandle();  
            
        }

    }  
    else {  
        /* The call to ulTaskNotifyTake() timed out. */  
    }  

    // 3 ms delay for responsivenes
    //vTaskDelay(pdMS_TO_TICKS(3)); 
    taskYIELD(); //Give CPU if no queue received
  }
}

// Task becomes active when Event arrives
void TFT_AO(void *pvParameters) {
  (void*) pvParameters;   
  while (1) {   
    // If queueing block contains update from encoder task, then print assigned data 
    if (xQueueReceive(displayQueue, &event, portMAX_DELAY) == pdTRUE) { 
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

// Task becomes active when Event arrives
void Pump_AO(void *pvParameters) {
  (void*) pvParameters;    
  // Set PB1 aka pin (9) as output, ignore the rest (for fast execution and memory saving, using C)
  DDRD |= 0x80; 		   // XXXXXXXX | 1000000 = XXXXXX1X 
  PORTD |= (1 << 7);   // Initialize pump to be off (HIGH means off for active low)
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
          PORTB &= ~(1 << 7); 
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

void loop(){ /*Empty. Things are done in Tasks*/ }

/*----------------------------------------------------------
ketch uses 23936 bytes (74%) of program storage space. Maximum is 32256 bytes.
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
#define configSUPPORT_STATIC_ALLOCATION     0

#define configUSE_IDLE_HOOK                 1
#define configIDLE_SHOULD_YIELD             1
#define configUSE_TICK_HOOK                 0

///* Timer definitions. 
#define configUSE_TIMERS                    1
#define configTIMER_TASK_PRIORITY           ( configMAX_PRIORITIES )
#define configTIMER_TASK_STACK_DEPTH        88
#define configTIMER_QUEUE_LENGTH            10

*/
