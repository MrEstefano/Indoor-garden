/**************************************************************************************************************/
/********************** Event-driven Active Object (AO) pattern with FreeRTOS *********************************/
/**************************************************************************************************************/
/*************************************************************************************************************/
/*                     Name    : Indoor Garden V65                                                   */
/*                     Author  : Stefan Zakutansky ATU.ie student                                            */
/*                     Date    : 5. 11.2024                                                                */
/*                     Notes   : The code operates on Active object architecture to faciliating              */
/*                               Queues feature of RTOS. The system measures a soil moisture RS485           */
/*                               sensor. The values are computed by PID alghoritm to calculate               */
/*                               a pump runtime acordingly. The sampling rate of 4 minutes provides          */
/*                               favorable soil condition to mantain healthy growth of the plant.            */
/*                               The pump is active LOW. The Countdown for each task currently running is    */
/*                               displayed, same as soil moisture, setpoint and total pump runtime           */
/*                               per day on a TFT display. User can intercat with encoder push button,       */
/*                               in order to orientate in short menu to adjust the treashold value and       */
/*                               dayly water limit  and PID cooeficients                                     */
/*************************************************************************************************************/
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

*/

#include <Arduino_FreeRTOS.h>
#include <timers.h>
#include <task.h>
#include <portable.h>
#include <queue.h>
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
    EVENT_PUMP_CONTROL,
    EVENT_DISPLAY_UPDATE 
} EventType;

// Active Object event and data carrier
struct Event {
    EventType type;
    void  *data;
}; Event event;

// Rotary Encoder Input data storage
struct EncoderData {
  bool adjustMode = false;
  int setpoint = 60;
};EncoderData encoderData;

// Soil sensor Input data storage
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

// Event Queues sensorQueue, encoderQueue, 
QueueHandle_t    sensorQueue, displayQueue, pumpQueue;

   
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
  sensorQueue = xQueueCreate(10, sizeof(SensorData));
  displayQueue = xQueueCreate(16, sizeof(Event));
  //encoderQueue = xQueueCreate(15, sizeof(Event));
  pumpQueue = xQueueCreate(10, sizeof(Event));

  // Single shot Timer - this timer is called by function, passing variable time interval
  pumpTimer = xTimerCreate("PumpTimer", pdMS_TO_TICKS(1), pdFALSE, (void *)1, pumpTimerCallback1);
  
  // Repetative Timer- the 1000 will be overwritten after first call
  sensorTimer = xTimerCreate("sensortimer", pdMS_TO_TICKS(15000), pdTRUE, (void *)1, sensorTimerCallback);  

  encoderTimer = xTimerCreate("encoderTimer", pdMS_TO_TICKS(160), pdTRUE, (void *)1, encoderTimerCallback);  
  // Initialize Encoder Object
  encoder = new ClickEncoder(ENCODER_CLK, ENCODER_DT, ENCODER_SW, 1, false); // (uint8_t A, uint8_t B, uint8_t BTN = -1, uint8_t stepsPerNotch = 1, bool active = LOW);

  //Encoder Input Event: EncoderAO reads the encoder’s value and sends the target setpoint updates to PumpControlAO.
  xTaskCreate(
    Enc_AO
  ,  "Check encoder"  // A name just for humans
  ,  68 // This stack size 
  ,  NULL //Parameters for the task
  ,  3//1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
  ,  NULL//&xTaskEnc 
  ); //Task Handle

  //Display Event: DisplayAO updates the display based on new sensor data.
  xTaskCreate(
    TFT_AO
    ,  "TFT Screen"  // A name just for humans
    ,  148 // This stack size 
    ,  NULL //Parameters for the task
    ,  3//2 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL//&xTaskTFT 
    ); //Task Handle

  //Sensor Read and Process Event Flow: The SensorHandlerAO will read sensor data and send events to DisplayAO and PumpControlAO.
  xTaskCreate(
    Sens_AO
    ,  "Sens the soil moisture"  // A name just for humans
    ,  108  // This stack size 
    ,  NULL //Parameters for the task
    ,  3  // Priority, with 3 being the highest, and 0 being the lowest.
    ,  NULL//N&xTaskSens  // Task handle 
    ); //Task Handle

  //Pump Control Event: PumpControlAO adjusts the pump operation based on the setpoint and sensor feedback.
  xTaskCreate(
    Pump_AO
    ,  "TaskreadSoilMoireONE"  // A name just for humans
    ,  118  // This stack size 
    ,  NULL //Parameters for the task
    ,  3//4    // Priority, with 3 being the highest, and 0 being the lowest.
    ,  NULL//&xTaskPump  // Task handle 
    ); //Task Handle
  
  xTimerStart(sensorTimer, 0);  // Start the timer imedietly
  xTimerStart(encoderTimer, 0);  // Start the timer imedietly
  

  vTaskStartScheduler();
}

//Proportional Integral derivate Calculation for time to run the pump
float computePID(int input,  int setpoint){ 
  // Initialize your PID controller, menu, etc.
  // You can also set these values here or keep them in the struct constructor
  /*
  static const float kp = 2.0;
  static const float ki = 0.1;
  static const float kd = 1.1;
    */
  // Tuned PID values
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
  if (integral > 1000) {integral = 1000;}  // limit the Integral to prevent creeping up
  else if (integral < -1000) {integral = -1000;}
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
  // Turn off the pump
  PORTB |= (1 << 1);
}

// Callback function is periodic every 6 seconds
void sensorTimerCallback(TimerHandle_t xTimer) {
  // Create sensor event and asigne the data from sensor to it
  Event sensorEvent = {EVENT_SENSOR_UPDATE, &sensorData};
  // Send sensor event
  xQueueSend(sensorQueue, &sensorEvent,(TickType_t) 0);
}

// Callback function is periodic every 6 seconds
void encoderTimerCallback(TimerHandle_t xTimer) {
      // Create the event
    Event encoderEvent = {EVENT_ENCODER_UPDATE,&encoderData};
    // Send the Event carrying the data to update the screen and new setpoint variable in pump task
    //xQueueSend(displayQueue, &encoderEvent, (TickType_t) 0);  
    xQueueSend(pumpQueue, &encoderEvent, (TickType_t) 0);  
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
  static const byte soilSensorRequest[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B};
  byte soilSensorResponse[9];
  while (1) {         
    
    // poll is used instead of vTaskdelay() - Intentionaly the CPU is not being released, 
    // otherwise transmition will be broken

    // send request to RS485 device
    mod.write(soilSensorRequest, sizeof(soilSensorRequest));
    //poll untoil software serial buffer is full of 9 bytes
    delayMs(10);
    // have the 1 second timeout pre-calculated using RTOS timer tick 
    TickType_t timeout = xTaskGetTickCount() + pdMS_TO_TICKS(1000); 
    // Stay here until the buffer is filled up
    while (mod.available() < 9 && xTaskGetTickCount() < timeout) {
      delayMs(1);   //poll untoil software serial buffer is full of 9 bytes
    }
    // When buffer is already full, store the data to an array
    if (mod.available() >= 9) {
      for (byte i = 0; i < 9; i++) {
        delayMs(1);  //poll to give the serial port a chance to store byte values for each index
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

    // create event
    Event sensorEvent = {EVENT_SENSOR_UPDATE, &sensorData};
    // Send the event to disply task Imedietly
    xQueueSend(displayQueue, &sensorEvent,(TickType_t) 0);    
    }
    else{
      //otherwise do nothing
    }  
    // 500 ms delay for smoother updates  
    vTaskDelay(pdMS_TO_TICKS(250)); 
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
        // limit the setpoint to range 0 - 100  
        if (encoderData.setpoint > 100){ encoderData.setpoint = 100;}  
        else if(encoderData.setpoint < 0){ encoderData.setpoint = 0;}
        last = value; 
      }        
    }
      Event encoderEvent = {EVENT_ENCODER_UPDATE,&encoderData};
    // Send the Event carrying the data to update the screen and new setpoint variable in pump task
    xQueueSend(displayQueue, &encoderEvent, (TickType_t) 0);  
    //xQueueSend(pumpQueue, &encoderEvent, (TickType_t) 0);  
    // 10 ms delay for responsivenes
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}

void TFT_AO(void *pvParameters) {
  (void*) pvParameters;   
  while (1) {   
    // do not interrupt SPI communication with TFT
    if (xQueueReceive(displayQueue, &event, portMAX_DELAY) == pdTRUE) { 
      configASSERT(event != (Event)0);
      if (event.type == EVENT_ENCODER_UPDATE ) {
        EncoderData  *data = (EncoderData*) event.data;
        tft.setTextSize(3);
        tft.setCursor(25, 94);
        if(data->adjustMode){
          tft.setTextColor(ST7735_BLACK, ST7735_GREEN); 
          taskENTER_CRITICAL(); // Disable interrupts here if needed
          tft.print(">"); 
          //size_t num = xPortGetFreeHeapSize();
          int num = data->setpoint;
          if (num < 10) {
              tft.print("  ");  // Add two spaces for single-digit numbers
          } else if (num < 100) {
              tft.print(" ");   // Add one space for two-digit numbers
          }
          tft.print(num);       // Print the number itself
          tft.print("<"); 
          taskEXIT_CRITICAL(); // Re-enable interrupts
        }
        else{
          tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
          taskENTER_CRITICAL(); // Disable interrupts here if needed
          tft.print(" "); 
          //size_t num_1 = xPortGetFreeHeapSize();
          int num_1 = data->setpoint;
          if (num_1 < 10) {
              tft.print("  ");  // Add two spaces for single-digit numbers
          } else if (num_1 < 100) {
              tft.print(" ");   // Add one space for two-digit numbers
          }
          tft.print(num_1);       // Print the number itself
          tft.print(" "); 
          taskEXIT_CRITICAL(); // Re-enable interrupts
        }   

        tft.setTextColor(ST7735_GREEN, ST7735_BLACK);   
        tft.setTextSize(2);
      }
      else if (event.type == EVENT_SENSOR_UPDATE) {
        SensorData *data = (SensorData*) event.data;
        tft.setTextColor(ST7735_GREEN, ST7735_BLACK);   
        tft.setTextSize(2);
        taskENTER_CRITICAL(); // Disable interrupts here if needed
        tft.setCursor(105, 0);
        int num = data->moisture;
        if (num < 10) {
            tft.print("  ");  // Add two spaces for single-digit numbers
        } else if (num < 100) {
            tft.print(" ");   // Add one space for two-digit numbers
        }
        tft.print(num);       // Print the number itself
        //tft.print(data->moisture);
        tft.setCursor(105, 20);
        int digit = data->temperature;
        if (digit < 10) {
            tft.print("  ");  // Add two spaces for single-digit numbers
        } else if (digit < 100) {
            tft.print(" ");   // Add one space for two-digit numbers
        }
        tft.print(digit);       // Print the number itself
        //tft.print(data->temperature);
        taskEXIT_CRITICAL(); // Re-enable interrupts
      }
    } 
    //taskYIELD();
    //
    vTaskDelay(pdMS_TO_TICKS(80)); // 120 ms delay for smoother updates
  }
}



// Task function using Mutex
void Pump_AO(void *pvParameters) {
  (void*) pvParameters;
  // Set PB1 aka pin (9) as output, ignore the rest (for fast execution and memory saving, using C)
  DDRB |= 0x02; 		   // XXXXXXXX | 00000010 = XXXXXX1X 
  PORTB |= (1 << 1);   // Initialize pump to be off (HIGH means off for active low)
  // After initializing period of 1 second, chenge to 30 000 which works out 2 min according to stopwatched revision
  //xTimerChangePeriod(sensorTimer, pdMS_TO_TICKS(3000), 0);
  int newSetpoint = 60;   
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
    //taskYIELD();   
  vTaskDelay(pdMS_TO_TICKS(80)); // 100 ms delay for smoother updates
  }
}

void loop(){ /*Empty. Things are done in Tasks*/ }

/*----------------------------------------------------------

Adjustment done in RTOSConfig.h file

 
#define configTICK_RATE_HZ   ( ( TickType_t ) 1000 )
#define configTOTAL_HEAP_SIZE  ( ( size_t ) ( 1280   ) )

//#define configUSE_TASK_NOTIFICATIONS 1
//#define configTASK_NOTIFICATION_ARRAY_ENTRIES 2

* Delay definition - here, the user can choose which delay implementation is required.
 * The default is to change nothing. *
#define configUSE_PORT_DELAY               1

/* And on to the things the same no matter the AVR type... *
#define configUSE_PREEMPTION                0//1

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

/* Timer definitions. *
#define configUSE_TIMERS                    1
#define configTIMER_TASK_PRIORITY           ( configMAX_PRIORITIES-1 )
#define configTIMER_TASK_STACK_DEPTH        68
#define configTIMER_QUEUE_LENGTH            5//10

 Current Compiled version at:
Sketch uses 25722 bytes (79%) of program storage space. Maximum is 32256 bytes.
Global variables use 672 bytes (32%) of dynamic memory, leaving 1376 bytes for local variables. Maximum is 2048 bytes.

*/
