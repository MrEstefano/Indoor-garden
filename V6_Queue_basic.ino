/********************************************************************/
/*  Name    : Indoor Garden V6                                      */
/*  Author  : Stefan Zakutansky ATU.ie student                      */
/*  Date    : 11. 8.2023                                            */
/*  Notes   : Queuing architecture, to mantain favorable soil       */
/*            moisture condition, measuring moisture by capacitive  */
/*            sensor, and turning LOW active water pump             */
/********************************************************************/

#define relayPin A2           /* Pump is contrpolled via PUMP+ on PCB*/
#define moisturePin A0        /* Capacitive moisture sensor is connected to SOIL on PCB*/
#define drySoil 740           /* dry soil moisture value from calibration*/
#define wetSoil 950           /* wet soil moisture value from calibration*/ 
#define moistureThreshold 60  /* treshold value from experiments for soil moisture */

  
unsigned long moistureCheckInterval = 120000; // Interval to check moisture (2 minute) 
unsigned long pumpRunTime = 30000; // How long to run the pump (30 seconds)
bool pumpState = false;


// Structure for a task in the queue to add a delay (the execution time)
struct Task { 
  void (*function)(); // Pointer to the function to execute 
  unsigned long executeTime; // Time when the task should be executed 
}; 

// Queue for tasks 
const int queueSize = 10; 
Task taskQueue[queueSize]; 
int queueStart = 0; 
int queueEnd = 0; 

// Function to add a task to the queue 
void enqueue(void (*function)(), unsigned long delayTime) { 
  unsigned long currentTime = millis(); 
  taskQueue[queueEnd].function = function; 
  taskQueue[queueEnd].executeTime = currentTime + delayTime; 
  queueEnd = (queueEnd + 1) % queueSize; 
} 

// Function to execute and remove the first task in the queue 
void dequeue() { 
  if (queueStart != queueEnd) { 
    // Check if the queue is not empty 
    unsigned long currentTime = millis();
    if (currentTime >= taskQueue[queueStart].executeTime) { 
      taskQueue[queueStart].function(); // Execute the task 
      queueStart = (queueStart + 1) % queueSize; // Remove the task from the queue 
    }
  }
} 

// Task to check soil moisture 
void checkMoisture() { 
  int moistureValue = analogRead(moisturePin); // Read the capacitive sensor
  moistureValue = constrain(moistureValue,drySoil,wetSoil); // needed to eliminate random readings outside the dry-wet boundries
  Serial.print("Raw Moisture Value : ");
  Serial.println(moistureValue);  
  moistureValue = map(moistureValue ,drySoil ,wetSoil ,0 ,100); // map the range in percantage
  Serial.print("Soil Moisture Level: ");
  Serial.print(moistureValue);   
  Serial.println(" %");
  //check the moisture percentage with treashold value and last state of the pump
  if (moistureValue < moistureThreshold && !pumpState) { 
    Serial.println("Soil Moisture low. Turning on the pump...");
    digitalWrite(relayPin, LOW); // Turn on the relay (active low) 
    pumpState = true;
    enqueue(stopPump, pumpRunTime); // Schedule a task to stop the pump after 30 seconds    
    enqueue(checkMoisture, moistureCheckInterval); // Re-schedule the moisture check task  
  }
  else{
  Serial.println("Soil Moisture above treashold. Lets check again...in 2 minutes ");
  enqueue(checkMoisture, moistureCheckInterval); // Re-schedule the moisture check task 
  } 
} 

// Task to stop the pump 
void stopPump() { 
  if(pumpState){
  Serial.println("Turning off the pump..."); 
  digitalWrite(relayPin, HIGH); // Turn off the relay 
  pumpState = false;
  }
} 

void setup() { 
  Serial.begin(112500); 
  pinMode(moisturePin, INPUT); 
  pinMode(relayPin, OUTPUT); 
  digitalWrite(relayPin, HIGH); // Initially turn off the pump 
  // Schedule the first moisture check 
  enqueue(checkMoisture, 0); // Start immediately 
} 

void loop() { 
  dequeue(); // Execute tasks as their time comes
}
