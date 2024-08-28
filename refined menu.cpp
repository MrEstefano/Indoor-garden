void updateTFTDisplay(int value, uint16_t color) {
  tft.fillRect(value, 128, 8, 19, color);
}

void startIpDisplay() {
  tft.drawRoundRect(40, 90, 40, 25, 5, ST7735_BLACK);
  tft.fillRoundRect(40, 90, 40, 25, 5, ST7735_BLACK);
  tft.setCursor(55, 95);
  tft.println(count);

  uint16_t colors[] = {
      ST7735_RED, ST7735_YELLOW, ST7735_YELLOW, ST7735_GREEN,
      ST7735_GREEN, ST7735_GREEN, ST7735_GREEN, ST7735_GREEN,
      ST7735_GREEN};

  if (count > 0 && count <= 9) {
    updateTFTDisplay(22 + (10 * (9 - count)), colors[9 - count]);
  }
}

void handleEncoderButton(ClickEncoder::Button b) {
  if (b == ClickEncoder::Clicked) {
    middle = true;
  }
}

void adjustMenuSettings(bool isUp, int page, int& menuItem, int& lastMenuItem, int& frame) {
  if (page == 1) {
    if (isUp) {
      lastMenuItem = menuItem;
      menuItem++;
      if (menuItem > 6) menuItem = 6;
      if (menuItem == 3 || menuItem == 4 || (menuItem == 5 && frame != 4)) frame++;
    } else {
      lastMenuItem = menuItem;
      menuItem--;
      if (menuItem < 1) menuItem = 1;
      if (menuItem == 2 || menuItem == 4 || (menuItem == 3 && frame != 2)) frame--;
    }
  } else if (page == 2) {
    int* value = nullptr;
    switch (menuItem) {
      case 1: value = &setpoint; break;
      case 2: value = &flow_limit; break;
      case 3: value = &kp; break;
      case 4: value = &ki; break;
    }
    if (value) {
      if (isUp) (*value)++;
      else (*value)--;
    }
  }
}

void handleMenuSelection(int& menuItem, int& page) {
  if (menuItem == 5 && page == 1) { // Pump manual overwrite
    pumpState = !pumpState;
    tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
    menuItem5 = pumpState ? "Pump: ON" : "Pump: OFF";
    digitalWrite(relayPin, pumpState ? LOW : HIGH);
  } else if (menuItem == 6 && page == 1) {
    resetDefaults();
  } else if (menuItem <= 4 && page == 1) {
    tft.fillRect(0, 55, 128, 105, ST7735_BLACK);  //refresh 
    page = 2;
  } else if (page == 2) {
    tft.fillRect(0, 55, 128, 105, ST7735_BLACK);  //refresh 
    page = 1;
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(moisturePin, INPUT);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH); // Initially turn off the pump
  pinMode(ENCODER_SW, INPUT_PULLUP); // Set encoder button as input with internal pull-up
  pinMode(FLOW_SENSOR_PIN, INPUT);
  
  // Initialize TFT 1.77 inch screen
  tft.initR(INITR_BLACKTAB);    
  tft.fillScreen(ST7735_BLACK);

  // Run intro graphics
  tft.drawRoundRect(3, 5, 122, 150, 5, ST7735_BLUE);
  tft.setCursor(30, 20);
  tft.setTextColor(ST7735_GREEN);
  tft.setTextSize(2);
  tft.println("SYSTEM");
  tft.setCursor(30, 45);
  tft.println("STARTS");
  tft.setCursor(50, 70);
  tft.println("IN");
  tft.drawRect(8, 125, 112, 25, ST7735_BLUE);
  tft.fillRect(11, 128, 8, 19, ST7735_RED);
  for (byte i = 10; count > 0; i--) {
    count--;
    startIpDisplay(); 
    delay(500);   
  }
  tft.fillScreen(ST7735_BLACK); // Clear the screen (white background) 
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr); 
  
  // Encoder entity
  encoder = new ClickEncoder(ENCODER_CLK, ENCODER_DT, ENCODER_SW, 1, false);
  encoder->setAccelerationEnabled(false);

  last = encoder->getValue();
  // Schedule the first moisture check
  enqueue(checkMoisture, 0); // Start measuring soil moisture immediately
 
  startingTimeStamp = millis();
}

void loop() {
  dequeue(); // Execute tasks as their time comes
  drawMenu();
  readRotaryEncoder();

  ClickEncoder::Button b = encoder->getButton();
  handleEncoderButton(b);
  
  adjustMenuSettings(up, page, menuItem, lastMenuItem, frame);
  adjustMenuSettings(down, page, menuItem, lastMenuItem, frame);
  
  if (middle) {
    middle = false;
    handleMenuSelection(menuItem, page);
  }
}
