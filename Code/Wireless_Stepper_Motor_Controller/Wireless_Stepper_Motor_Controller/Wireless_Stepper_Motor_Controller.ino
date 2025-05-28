/*
 * ===============================================================================
 * TMC2240 STEPPER MOTOR CONTROL WITH PD POWER DELIVERY & RGB STATUS INDICATORS
 * ===============================================================================
 * 
 * DESCRIPTION:
 * This project implements a comprehensive stepper motor control system using the 
 * TMC2240 driver with advanced features including PD (Power Delivery) negotiation,
 * magnetic encoder feedback, and RGB LED status indication.
 * 
 * MAIN FEATURES:
 * 1. TMC2240 Stepper Motor Driver Control
 *    - Configurable microstepping (1 to 256 steps)
 *    - Current control (up to 2A+)
 *    - SPI communication interface
 *    - Real-time temperature monitoring
 * 
 * 2. USB-C Power Delivery (PD) Integration
 *    - FUSB302 PD controller support
 *    - Multiple voltage profiles (5V, 9V, 12V, 15V, 20V)
 *    - Automatic power negotiation
 *    - Real-time voltage/current monitoring
 * 
 * 3. AS5600 Magnetic Encoder Integration
 *    - 12-bit resolution (4096 steps per revolution)
 *    - Absolute angle positioning
 *    - I2C communication
 *    - Precise angle-based motor control
 * 
 * 4. RGB LED Status System
 *    - WiFi connection status indication
 *    - PD voltage level color coding
 *    - Motor state indication (enabled/disabled)
 *    - Web client connection status
 * 
 * 5. Web-based Control Interface
 *    - Real-time motor control
 *    - Parameter adjustment (speed, current, microstepping)
 *    - System status monitoring
 *    - Emergency stop functionality
 * 
 * 6. Advanced Motion Control
 *    - Absolute angle positioning
 *    - Dynamic speed control
 *    - Precision movement with tolerance
 *    - Direction control
 * 
 * WORKING PRINCIPLE:
 * The system operates by combining multiple control interfaces:
 * - Web interface provides user control and monitoring
 * - PD controller manages power delivery from USB-C source
 * - TMC2240 drives the stepper motor with precise control
 * - AS5600 provides position feedback for closed-loop control
 * - RGB LED provides visual status feedback
 * 
 * HARDWARE CONNECTIONS:
 * - TMC2240: SPI interface (MOSI=11, MISO=13, SCK=12, CS=10)
 * - AS5600: I2C interface (SDA=8, SCL=9)
 * - FUSB302: Digital pin 7 for interrupt
 * - RGB LED: Pins 40(R), 42(G), 41(B)
 * - Motor Control: EN=14, STEP=5, DIR=6, UART_EN=15
 * 
 * AUTHOR: [Rithik Krisna M]
 * DATE: [28/05/2025]
 * VERSION: 1.0
 * ===============================================================================
 */

// ===============================================================================
// LIBRARY INCLUDES
// ===============================================================================
#include <SPI.h>           // SPI communication for TMC2240
#include <WiFi.h>          // WiFi connectivity
#include <WebServer.h>     // HTTP server for web interface
#include "AS5600.h"        // Magnetic encoder library
#include <Wire.h>          // I2C communication
#include <PD_UFP.h>        // USB-C Power Delivery library
#include <ArduinoJson.h>   // JSON handling for web API
#include "index_page.h"    // HTML page for web interface

// ===============================================================================
// PIN DEFINITIONS
// ===============================================================================
#define FUSB302_INT_PIN 7  // Interrupt pin for PD controller

// WiFi Configuration
const char* ssid = "xxxx";      // WiFi network name
const char* password = "xxxx";   // WiFi password

// ===============================================================================
// OBJECT INSTANTIATION
// ===============================================================================
WebServer server(80);      // Web server on port 80
AS5600 as5600;            // Magnetic encoder object
PD_UFP_c PD_UFP;          // Power Delivery controller object

// ===============================================================================
// POWER DELIVERY PROFILES CONFIGURATION
// ===============================================================================
// Structure to define PD power profiles with voltage, current, and name
struct PDProfile {
    float voltage;         // Voltage in volts
    float current;         // Current in amperes
    const char* name;      // Human-readable name
};

// Predefined PD power profiles for different voltage levels
const PDProfile pd_profiles[] = {
    {5.0, 3.0, "5V/3A"},      // USB standard power
    {9.0, 2.0, "9V/2A"},      // Quick charge level 1
    {9.0, 3.0, "9V/3A"},      // Quick charge level 2
    {12.0, 1.5, "12V/1.5A"},  // Low power 12V
    {15.0, 2.0, "15V/2A"},    // Medium power 15V
    {20.0, 1.5, "20V/1.5A"}   // High voltage 20V
};

// ===============================================================================
// RGB LED CONFIGURATION
// ===============================================================================
// Pin definitions for RGB LED (common anode or cathode)
const int RGB_RED_PIN = 40;    // Red LED pin
const int RGB_GREEN_PIN = 42;  // Green LED pin
const int RGB_BLUE_PIN = 41;   // Blue LED pin

// RGB color structure for easy color management
struct RGBColor {
  int red;    // Red component (0-255)
  int green;  // Green component (0-255)
  int blue;   // Blue component (0-255)
};

// Predefined color constants for different states
const RGBColor COLOR_OFF = { 0, 0, 0 };         // LED off
const RGBColor COLOR_RED = { 255, 0, 0 };       // Error/disconnected
const RGBColor COLOR_GREEN = { 0, 255, 0 };     // Connected/5V
const RGBColor COLOR_BLUE = { 0, 0, 255 };      // Waiting/9V
const RGBColor COLOR_YELLOW = { 255, 255, 0 };  // Connecting
const RGBColor COLOR_PURPLE = { 255, 0, 255 };  // 12V
const RGBColor COLOR_CYAN = { 0, 255, 255 };    // Special state
const RGBColor COLOR_WHITE = { 255, 255, 255 }; // Unknown/default
const RGBColor COLOR_ORANGE = { 255, 165, 0 };  // 15V
const RGBColor COLOR_PINK = { 255, 20, 147 };   // 20V

// ===============================================================================
// LED STATE MANAGEMENT VARIABLES
// ===============================================================================
unsigned long lastLEDUpdate = 0;       // Last LED update timestamp
unsigned long ledBlinkInterval = 500;   // Blink interval in milliseconds
bool ledBlinkState = false;             // Current blink state (on/off)
RGBColor currentLEDColor = COLOR_OFF;   // Current LED color
bool ledBlinkEnabled = false;           // Blink enable flag
unsigned long lastClientRequest = 0;    // Last web client request time
bool webClientConnected = false;        // Web client connection status
bool pdColorModeActive = false;         // PD color mode active flag

// ===============================================================================
// TMC2240 STEPPER DRIVER CONFIGURATION
// ===============================================================================
// SPI pin definitions for TMC2240 communication
const int MOSI_PIN = 11, MISO_PIN = 13, SCK_PIN = 12, CS_PIN = 10;

// Motor control pin definitions
const int EN_PIN = 14;      // Enable pin (LOW = enabled, HIGH = disabled)
const int STEP_PIN = 5;     // Step pulse pin
const int DIR_PIN = 6;      // Direction pin (HIGH/LOW for CW/CCW)
const int UART_EN_PIN = 15; // UART enable pin (not used in SPI mode)

// ===============================================================================
// TMC2240 REGISTER ADDRESSES
// ===============================================================================
const uint8_t GCONF = 0x00;      // General configuration register
const uint8_t CHOPCONF = 0x6C;   // Chopper configuration register
const uint8_t IHOLD_IRUN = 0x10; // Current control register
const uint8_t TPOWERDOWN = 0x11; // Power down delay register
const uint8_t MSCNT = 0x6A;      // Microstep counter register
const uint8_t MSCURACT = 0x6B;   // Actual microstep current register
const uint8_t SG_RESULT = 0x40;  // StallGuard result register
const uint8_t DRV_STATUS = 0x6F; // Driver status register
const uint8_t ADC_TEMP = 0x51;   // Temperature ADC register

// ===============================================================================
// MOTOR CONFIGURATION CONSTANTS
// ===============================================================================
const uint16_t DEFAULT_CURRENT = 1000;     // Default motor current in mA
const uint8_t DEFAULT_MICROSTEPS = 16;     // Default microstepping setting
const uint16_t DEFAULT_SPEED = 5;          // Default speed in steps/sec
const int STEPS_PER_REV = 200;             // Steps per revolution (1.8° motors)
const int MIN_STEP_DELAY = 100;            // Minimum delay between steps (μs)

// ===============================================================================
// GLOBAL STATE VARIABLES
// ===============================================================================
// Motor state variables
bool isMotorEnabled = false;               // Motor enable state
bool currentDirection = true;              // Current direction (true = forward)
bool isMovingToAngle = false;             // Angle movement in progress flag
bool pd_negotiation_complete = false;     // PD negotiation status

// Motor parameter variables
int currentSpeed = DEFAULT_SPEED;          // Current speed setting
uint16_t motorCurrent = DEFAULT_CURRENT;   // Current motor current setting
uint8_t microSteps = DEFAULT_MICROSTEPS;   // Current microstepping setting
int STEPS_PER_REVOLUTION = STEPS_PER_REV * DEFAULT_MICROSTEPS; // Total steps per rev

// Position control variables
float targetAngle = 0.0;                   // Target angle for movement
float currentAngleTolerance = 0.0;         // Current angle tolerance
float cachedCurrentAngle = 0.0;            // Cached current angle reading
float cachedTemperature = 0.0;             // Cached temperature reading
int cachedCurrentB = 0;                    // Cached current reading

// ===============================================================================
// TIMING CONTROL VARIABLES
// ===============================================================================
unsigned long lastStatusUpdate = 0;        // Last status update timestamp
unsigned long lastEncoderUpdate = 0;       // Last encoder update timestamp
unsigned long last_pd_check = 0;           // Last PD check timestamp
unsigned long lastPDVoltageCheck = 0;      // Last PD voltage check timestamp

// Timing intervals (in milliseconds)
const unsigned long STATUS_UPDATE_INTERVAL = 1000;   // Status update every 1 second
const unsigned long ENCODER_UPDATE_INTERVAL = 100;   // Encoder update every 100ms
const unsigned long PD_CHECK_INTERVAL = 100;         // PD check every 100ms
const unsigned long CLIENT_TIMEOUT = 5000;           // Web client timeout (5 seconds)

// Current PD profile index
int currentPDProfile = 0; // Default to the first profile (5V/3A)

// PD voltage check interval
const unsigned long PD_VOLTAGE_CHECK_INTERVAL = 100; 

// ===============================================================================
// RGB LED CONTROL FUNCTIONS
// ===============================================================================

/**
 * Set RGB LED to specified color
 * @param color RGBColor structure with red, green, blue values
 */
void setRGBColor(RGBColor color) {
  analogWrite(RGB_RED_PIN, color.red);     // Set red component
  analogWrite(RGB_GREEN_PIN, color.green); // Set green component
  analogWrite(RGB_BLUE_PIN, color.blue);   // Set blue component
  currentLEDColor = color;                 // Update current color
}

/**
 * Set RGB LED with individual color values
 * @param red Red component (0-255)
 * @param green Green component (0-255)
 * @param blue Blue component (0-255)
 */
void setRGBColor(int red, int green, int blue) {
  RGBColor color = { red, green, blue };
  setRGBColor(color);
}

/**
 * Turn off RGB LED and disable blinking
 */
void turnOffLED() {
  setRGBColor(COLOR_OFF);
  ledBlinkEnabled = false;
}

/**
 * Enable LED blinking with specified color and interval
 * @param color Color to blink
 * @param interval Blink interval in milliseconds (default: 500ms)
 */
void enableLEDBlink(RGBColor color, unsigned long interval = 500) {
  currentLEDColor = color;
  ledBlinkInterval = interval;
  ledBlinkEnabled = true;
  ledBlinkState = false;
  lastLEDUpdate = millis();
}

/**
 * Update blinking LED state (call in main loop)
 * Handles the timing and state changes for LED blinking
 */
void updateLEDBlink() {
  if (!ledBlinkEnabled) return;

  // Check if it's time to toggle the LED
  if (millis() - lastLEDUpdate >= ledBlinkInterval) {
    if (ledBlinkState) {
      setRGBColor(COLOR_OFF);           // Turn off LED
    } else {
      setRGBColor(currentLEDColor);     // Turn on LED with current color
    }
    ledBlinkState = !ledBlinkState;     // Toggle blink state
    lastLEDUpdate = millis();           // Update last update time
  }
}

/**
 * Update status LED based on system state priority
 * Priority: WiFi > PD Status > Client Connection
 */
void updateStatusLED() {
  // Highest Priority: WiFi disconnected - Red fast blink
  if (WiFi.status() != WL_CONNECTED) {
    pdColorModeActive = false;
    enableLEDBlink(COLOR_RED, 200);  // Red fast blink for WiFi issues
    return;
  }
  
  // Second Priority: PD is ready and negotiated - show voltage colors
  if (PD_UFP.is_power_ready() && pd_negotiation_complete) {
    // Only update PD colors periodically to avoid constant changes
    if (millis() - lastPDVoltageCheck >= PD_CHECK_INTERVAL) {
      updatePDVoltageColor();
      lastPDVoltageCheck = millis();
    }
    return; // Exit here - PD colors are active
  }
  
  // Third Priority: WiFi connected but no PD - show connection status
  pdColorModeActive = false;
  
  // Update client connection status based on timeout
  if (millis() - lastClientRequest > CLIENT_TIMEOUT) {
    webClientConnected = false;
  }

  if (webClientConnected) {
    // WiFi connected and web client active - Solid Green
    setRGBColor(COLOR_GREEN);
    ledBlinkEnabled = false;
  } else {
    // WiFi connected but no web client - Blue slow blink
    enableLEDBlink(COLOR_BLUE, 1000);
  }
}

/**
 * Update LED color based on PD voltage level
 * Different colors represent different voltage levels:
 * Green=5V, Blue=9V, Purple=12V, Orange=15V, Pink=20V, Red=Unknown
 */
void updatePDVoltageColor() {
  float voltage = pd_profiles[currentPDProfile].voltage; // Use selected profile voltage
  RGBColor voltageColor = COLOR_WHITE; // Default for unknown voltage

  // Determine color based on voltage range
  if (voltage >= 4.5 && voltage < 6.0) {
    voltageColor = COLOR_GREEN;          // 5V - Green
  } else if (voltage >= 8.5 && voltage < 10.0) {
    voltageColor = COLOR_BLUE;           // 9V - Blue
  } else if (voltage >= 11.5 && voltage < 13.0) {
    voltageColor = COLOR_PURPLE;         // 12V - Purple
  } else if (voltage >= 14.5 && voltage < 16.0) {
    voltageColor = COLOR_ORANGE;         // 15V - Orange
  } else if (voltage >= 19.0 && voltage < 21.0) {
    voltageColor = COLOR_PINK;           // 20V - Pink
  } else if (voltage > 0.0) {
    voltageColor = COLOR_RED;            // Unknown voltage - Red
  } else {
    voltageColor = COLOR_WHITE;          // No voltage/not ready - White
  }
  
  // Set PD color mode active
  pdColorModeActive = true;
  
  // Set blink speed based on motor state
  if (isMotorEnabled) {
    enableLEDBlink(voltageColor, 200);   // Fast blink when motor enabled
  } else {
    enableLEDBlink(voltageColor, 1000);  // Slow blink when motor disabled  
  }
  
  // Debug output for voltage color (can be removed in production)
  if (voltageColor.red == COLOR_GREEN.red && voltageColor.green == COLOR_GREEN.green) Serial.print("GREEN");
  else if (voltageColor.red == COLOR_BLUE.red && voltageColor.green == COLOR_BLUE.green) Serial.print("BLUE");
  else if (voltageColor.red == COLOR_PURPLE.red && voltageColor.green == COLOR_PURPLE.green) Serial.print("PURPLE");
  else if (voltageColor.red == COLOR_ORANGE.red && voltageColor.green == COLOR_ORANGE.green) Serial.print("ORANGE");
  else if (voltageColor.red == COLOR_PINK.red && voltageColor.green == COLOR_PINK.green) Serial.print("PINK");
  else if (voltageColor.red == COLOR_RED.red && voltageColor.green == COLOR_RED.green) Serial.print("RED");
  else Serial.print("WHITE");
}

/**
 * Update motor status color when motor state changes
 */
void updateMotorStatusColor() {
  // Only update if PD is ready, otherwise let updateStatusLED handle it
  if (PD_UFP.is_power_ready() && pd_negotiation_complete) {
    updatePDVoltageColor();
  }
}

/**
 * Force immediate PD color update (call when motor state changes)
 */
void forceUpdatePDColors() {
  if (PD_UFP.is_power_ready() && pd_negotiation_complete) {
    lastPDVoltageCheck = 0; // Force immediate update
    updatePDVoltageColor();
  }
}

/**
 * Initialize RGB LED pins and test sequence
 */
void initRGBLED() {
  // Set pins as outputs
  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);

  // Test sequence to verify LED functionality
  setRGBColor(COLOR_RED);
  delay(200);
  setRGBColor(COLOR_GREEN);
  delay(200);
  setRGBColor(COLOR_BLUE);
  delay(200);
  turnOffLED();

  Serial.println("RGB LED initialized");
}

// ===============================================================================
// TMC2240 DRIVER CONTROL FUNCTIONS
// ===============================================================================

// Function declarations for TMC2240 control
void configureDriver();                              // Configure TMC2240 with default settings
void setCurrent(uint16_t current);                   // Set motor current
void setMicrostepping(uint8_t microsteps);          // Set microstepping resolution
void writeRegister(uint8_t address, uint32_t value); // Write to TMC2240 register
uint32_t readRegister(uint8_t address);             // Read from TMC2240 register
void updateCachedValues();                          // Update cached sensor values
void moveMotor(bool direction, uint32_t steps);     // Move motor specified steps
void moveToAngle();                                 // Move to target angle
float calculateAngleTolerance(uint8_t microsteps);  // Calculate angle tolerance

// ===============================================================================
// MAIN SETUP FUNCTION
// ===============================================================================
void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  Serial.println("TMC2240 Stepper Motor Control");
  
  // Initialize RGB LED system
  initRGBLED();
  
  // Initialize I2C for AS5600 encoder
  Wire.setPins(8, 9);  // Set custom I2C pins
  Wire.begin();

  // Initialize PD controller with 5V maximum initially
  PD_UFP.init(FUSB302_INT_PIN, PD_POWER_OPTION_MAX_5V);
  PD_UFP.set_power_option(PD_POWER_OPTION_MAX_5V);

  // Initialize AS5600 magnetic encoder
  as5600.begin(4);  // Begin with direction pin 4
  as5600.setDirection(AS5600_CLOCK_WISE);

  // Check encoder connection
  if (!as5600.isConnected()) {
    Serial.println("Warning: AS5600 encoder not connected!");
  }

  // Initialize TMC2240 control pins
  pinMode(EN_PIN, OUTPUT);      // Motor enable pin
  pinMode(STEP_PIN, OUTPUT);    // Step pulse pin
  pinMode(DIR_PIN, OUTPUT);     // Direction control pin
  pinMode(CS_PIN, OUTPUT);      // SPI chip select pin
  pinMode(MOSI_PIN, OUTPUT);    // SPI master out, slave in
  pinMode(MISO_PIN, INPUT);     // SPI master in, slave out
  pinMode(SCK_PIN, OUTPUT);     // SPI clock pin
  pinMode(UART_EN_PIN, OUTPUT); // UART enable (not used)

  // Set initial pin states
  digitalWrite(CS_PIN, HIGH);                    // Deselect TMC2240
  digitalWrite(EN_PIN, HIGH);                    // Disable motor initially
  digitalWrite(UART_EN_PIN, LOW);                // Disable UART mode
  digitalWrite(DIR_PIN, currentDirection);       // Set initial direction

  // Initialize SPI communication
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);

  // Connect to WiFi network
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  setRGBColor(COLOR_YELLOW);  // Yellow while connecting

  // Wait for WiFi connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  setRGBColor(COLOR_GREEN);  // Green when connected
  delay(1000);

  // ===============================================================================
  // WEB SERVER ROUTE CONFIGURATION
  // ===============================================================================
  
  // Main page route - serves the web interface
  server.on("/", HTTP_GET, []() {
    lastClientRequest = millis();
    webClientConnected = true;
    server.send(200, "text/html", html_page);
  });

  // Motor enable route - enables the stepper motor
  server.on("/enable", HTTP_GET, []() {
    lastClientRequest = millis();
    webClientConnected = true;
    digitalWrite(EN_PIN, LOW);        // Enable motor (active low)
    isMotorEnabled = true;
    updateMotorStatusColor();         // Update LED status
    server.send(200, "text/plain", "Motor Enabled");
  });

  // Motor disable route - disables the stepper motor
  server.on("/disable", HTTP_GET, []() {
    lastClientRequest = millis();
    webClientConnected = true;
    digitalWrite(EN_PIN, HIGH);       // Disable motor (active low)
    isMotorEnabled = false;
    updateMotorStatusColor();         // Update LED status
    server.send(200, "text/plain", "Motor Disabled");
  });

  // Emergency stop route - immediately stops all motor activity
  server.on("/stop", HTTP_GET, []() {
    lastClientRequest = millis();
    webClientConnected = true;
    digitalWrite(EN_PIN, HIGH);       // Disable motor
    digitalWrite(STEP_PIN, LOW);      // Ensure step pin is low
    isMotorEnabled = false;
    isMovingToAngle = false;          // Stop angle movement
    currentSpeed = DEFAULT_SPEED;     // Reset speed
    updateMotorStatusColor();         // Update LED status

    String response = "{\"status\":\"Emergency Stop Activated\",\"motorEnabled\":false}";
    server.send(200, "application/json", response);

    Serial.println("EMERGENCY STOP ACTIVATED");
  });

  // Speed control route - sets motor speed
  server.on("/speed", HTTP_GET, []() {
    lastClientRequest = millis();
    webClientConnected = true;
    if (server.hasArg("value")) {
      currentSpeed = server.arg("value").toInt();
      server.send(200, "text/plain", "Speed set to " + String(currentSpeed));
    }
  });

  // Manual movement route - moves motor by specified steps
  server.on("/move", HTTP_GET, []() {
    lastClientRequest = millis();
    webClientConnected = true;
    if (server.hasArg("steps")) {
      int targetSteps = server.arg("steps").toInt();
      moveMotor(currentDirection, targetSteps);
      server.send(200, "text/plain", "Moving " + String(targetSteps) + " steps");
    }
  });

  // System status route - returns current system status
  server.on("/status", HTTP_GET, []() {
    lastClientRequest = millis();
    webClientConnected = true;
    bool pd_ready = PD_UFP.is_power_ready();
    float pd_voltage = (pd_ready && pd_negotiation_complete) ? PD_UFP.get_voltage() : 0.0;
    float pd_current = (pd_ready && pd_negotiation_complete) ? PD_UFP.get_current() : 0.0;

    String status = "{\"temperature\":" + String(cachedTemperature, 1) + 
                   ",\"current\":" + String(cachedCurrentB) + 
                   ",\"pd_ready\":" + String(pd_ready ? "true" : "false") + 
                   ",\"pd_negotiated\":" + String(pd_negotiation_complete ? "true" : "false") + 
                   ",\"pd_voltage\":" + String(pd_voltage, 2) + 
                   ",\"pd_current\":" + String(pd_current, 2) + "}";
    server.send(200, "application/json", status);
  });

  // Motor current control route - sets motor current
  server.on("/current", HTTP_GET, []() {
    lastClientRequest = millis();
    webClientConnected = true;
    if (server.hasArg("value")) {
      motorCurrent = server.arg("value").toInt();
      setCurrent(motorCurrent);
      server.send(200, "text/plain", "Motor current set to " + String(motorCurrent) + " mA");
    }
  });

  // Microstepping control route - sets microstepping resolution
  server.on("/microsteps", HTTP_GET, []() {
    lastClientRequest = millis();
    webClientConnected = true;
    if (server.hasArg("value")) {
      microSteps = server.arg("value").toInt();
      setMicrostepping(microSteps);
      server.send(200, "text/plain", "Microstepping set to 1/" + String(microSteps));
    }
  });

  // Encoder status route - returns current encoder readings
  server.on("/encoder", HTTP_GET, []() {
    lastClientRequest = millis();
    webClientConnected = true;
    String json = String("{\"speed\":0") + 
                 ",\"rawAngle\":" + String(cachedCurrentAngle, 1) + 
                 ",\"current\":" + String(cachedCurrentB) + "}";
    server.send(200, "application/json", json);
  });

  // Power status route - returns PD power status
  server.on("/power_status", HTTP_GET, []() {
    lastClientRequest = millis();
    webClientConnected = true;
    StaticJsonDocument<200> doc;
    bool power_ready = PD_UFP.is_power_ready();
    float voltage = 0.0;
    float current = 0.0;

    if (power_ready && pd_negotiation_complete) {
      voltage = PD_UFP.get_voltage();
      current = PD_UFP.get_current();
    }

    bool pd_connected = power_ready && pd_negotiation_complete && voltage > 0.0 && current > 0.0;
    doc["pd_connected"] = pd_connected;
    doc["pd_negotiated"] = pd_negotiation_complete;

    if (pd_connected) {
      doc["voltage"] = voltage;
      doc["current"] = current;
      doc["power"] = voltage * current;
    }

    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });

  // Direction control route - sets motor direction
  server.on("/direction", HTTP_GET, []() {
    lastClientRequest = millis();
    webClientConnected = true;
    if (server.hasArg("dir")) {
      currentDirection = (server.arg("dir") == "true");
      digitalWrite(DIR_PIN, currentDirection);
      server.send(200, "text/plain", "Direction set to " + String(currentDirection ? "Forward" : "Backward"));
    } else {
      server.send(400, "text/plain", "Missing direction parameter");
    }
  });

  // Angle positioning route - moves motor to specific angle
  server.on("/set_angle", HTTP_GET, []() {
    lastClientRequest = millis();
    webClientConnected = true;
    if (server.hasArg("angle")) {
      targetAngle = server.arg("angle").toFloat();
      currentAngleTolerance = calculateAngleTolerance(microSteps);
      isMovingToAngle = true;
      server.send(200, "text/plain", "Moving to angle " + String(targetAngle) + "° with accuracy ±" + String(currentAngleTolerance) + "°");
    } else {
      server.send(400, "text/plain", "Missing angle parameter");
    }
  });

// Web server endpoint to set Power Delivery (PD) profile
server.on("/setPD", HTTP_GET, []() {
  // Update client activity tracking
  lastClientRequest = millis();
  webClientConnected = true;
  
  // Check if profile parameter is provided in the request
  if (server.hasArg("profile")) {
    int profile = server.arg("profile").toInt();  // Get requested voltage profile
    PD_power_option_t power_option;  // Variable to store PD power option
    
    // Map voltage profile to corresponding PD power option and profile index
    switch (profile) {
      case 5:
        power_option = PD_POWER_OPTION_MAX_5V;
        currentPDProfile = 0; // Index of 5V profile in pd_profiles[] array
        break;
      case 9:
        power_option = PD_POWER_OPTION_MAX_9V;
        currentPDProfile = 1; // Index of 9V profile in pd_profiles[] array
        break;
      case 12:
        power_option = PD_POWER_OPTION_MAX_12V;
        currentPDProfile = 2; // Index of 12V profile in pd_profiles[] array
        break;
      case 15:
        power_option = PD_POWER_OPTION_MAX_15V;
        currentPDProfile = 3; // Index of 15V profile in pd_profiles[] array
        break;
      case 20:
        power_option = PD_POWER_OPTION_MAX_20V;
        currentPDProfile = 4; // Index of 20V profile in pd_profiles[] array
        break;
      default:
        // Invalid profile requested - send error response
        server.send(400, "text/plain", "Invalid profile");
        return;
    }

    // Apply the new power option to PD controller
    PD_UFP.set_power_option(power_option);
    
    // Run PD controller multiple times to ensure negotiation
    for (int i = 0; i < 5; i++) {
      PD_UFP.run();  // Process PD negotiation
      delay(100);    // Small delay between runs
    }
    
    // Force immediate LED color update to reflect new voltage
    forceUpdatePDColors();
    
    // Send success response to client
    server.send(200, "text/plain", "PD profile set to " + String(profile) + "V");
  } else {
    // No profile parameter provided - send error response
    server.send(400, "text/plain", "Missing profile parameter");
  }
});

// Start the web server
server.begin();
Serial.println("HTTP server started");

// Configure the TMC2240 stepper motor driver
configureDriver();
}

// Main program loop - runs continuously
void loop() {
  // Handle incoming web server requests
  server.handleClient();
  
  // Run PD controller to maintain power negotiation
  PD_UFP.run();

  // Update LED blinking animation if enabled
  updateLEDBlink();
  
  // Update status LED periodically (every 100ms) to avoid constant updates
  static unsigned long lastStatusLEDUpdate = 0;
  if (millis() - lastStatusLEDUpdate >= 100) {
    updateStatusLED();  // Update LED based on system status
    lastStatusLEDUpdate = millis();
  }

  // Handle Power Delivery controller status checks
  if (millis() - last_pd_check >= PD_CHECK_INTERVAL) {
    bool was_negotiated = pd_negotiation_complete;  // Store previous state
    
    // Check if PD power is ready/negotiated
    if (PD_UFP.is_power_ready()) {
      if (!pd_negotiation_complete) {
        // Power negotiation just completed
        Serial.println("PD Power Negotiation Complete!");
        pd_negotiation_complete = true;
        // Force immediate LED color update to show new voltage
        lastPDVoltageCheck = 0;
      }
    } else {
      if (pd_negotiation_complete) {
        // Power negotiation was lost
        Serial.println("PD Power Negotiation Lost!");
        pd_negotiation_complete = false;
        pdColorModeActive = false;  // Disable PD color mode
      }
    }
    last_pd_check = millis();  // Reset check timer
  }

  // Update cached sensor values periodically
  if (millis() - lastEncoderUpdate >= ENCODER_UPDATE_INTERVAL) {
    updateCachedValues();  // Update angle, temperature, current readings
    lastEncoderUpdate = millis();
  }

  // Handle automatic angle-based motor movement
  if (isMovingToAngle) {
    moveToAngle();  // Continue moving motor to target angle
  }

  // Print system status to serial monitor periodically
  if (millis() - lastStatusUpdate >= STATUS_UPDATE_INTERVAL) {
    Serial.println("System Status: Motor " + String(isMotorEnabled ? "Enabled" : "Disabled") + 
                   ", Temp: " + String(cachedTemperature, 1) + "°C" + 
                   ", WiFi: " + String(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected") + 
                   ", Web Client: " + String(webClientConnected ? "Active" : "Inactive") +
                   ", PD Mode: " + String(pdColorModeActive ? "Active" : "Inactive"));
    lastStatusUpdate = millis();
  }
}

// Configure TMC2240 stepper driver with default settings
void configureDriver() {
  // Set motor current and microstepping to default values
  setCurrent(DEFAULT_CURRENT);
  setMicrostepping(DEFAULT_MICROSTEPS);

  // Set power down delay to 0 (immediate power down when idle)
  writeRegister(TPOWERDOWN, 0x0000);

  // Enable step interpolation for smoother movement
  uint32_t gconf = readRegister(GCONF);
  gconf |= (1 << 1);  // Set interpolation bit
  writeRegister(GCONF, gconf);

  // Set chopper off time (TOFF) to 5 for proper operation
  uint32_t chopconf = readRegister(CHOPCONF);
  chopconf &= ~(0x0F << 0);  // Clear TOFF bits
  chopconf |= (5 << 0);      // Set TOFF to 5
  writeRegister(CHOPCONF, chopconf);

  Serial.println("TMC2240 configured with default settings");
}

// Set motor current in milliamps
void setCurrent(uint16_t current) {
  uint32_t ihold_irun = 0;
  // Set hold current (current when motor is stationary)
  ihold_irun |= ((current / 100) & 0x1F) << 0;
  // Set run current (current when motor is moving)
  ihold_irun |= ((current / 100) & 0x1F) << 8;  
  // Set hold delay (time before reducing to hold current)
  ihold_irun |= ((current / 200) & 0x0F) << 16;
  writeRegister(IHOLD_IRUN, ihold_irun);
}

// Set microstepping resolution (1, 2, 4, 8, 16, 32, 64, 128, 256)
void setMicrostepping(uint8_t microsteps) {
  uint32_t chopconf = readRegister(CHOPCONF);
  chopconf &= ~(0x0F << 24);  // Clear MRES bits

  // Convert microsteps to MRES register value
  uint8_t mres;
  switch (microsteps) {
    case 256: mres = 0x0; break;  // 1/256 microstepping
    case 128: mres = 0x1; break;  // 1/128 microstepping
    case 64:  mres = 0x2; break;  // 1/64 microstepping
    case 32:  mres = 0x3; break;  // 1/32 microstepping
    case 16:  mres = 0x4; break;  // 1/16 microstepping
    case 8:   mres = 0x5; break;  // 1/8 microstepping
    case 4:   mres = 0x6; break;  // 1/4 microstepping
    case 2:   mres = 0x7; break;  // 1/2 microstepping (half step)
    case 1:   mres = 0x8; break;  // Full step
    default:  mres = 0x4; break;  // Default to 1/16 microstepping
  }

  chopconf |= (mres << 24);  // Set MRES bits
  chopconf |= (1 << 28);     // Enable interpolation
  writeRegister(CHOPCONF, chopconf);

  // Update global calculation variables
  STEPS_PER_REVOLUTION = STEPS_PER_REV * microsteps;
  currentAngleTolerance = calculateAngleTolerance(microsteps);

  Serial.print("Microstepping set to 1/");
  Serial.println(microsteps);
}

// Move motor a specified number of steps in given direction
void moveMotor(bool direction, uint32_t steps) {
  digitalWrite(DIR_PIN, direction);  // Set direction pin
  static unsigned long lastStepTime = 0;
  static uint32_t stepsRemaining = 0;

  // Initialize step counter on first call
  if (stepsRemaining == 0) {
    stepsRemaining = steps;
    lastStepTime = millis();
  }

  // Generate step pulses at controlled speed
  if (stepsRemaining > 0 && millis() - lastStepTime >= (1000 / currentSpeed)) {
    digitalWrite(STEP_PIN, HIGH);  // Start step pulse
    delayMicroseconds(10);         // Short pulse duration
    digitalWrite(STEP_PIN, LOW);   // End step pulse
    stepsRemaining--;              // Decrement remaining steps
    lastStepTime = millis();       // Update timing
  }
}

// Write data to TMC2240 register via SPI
void writeRegister(uint8_t address, uint32_t value) {
  digitalWrite(CS_PIN, LOW);                    // Select TMC2240
  SPI.transfer(address | 0x80);                 // Send address with write bit
  SPI.transfer((value >> 24) & 0xFF);          // Send MSB
  SPI.transfer((value >> 16) & 0xFF);          // Send byte 2
  SPI.transfer((value >> 8) & 0xFF);           // Send byte 1
  SPI.transfer(value & 0xFF);                  // Send LSB
  digitalWrite(CS_PIN, HIGH);                   // Deselect TMC2240
}

// Read data from TMC2240 register via SPI
uint32_t readRegister(uint8_t address) {
  digitalWrite(CS_PIN, LOW);                    // Select TMC2240
  SPI.transfer(address & 0x7F);                 // Send address with read bit
  uint32_t value = 0;
  value |= (uint32_t)SPI.transfer(0) << 24;    // Read MSB
  value |= (uint32_t)SPI.transfer(0) << 16;    // Read byte 2
  value |= (uint32_t)SPI.transfer(0) << 8;     // Read byte 1
  value |= (uint32_t)SPI.transfer(0);          // Read LSB
  digitalWrite(CS_PIN, HIGH);                   // Deselect TMC2240
  return value;
}

// Update cached sensor values (called periodically to reduce I2C/SPI traffic)
void updateCachedValues() {
  // Read and convert encoder angle from raw value to degrees
  cachedCurrentAngle = (as5600.readAngle() * 360.0) / 4096.0;

  // Read TMC2240 temperature sensor with filtering
  uint32_t adc_temp = readRegister(ADC_TEMP);
  int temp_raw = adc_temp & 0x1FFF;           // Extract temperature bits
  float temp_celsius = (temp_raw - 2000) * 0.13; // Convert to Celsius

  // Only update temperature if reading is within reasonable range
  if (temp_celsius >= -50 && temp_celsius <= 150) {
    cachedTemperature = temp_celsius;
  }

  // Read motor current from TMC2240
  uint32_t mscuract = readRegister(MSCURACT);
  cachedCurrentB = mscuract & 0x01FF;  // Extract current bits
}

// Automatically move motor to target angle with precision control
void moveToAngle() {
  if (!isMovingToAngle) return;  // Exit if not in angle movement mode

  // Calculate angular difference to target
  float angleDifference = targetAngle - cachedCurrentAngle;

  // Normalize angle difference to [-180, 180] range for shortest path
  while (angleDifference > 180) angleDifference -= 360;
  while (angleDifference < -180) angleDifference += 360;

  // Check if target angle is reached within tolerance
  if (abs(angleDifference) <= currentAngleTolerance) {
    isMovingToAngle = false;        // Stop angle movement mode
    digitalWrite(STEP_PIN, LOW);    // Ensure step pin is low
    Serial.println("Target angle reached");
    return;
  }

  // Set motor direction based on angle difference
  currentDirection = (angleDifference > 0);  // Positive = forward
  digitalWrite(DIR_PIN, currentDirection);

  // Enable motor if not already enabled
  if (!isMotorEnabled) {
    digitalWrite(EN_PIN, LOW);  // Enable motor (active low)
    isMotorEnabled = true;
  }

  // Dynamic speed control based on remaining angle
  int stepDelay;
  float absDiff = abs(angleDifference);

  // Slower speeds as we approach target for better precision
  if (absDiff <= 5.0) {
    stepDelay = MIN_STEP_DELAY * 4;      // Very slow for final approach
  } else if (absDiff <= 20.0) {
    stepDelay = MIN_STEP_DELAY * 2;      // Slow for precision zone
  } else if (absDiff <= 45.0) {
    stepDelay = MIN_STEP_DELAY;          // Normal speed
  } else {
    stepDelay = MIN_STEP_DELAY / 2;      // Fast for large movements
  }

  // Take multiple steps toward target (up to 10 per loop iteration)
  for (int i = 0; i < 10 && isMovingToAngle; i++) {
    digitalWrite(STEP_PIN, HIGH);        // Start step pulse
    delayMicroseconds(stepDelay);        // Pulse high time
    digitalWrite(STEP_PIN, LOW);         // End step pulse
    delayMicroseconds(stepDelay);        // Pulse low time

    // Update current angle reading after each step
    cachedCurrentAngle = (as5600.readAngle() * 360.0) / 4096.0;
    angleDifference = targetAngle - cachedCurrentAngle;
    
    // Re-normalize angle difference
    while (angleDifference > 180) angleDifference -= 360;
    while (angleDifference < -180) angleDifference += 360;

    // Check if target reached during movement
    if (abs(angleDifference) <= currentAngleTolerance) {
      isMovingToAngle = false;
      Serial.println("Target angle reached during movement");
      break;
    }
  }
}

// Calculate angle tolerance based on microstepping resolution
float calculateAngleTolerance(uint8_t microsteps) {
  // Calculate minimum angle step and multiply by 2 for tolerance
  return (360.0 / (200.0 * microsteps)) * 2.0;
}