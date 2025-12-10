/**
 * Laser Turret Controller - XIAO ESP32S3 Sense
 * 
 * Streams camera frames over serial and receives servo/laser commands.
 * 
 * Pin Configuration:
 * - D0 (GPIO1): Pan servo (increase angle = move RIGHT)
 * - D1 (GPIO2): Tilt servo (increase angle = move DOWN)
 * - D2 (GPIO3): Laser control (HIGH = on)
 * 
 * Serial Protocol:
 * - Outgoing: Raw JPEG frames (start: 0xFFD8, end: 0xFFD9)
 * - Incoming: <P:angle,T:angle,L:0/1> (e.g., <P:90,T:45,L:1>)
 */

#include <Arduino.h>
#include <ESP32Servo.h>
#include "esp_camera.h"

// =============================================================================
// Pin Definitions - XIAO ESP32S3 Sense
// =============================================================================

// Servo and laser pins
#define PAN_SERVO_PIN   D0  // GPIO1
#define TILT_SERVO_PIN  D1  // GPIO2
#define LASER_PIN       D2  // GPIO3

// Camera pins for XIAO ESP32S3 Sense (OV2640)
// Reference: https://wiki.seeedstudio.com/xiao_esp32s3_camera_usage/
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   10
#define SIOD_GPIO_NUM   40
#define SIOC_GPIO_NUM   39
#define Y9_GPIO_NUM     48
#define Y8_GPIO_NUM     11
#define Y7_GPIO_NUM     12
#define Y6_GPIO_NUM     14
#define Y5_GPIO_NUM     16
#define Y4_GPIO_NUM     18
#define Y3_GPIO_NUM     17
#define Y2_GPIO_NUM     15
#define VSYNC_GPIO_NUM  38
#define HREF_GPIO_NUM   47
#define PCLK_GPIO_NUM   13

// =============================================================================
// Configuration
// =============================================================================

#define SERIAL_BAUD_RATE  921600

// Servo limits (degrees)
#define PAN_MIN   0
#define PAN_MAX   180
#define TILT_MIN  0
#define TILT_MAX  180

// Default positions (center)
#define PAN_CENTER  90
#define TILT_CENTER 90

// Command buffer
#define CMD_BUFFER_SIZE 64

// =============================================================================
// Global Variables
// =============================================================================

Servo panServo;
Servo tiltServo;

int currentPan = PAN_CENTER;
int currentTilt = TILT_CENTER;
bool laserOn = false;

char cmdBuffer[CMD_BUFFER_SIZE];
int cmdIndex = 0;
bool cmdStarted = false;

// =============================================================================
// Camera Initialization
// =============================================================================

bool initCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.frame_size = FRAMESIZE_QVGA;  // 320x240
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode = CAMERA_GRAB_LATEST;  // Always get latest frame
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 15;  // Good balance of quality and size
    config.fb_count = 1;  // Single buffer to reduce latency

    // Initialize camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        return false;
    }

    // Adjust camera settings for better image
    sensor_t *s = esp_camera_sensor_get();
    if (s != NULL) {
        s->set_brightness(s, 0);
        s->set_contrast(s, 0);
        s->set_saturation(s, 0);
        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);
        s->set_wb_mode(s, 0);
        s->set_exposure_ctrl(s, 1);
        s->set_aec2(s, 0);
        s->set_gain_ctrl(s, 1);
        s->set_agc_gain(s, 0);
        s->set_gainceiling(s, (gainceiling_t)0);
        s->set_bpc(s, 0);
        s->set_wpc(s, 1);
        s->set_raw_gma(s, 1);
        s->set_lenc(s, 1);
        s->set_hmirror(s, 0);
        s->set_vflip(s, 0);
        s->set_dcw(s, 1);
    }

    return true;
}

// =============================================================================
// Servo Control
// =============================================================================

void initServos() {
    // Allow allocation of all timers for servos
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    panServo.setPeriodHertz(50);
    tiltServo.setPeriodHertz(50);

    panServo.attach(PAN_SERVO_PIN, 500, 2500);
    tiltServo.attach(TILT_SERVO_PIN, 500, 2500);

    // Move to center position
    panServo.write(currentPan);
    tiltServo.write(currentTilt);
}

void setPan(int angle) {
    currentPan = constrain(angle, PAN_MIN, PAN_MAX);
    panServo.write(currentPan);
}

void setTilt(int angle) {
    currentTilt = constrain(angle, TILT_MIN, TILT_MAX);
    tiltServo.write(currentTilt);
}

// =============================================================================
// Laser Control
// =============================================================================

void initLaser() {
    pinMode(LASER_PIN, OUTPUT);
    digitalWrite(LASER_PIN, LOW);
    laserOn = false;
}

void setLaser(bool on) {
    laserOn = on;
    digitalWrite(LASER_PIN, on ? HIGH : LOW);
}

// =============================================================================
// Command Parsing
// =============================================================================

bool frameRequested = false;

void parseCommand(const char* cmd) {
    // Expected format: P:angle,T:angle,L:0/1,F:1
    // Example: P:90,T:45,L:1,F:1
    // F:1 requests a new frame
    
    int pan = currentPan;
    int tilt = currentTilt;
    int laser = laserOn ? 1 : 0;

    // Parse pan
    const char* pPtr = strstr(cmd, "P:");
    if (pPtr != NULL) {
        pan = atoi(pPtr + 2);
    }

    // Parse tilt
    const char* tPtr = strstr(cmd, "T:");
    if (tPtr != NULL) {
        tilt = atoi(tPtr + 2);
    }

    // Parse laser
    const char* lPtr = strstr(cmd, "L:");
    if (lPtr != NULL) {
        laser = atoi(lPtr + 2);
    }

    // Parse frame request
    const char* fPtr = strstr(cmd, "F:");
    if (fPtr != NULL && atoi(fPtr + 2) == 1) {
        frameRequested = true;
    }

    // Apply commands
    setPan(pan);
    setTilt(tilt);
    setLaser(laser != 0);
}

// processSerialInput is now inlined in loop() for better responsiveness

// =============================================================================
// Frame Capture and Send
// =============================================================================

// Frame start marker (unlikely to appear in JPEG data)
const uint8_t FRAME_START[4] = {0xAA, 0x55, 0xAA, 0x55};

void captureAndSendFrame() {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb == NULL) {
        return;
    }

    // Send frame with length-prefix protocol:
    // [4 bytes: start marker][4 bytes: length (little endian)][JPEG data]
    
    // Send start marker
    Serial.write(FRAME_START, 4);
    
    // Send frame length as 4 bytes (little endian)
    uint32_t len = fb->len;
    Serial.write((uint8_t)(len & 0xFF));
    Serial.write((uint8_t)((len >> 8) & 0xFF));
    Serial.write((uint8_t)((len >> 16) & 0xFF));
    Serial.write((uint8_t)((len >> 24) & 0xFF));
    
    // Send the JPEG data
    Serial.write(fb->buf, fb->len);

    esp_camera_fb_return(fb);
}

// =============================================================================
// Main Setup and Loop
// =============================================================================

void setup() {
    // Initialize serial at high baud rate
    Serial.begin(SERIAL_BAUD_RATE);
    
    // Wait for serial/USB connection to stabilize
    delay(2000);

    // Initialize components
    initLaser();
    initServos();

    // Initialize camera
    if (!initCamera()) {
        // Blink laser rapidly to indicate camera error (10 fast blinks)
        for (int i = 0; i < 10; i++) {
            setLaser(true);
            delay(100);
            setLaser(false);
            delay(100);
        }
        // Keep blinking slowly to indicate persistent error
        while (true) {
            setLaser(true);
            delay(500);
            setLaser(false);
            delay(500);
        }
    }

    // Two brief laser flashes to indicate ready
    for (int i = 0; i < 2; i++) {
        setLaser(true);
        delay(150);
        setLaser(false);
        delay(150);
    }
}

void loop() {
    // Process incoming serial data (may contain multiple characters)
    // Read all available data before checking frameRequested
    while (Serial.available() > 0) {
        char c = Serial.read();

        if (c == '<') {
            cmdStarted = true;
            cmdIndex = 0;
        } else if (c == '>' && cmdStarted) {
            cmdBuffer[cmdIndex] = '\0';
            parseCommand(cmdBuffer);
            cmdStarted = false;
            cmdIndex = 0;
        } else if (cmdStarted && cmdIndex < CMD_BUFFER_SIZE - 1) {
            cmdBuffer[cmdIndex++] = c;
        }
    }

    // Send frame when Python requests one (reduces latency)
    if (frameRequested) {
        captureAndSendFrame();
        frameRequested = false;
    }
}

