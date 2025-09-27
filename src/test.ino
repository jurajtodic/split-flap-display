#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// --- Wi-Fi & Time Configuration ---
const char* ssid = "JurajevMob";
const char* password = "juki1234";
WiFiServer server(80);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7200); 
int lastDisplayedSecond = -1;

// --- System Configuration ---
const int NUM_STEPPERS = 8;
const int NUM_SHIFT_REGISTERS = 4;
const int DATA_PIN = 23;
const int CLOCK_PIN = 18;
const int LATCH_PIN = 4;

// --- Stepper Motor & Flap Properties ---
const int STEPS_PER_REVOLUTION = 2048;
const int NUM_FLAPS = 36;
const long BASE_STEPS_PER_FLAP = STEPS_PER_REVOLUTION / NUM_FLAPS;

// --- Behavior Configuration ---
const int MOVES_FOR_RECALIBRATION_WORD_MODE = 2;
const int NUDGE_STEPS = 35;
const long RECALIBRATION_TIMEOUT_STEPS = STEPS_PER_REVOLUTION * 2;
const int BLIND_SPIN_STEPS = 300; 

// --- Speed & Ramping Configuration ---
const int MAX_SPEED_DELAY_US = 2200;
const int START_STOP_DELAY_US = 2200;
const int RECALIBRATE_DELAY_US = 2200;
const int RAMP_STEPS = 300;

// --- Character Mapping ---
const char letterMap[] = {
    'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', '9', '8', '7',
    '6', '5', '4', '3', '2', '1', '0', 'A', 'B', 'C', 'D', 'E',
    'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', ':', 'P', 'Q'
};
const char tensDigitMap[] = {
    '4', '5', '0', '1', '2', '3', '4', '5', '0', '1', '2', '3',
    '4', '5', '0', '1', '2', '3', '4', '5', '0', '1', '2', '3',
    '4', '5', '0', '1', '2', '3', '4', '5', '0', '1', '2', '3'
};
const char unitsDigitMap[] = {
    '8', '9', '0', '1', '2', '3', '4', '5', '6', '7',
    '8', '9', '0', '1', '2', '3', '4', '5', '6', '7',
    '8', '9', '0', '1', '2', '3', '4', '5', '6', '7',
    '8', '9', '0', '1', '2', '3'
};

// --- Low-Level Control ---
const byte step_patterns[] = {0b1001, 0b1100, 0b0110, 0b0011};
byte output_buffer[NUM_SHIFT_REGISTERS];

// --- System State Machine ---
enum DisplayMode { WORD_MODE, CLOCK_MODE };
DisplayMode currentMode = WORD_MODE;

enum MotorActivity {
    IDLE, RECALIBRATING_START, RECALIBRATING_BLIND_SPIN, RECALIBRATING_FIND_HOME, RECALIBRATING_FIND_EDGE,
    MOVING_TO_TARGET, NUDGING_FORWARD, NUDGING_BACKWARD, HALTED
};

struct MotorState {
    const int hall_pin;
    MotorActivity state = RECALIBRATING_START;
    long current_pos = 0;
    long target_pos = 0;
    int phase = 0;
    unsigned long last_step_time_us = 0;
    int current_step_delay_us = START_STOP_DELAY_US;
    long steps_in_current_move = 0;
    long total_steps_for_move = 0;
    int nudge_step_counter = 0;
    int moves_since_recalibration = 0; // Only for WORD_MODE
    char post_recal_target_char = '\0';
    long recalibration_step_counter = 0;

    // --- "LAP TIMER" VARIABLES ---
    long steps_since_last_home = 0;
    long position_error = 0;
    // This flag is our "memory" of the sensor's previous state to detect a fresh trigger.
    bool home_sensor_active = false; 

    MotorState(int pin) : hall_pin(pin) {}
};

MotorState motors[NUM_STEPPERS] = {
  {36}, {39}, {34}, {35}, {32}, {33}, {17}, {16}
};

void setTargetWord(String word);

// --- Core Functions ---
void updateShiftRegisters(bool releaseTorque) {
    if (releaseTorque) { memset(output_buffer, 0, sizeof(output_buffer)); }
    digitalWrite(LATCH_PIN, LOW);
    for (int i = NUM_SHIFT_REGISTERS - 1; i >= 0; i--) { SPI.transfer(output_buffer[i]); }
    digitalWrite(LATCH_PIN, HIGH);
}

void stepMotor(int motorIndex, bool forward) {
    motors[motorIndex].phase = (motors[motorIndex].phase + (forward ? -1 : 1) + 4) % 4;
    motors[motorIndex].current_pos += (forward ? 1 : -1);
    byte pattern = step_patterns[motors[motorIndex].phase];
    int start_bit_pos = motorIndex * 4;
    int start_byte_index = start_bit_pos / 8;
    int bit_shift_in_byte = start_bit_pos % 8;
    
    output_buffer[start_byte_index] &= ~(0b1111 << bit_shift_in_byte);
    if (bit_shift_in_byte > 4 && start_byte_index + 1 < NUM_SHIFT_REGISTERS) {
         output_buffer[start_byte_index + 1] &= ~(0b1111 >> (8-bit_shift_in_byte));
    }
    output_buffer[start_byte_index] |= (pattern << bit_shift_in_byte);
    if (bit_shift_in_byte > 4 && start_byte_index + 1 < NUM_SHIFT_REGISTERS) {
        output_buffer[start_byte_index + 1] |= (pattern >> (8 - bit_shift_in_byte));
    }

    // --- "LAP TIMER" LOGIC ---
    motors[motorIndex].steps_since_last_home++;
    
    // Check the sensor's state RIGHT NOW. (LOW means active)
    bool is_active = (digitalRead(motors[motorIndex].hall_pin) == LOW);
    
    // Check for the LEADING EDGE: Was the sensor OFF before, but is ON now?
    if (is_active && !motors[motorIndex].home_sensor_active) {
        // Lap complete! Calculate the error from the ideal revolution size.
        motors[motorIndex].position_error = motors[motorIndex].steps_since_last_home - STEPS_PER_REVOLUTION;
        Serial.printf("Motor %d: LAP! %ld steps. Error: %ld\n", motorIndex, motors[motorIndex].steps_since_last_home, motors[motorIndex].position_error);
        
        // Reset the lap counter for the next revolution.
        motors[motorIndex].steps_since_last_home = 0;
    }
    
    // Update the "memory" for the next step's comparison.
    motors[motorIndex].home_sensor_active = is_active;
}

// --- Main Action Functions ---
void setTargetWord(String word) {
    for (int i = 0; i < NUM_STEPPERS; i++) {
        if (motors[i].state == RECALIBRATING_BLIND_SPIN || motors[i].state == RECALIBRATING_FIND_HOME || motors[i].state == RECALIBRATING_FIND_EDGE) {
            continue;
        }
        if (motors[i].state == HALTED) continue;

        if (motors[i].position_error != 0) {
            motors[i].current_pos -= motors[i].position_error; // Apply correction
            Serial.printf("Motor %d: Applying correction of %ld steps.\n", i, -motors[i].position_error);
            motors[i].position_error = 0; // Clear error after use
        }

        char targetLetter = (i < word.length()) ? toupper(word[i]) : ' ';
        if (targetLetter == 'O') targetLetter = '0';

        long current_flap_pos = (motors[i].current_pos % STEPS_PER_REVOLUTION) / BASE_STEPS_PER_FLAP;
        long flaps_to_move = 0;
        int targetIndex = -1;

        if (currentMode == WORD_MODE) {
            if (i >= 6) targetLetter = ' ';

            for (int j = 0; j < NUM_FLAPS; j++) { if (letterMap[j] == targetLetter) { targetIndex = j; break; } }
            if (targetIndex == -1) continue;

            motors[i].moves_since_recalibration++;
            if (motors[i].moves_since_recalibration >= MOVES_FOR_RECALIBRATION_WORD_MODE) {
                motors[i].post_recal_target_char = targetLetter;
                motors[i].state = RECALIBRATING_START;
                continue;
            }
            flaps_to_move = (targetIndex - current_flap_pos + NUM_FLAPS) % NUM_FLAPS;
        } else { // CLOCK_MODE
            const char* currentMap;
            int numUniqueFlaps;

            if (i < 6) { currentMap = letterMap; numUniqueFlaps = NUM_FLAPS; }
            else if (i == 6) { currentMap = tensDigitMap; numUniqueFlaps = 6; }
            else { currentMap = unitsDigitMap; numUniqueFlaps = 10; }

            for (int j = 0; j < NUM_FLAPS; j++) { if (currentMap[j] == targetLetter) { targetIndex = j; break; } }
            if (targetIndex == -1) continue;
            
            if (i < 6) {
                 flaps_to_move = (targetIndex - current_flap_pos + NUM_FLAPS) % NUM_FLAPS;
            } else {
                long current_flap_unique_pos = current_flap_pos % numUniqueFlaps;
                long target_flap_unique_pos = targetIndex % numUniqueFlaps;
                flaps_to_move = (target_flap_unique_pos - current_flap_unique_pos + numUniqueFlaps) % numUniqueFlaps;

                if (flaps_to_move == 0 && i == 7) { 
                    flaps_to_move = numUniqueFlaps; 
                }
            }
        }

        if (flaps_to_move > 0) {
            long steps_to_move = flaps_to_move * BASE_STEPS_PER_FLAP;
            motors[i].target_pos = motors[i].current_pos + steps_to_move;
            motors[i].total_steps_for_move = steps_to_move;
            motors[i].steps_in_current_move = 0;
            motors[i].state = MOVING_TO_TARGET;
        } else {
            motors[i].state = IDLE;
        }
    }
}

void recalibrateAllMotors() {
    Serial.println("Web request received: Recalibrating all motors.");
    for (int i = 0; i < NUM_STEPPERS; i++) {
        if (motors[i].state != HALTED) {
            motors[i].post_recal_target_char = '\0';
            motors[i].state = RECALIBRATING_START;
        }
    }
}

// --- State Machine ---
void updateMotor(int motorIndex) {
    MotorState &motor = motors[motorIndex];
    if (motor.state == IDLE || motor.state == HALTED) { return; }
    if (micros() - motor.last_step_time_us < motor.current_step_delay_us) { return; }
    motor.last_step_time_us = micros();

    switch (motor.state) {
        case RECALIBRATING_START:
            motor.current_step_delay_us = RECALIBRATE_DELAY_US;
            motor.recalibration_step_counter = 0;
            motor.state = RECALIBRATING_BLIND_SPIN;
            break;

        case RECALIBRATING_BLIND_SPIN:
            if (motor.recalibration_step_counter >= BLIND_SPIN_STEPS) {
                motor.recalibration_step_counter = 0;
                motor.state = RECALIBRATING_FIND_HOME;
                break;
            }
            stepMotor(motorIndex, true);
            motor.recalibration_step_counter++;
            break;

        case RECALIBRATING_FIND_HOME:
            stepMotor(motorIndex, true); motor.recalibration_step_counter++;
            if (digitalRead(motor.hall_pin) == HIGH) {
                motor.state = RECALIBRATING_FIND_EDGE;
            }
            if (motor.recalibration_step_counter > RECALIBRATION_TIMEOUT_STEPS) {
                motor.state = HALTED;
            }
            break;

        case RECALIBRATING_FIND_EDGE:
            stepMotor(motorIndex, true);
            motor.recalibration_step_counter++;
            if (digitalRead(motor.hall_pin) == LOW) {
                motor.current_pos = 0; motor.target_pos = 0;
                motor.moves_since_recalibration = 0;

                // After recalibrating, reset the lap timer too for a clean start.
                motors[motorIndex].steps_since_last_home = 0;
                motors[motorIndex].position_error = 0;
                motors[motorIndex].home_sensor_active = true; // Sensor is active now

                if (motor.post_recal_target_char != '\0') {
                    char tempTarget = motor.post_recal_target_char;
                    motor.post_recal_target_char = '\0';
                    String word_to_set = "";
                    for(int i=0; i<NUM_STEPPERS; i++) { word_to_set += (i==motorIndex) ? tempTarget : ' '; }
                    setTargetWord(word_to_set);
                } else {
                    motor.state = IDLE;
                }
            }
            if (motor.recalibration_step_counter > RECALIBRATION_TIMEOUT_STEPS) {
                 motor.state = HALTED;
            }
            break;
            
        case MOVING_TO_TARGET:
            if (motor.current_pos >= motor.target_pos) {
                motor.nudge_step_counter = 0; motor.state = NUDGING_FORWARD; break;
            }
            if (motor.steps_in_current_move < RAMP_STEPS) {
                motor.current_step_delay_us = START_STOP_DELAY_US - ((START_STOP_DELAY_US - MAX_SPEED_DELAY_US) * motor.steps_in_current_move / RAMP_STEPS);
            } else if (motor.steps_in_current_move >= (motor.total_steps_for_move - RAMP_STEPS)) {
                long decel_step = motor.steps_in_current_move - (motor.total_steps_for_move - RAMP_STEPS);
                motor.current_step_delay_us = MAX_SPEED_DELAY_US + ((START_STOP_DELAY_US - MAX_SPEED_DELAY_US) * decel_step / RAMP_STEPS);
            } else { motor.current_step_delay_us = MAX_SPEED_DELAY_US; }
            stepMotor(motorIndex, true); motor.steps_in_current_move++;
            break;
        case NUDGING_FORWARD:
            motor.current_step_delay_us = START_STOP_DELAY_US;
            if (motor.nudge_step_counter >= NUDGE_STEPS) {
                motor.nudge_step_counter = 0; motor.state = NUDGING_BACKWARD; break;
            }
            stepMotor(motorIndex, true); motor.nudge_step_counter++;
            break;
        case NUDGING_BACKWARD:
            motor.current_step_delay_us = START_STOP_DELAY_US;
            if (motor.nudge_step_counter >= NUDGE_STEPS) {
                motor.state = IDLE; break;
            }
            stepMotor(motorIndex, false); motor.nudge_step_counter++;
            break;
    }
}

// --- Wi-Fi & Mode Functions ---
void setupWiFi() {
  Serial.print("Connecting to "); Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: "); Serial.println(WiFi.localIP());
  server.begin();
  timeClient.begin();
  timeClient.update();
  Serial.println("Web server and NTP client started.");
}

void handleClient() {
  WiFiClient client = server.available();
  if (!client) { return; }

  String request = client.readStringUntil('\r');
  client.flush();

  if (request.indexOf("GET /recalibrate") != -1) {
    recalibrateAllMotors();
  } else if (request.indexOf("GET /set_clock_mode") != -1) {
    currentMode = CLOCK_MODE;
    timeClient.update();
    lastDisplayedSecond = -1; 
  } else if (request.indexOf("GET /set_word_mode") != -1) {
    currentMode = WORD_MODE;
  } else if (request.indexOf("/move?word=") != -1) {
    int startIndex = request.indexOf('=') + 1;
    int endIndex = request.indexOf(' ', startIndex);
    String targetWord = request.substring(startIndex, endIndex);
    targetWord.replace("%20", " ");
    setTargetWord(targetWord);
  }
  
  client.println("HTTP/1.1 200 OK");
  client.println("Content-type:text/html");
  client.println();
  client.println("<html><head><title>Split-Flap Control</title><style>body{font-family: sans-serif; text-align: center;} form,div{margin-bottom: 20px;}</style></head><body><h1>Split-Flap Control</h1>");
  
  if (currentMode == WORD_MODE) {
    client.println("<div>Current Mode: <b>Word Display</b></div>");
    client.printf("<form action='/move' method='GET'>");
    // Updated to 6 characters
    client.printf("Enter up to 6 characters: <input type='text' name='word' maxlength='6' style='padding: 5px; text-transform: uppercase;'><br><br>");
    client.println("<input type='submit' value='Display Word' style='padding: 10px;'>");
    client.println("</form>");
    client.println("<form action='/set_clock_mode' method='GET'><input type='submit' value='Switch to Clock Mode' style='padding: 10px;'></form>");
  } else { // CLOCK_MODE
    client.println("<div>Current Mode: <b>Clock</b></div>");
    // Updated to show full time, including note about UTC
    client.printf("<div>Current Time (UTC): %s</div>", timeClient.getFormattedTime().c_str());
    client.println("<form action='/set_word_mode' method='GET'><input type='submit' value='Switch to Word Mode' style='padding: 10px;'></form>");
  }
  
  client.println("<hr style='margin-top: 30px;'>");
  client.println("<form action='/recalibrate' method='GET'>");
  client.println("<input type='submit' value='Recalibrate All Motors' style='padding: 10px; background-color: #f44336; color: white; border: none; margin-top: 10px;'>");
  client.println("</form>");
  client.println("</body></html>");
  delay(1);
  client.stop();
}

void loopWordMode() { /* Word mode is entirely driven by web commands */ }

void loopClockMode() {
    timeClient.update();
    int currentSecond = timeClient.getSeconds();
    if (currentSecond != lastDisplayedSecond) {
        lastDisplayedSecond = currentSecond;
        char timeString[9]; 
        sprintf(timeString, "%02d:%02d:%02d", timeClient.getHours(), timeClient.getMinutes(), timeClient.getSeconds());
        setTargetWord(String(timeString));
    }
}

// --- Main Program ---
void setup() {
    Serial.begin(115200);
    Serial.println("--- Dual Mode Split-Flap Display ---");
    
    // First, connect to WiFi
    setupWiFi();
    
    // Second, initialize pins and SPI
    pinMode(LATCH_PIN, OUTPUT);
    digitalWrite(LATCH_PIN, HIGH);
    for (int i = 0; i < NUM_STEPPERS; i++) {
        pinMode(motors[i].hall_pin, INPUT_PULLUP);
    }
    SPI.begin();
    SPI.setFrequency(4000000);
    
    // Force time synchronization BEFORE calibrating
    Serial.print("Waiting for first NTP time sync...");
    // The getEpochTime() will be very low (near 0) until the first sync has completed.
    // An Epoch time less than 1 billion is a reliable way to check if it's a real, modern time.
    while (timeClient.getEpochTime() < 1000000000) {
      timeClient.update();
      Serial.print(".");
      delay(1000);
    }
    Serial.println("\nTime has been synchronized!");

    // Finally, now that we have the correct time, recalibrate all motors
    // This will run AFTER the initial RECALIBRATING_START state is set in the MotorState struct.
    recalibrateAllMotors();
}

void loop() {
    handleClient(); // Make sure your full handleClient function is here
    if (currentMode == WORD_MODE) {
        loopWordMode();
    } else {
        loopClockMode();
    }
    
    bool any_motor_is_moving = false;
    memset(output_buffer, 0, sizeof(output_buffer));
    for (int i = 0; i < NUM_STEPPERS; i++) {
        updateMotor(i);
        if (motors[i].state != IDLE && motors[i].state != HALTED) {
            any_motor_is_moving = true;
            byte pattern = step_patterns[motors[i].phase];
            int start_bit_pos = i * 4;
            int start_byte_index = start_bit_pos / 8;
            int bit_shift_in_byte = start_bit_pos % 8;

            output_buffer[start_byte_index] |= (pattern << bit_shift_in_byte);
            if (bit_shift_in_byte > 4 && start_byte_index + 1 < NUM_SHIFT_REGISTERS) {
                output_buffer[start_byte_index + 1] |= (pattern >> (8 - bit_shift_in_byte));
            }
        }
    }
    if (any_motor_is_moving) {
        updateShiftRegisters(false);
    } else {
        updateShiftRegisters(true);
        delay(20);
    }
}