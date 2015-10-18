#include <EEPROM.h>

//const byte EEPROM_START = 0;
const int SERIAL_BAUDRATE = 9600;
const byte MAX_LED_ON = 10; //10ms hopefully
const byte HOLD_CAMERA_PIN_UP = 30;
const byte FLASH_SEQUNCE_SIZE = 8; //4 delays, 4 on's,
const byte CONFIG_SIZE = 3; //4 delays, 4 on's,
const byte EEPROM_SETTINGS_OFFSET = 10;
const byte EEPROM_TRIGGER_LEVEL_HIGH = EEPROM_SETTINGS_OFFSET + 1;
const byte EEPROM_TRIGGER_LEVEL_LOW = EEPROM_SETTINGS_OFFSET + 2;
const byte SIMULATION_LED = 13;
const byte CONFIG_START_INDEX = 50;
const int SIMULATION_MULTIPLIER = 2;
const unsigned int MAX_TRIGGER_LEVEL = 1023;

//Pins
const int CAMERA_PIN = 4; //TODO: Config this.
const int LED_PIN = 10;

byte flashSequence[FLASH_SEQUNCE_SIZE];
byte configSequence[CONFIG_SIZE];

//Settings
boolean isAnalog;
boolean isThresholdMode;
boolean isTriggerAboveLimit;
boolean isFiredOnce = false;
byte port;
unsigned int triggerLevel;
unsigned int startValue;

void setup() {
    readDefaultsFromEeprom();
    updateTriggering();
    initOutputs();
    initInputs();
    initSerial();

}

int readAnalog(byte port) {
    return analogRead(port);
}

boolean readDigital(byte port) {
    return digitalRead(port) == HIGH;
}

void initInputs() {
    if (isAnalog && isThresholdMode) {
        startValue = readAnalog(port);
    }
}

void initOutputs() {
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    readSerial();

    if (shouldTriggerLedFlash()) {
        triggerImageTaking();
    }
}

void triggerImageTaking() {
    triggerCamera();
    triggerFlashing();
}

boolean shouldTriggerLedFlash() {
    if (isFiredOnce) {
        return false;
    }
    if (readTrigger()) {
        isFiredOnce = true;
        return true;
    }
    return false;
}

boolean readTrigger() {
    if (isAnalog) {
        int readOut = readAnalog(port);
        if (isThresholdMode) {
            // start value vs current value has diff
            return (startValue - readOut) > triggerLevel;
        } else {
            //current value bigger/smaller than trigger level
            if (isTriggerAboveLimit) {
                return readOut < triggerLevel;
            } else {
                return readOut > triggerLevel;
            }
        }
    } else {
        return readDigital(port);
    }
}

void waitFor(int time) {
    delay(time);
}

void triggerCamera() {
    digitalWrite(CAMERA_PIN, HIGH);
    waitFor(HOLD_CAMERA_PIN_UP);
    digitalWrite(CAMERA_PIN, LOW);
}

void triggerFlashing() {
    for (byte i = 0; i < FLASH_SEQUNCE_SIZE; i++) {
        int time = flashSequence[i];
        if (i % 2 == 0) {
            waitFor(time);
        }
        else {
            turnLedOn(time);
        }
    }
}

void turnLedOn(int time) {
    if (time > MAX_LED_ON) {
        time = 1;
    }
    digitalWrite(LED_PIN, HIGH);
    waitFor(time);
    digitalWrite(LED_PIN, LOW);
}

void readDefaultsFromEeprom() {
    for (byte i = 0; i < FLASH_SEQUNCE_SIZE; i++) {
        flashSequence[i] = EEPROM.read(i);
    }

    for (byte i = CONFIG_START_INDEX; i < CONFIG_SIZE + CONFIG_START_INDEX; i++) {
        configSequence[i] = EEPROM.read(i);
    }
}

void updateFlashSequenceToEeprom(byte *newFlashSequence) {
    for (byte i = 0; i < FLASH_SEQUNCE_SIZE; i++) {
        if (newFlashSequence[i] != flashSequence[i]) {
            EEPROM.write(i, flashSequence[i]);
        }

        flashSequence[i] = newFlashSequence[i];
    }
}

//    Sensor configure
// 2.byte:
//     -1bit Analog/digital mode
//     -1bit Threshold/limit mode
//     -1bit Below/above limit trigger
//     -1bit Reserved
//     -4bit PORT number - use all bits!

// 3-4. bytes 1024 Trigger level (only needs 11bits)
void updateSensorConfigToEeprom(byte *configs) {
    for (byte i = CONFIG_START_INDEX; i < CONFIG_SIZE + CONFIG_START_INDEX; i++) {
        if (configSequence[i] != configs[i]) {
            EEPROM.write(i, configSequence[i]);
        }
        configSequence[i] = configs[i];
    }


    //TODO: Bit masking shifting
    isAnalog = configSequence[0] & 0x01 == 1;
    isThresholdMode = configSequence[0] & 0x02 == 1;
    isTriggerAboveLimit = configSequence[0] & 0x04 == 1;
    //isReserved = configSequence[0] && 0x08;
    port = configSequence[0] >> 4;

    int level = bitShiftCombine(configSequence[1], configSequence[2]);
    triggerLevel = level > MAX_TRIGGER_LEVEL ? MAX_TRIGGER_LEVEL : level;
}

void updateTriggering() {
    updateTriggerSettingsFromEeprom();
    updateTriggerLevelFromEeprom();
}

void updateTriggerSettingsFromEeprom() {
    byte settings = EEPROM.read(EEPROM_SETTINGS_OFFSET);
    isAnalog = bitRead(settings, 0);
    isThresholdMode = bitRead(settings, 1);
    isTriggerAboveLimit = bitRead(settings, 2);
    port = settings >> 4;
}

void updateTriggerLevelFromEeprom() {
    byte high = EEPROM.read(EEPROM_TRIGGER_LEVEL_HIGH);
    byte low = EEPROM.read(EEPROM_TRIGGER_LEVEL_LOW);
    unsigned int combined = bitShiftCombine(high, low);
    if (combined > MAX_TRIGGER_LEVEL) {
        combined = MAX_TRIGGER_LEVEL;
    }
    triggerLevel = combined;
}

void updateSensorConfig() {
    //TODO Sensor port & trigger level
    //Read configs
    //set IsAnalog/Digital, isThresholdLimitMode, portNumber  
}

void initSerial() {
    Serial.begin(SERIAL_BAUDRATE);
}

void readSerial() {
    while (Serial.available() > 0) {
        byte modeSelection = Serial.read();
        switch (modeSelection) {
            case 'a'://0xFA:
                readFlashSequenceFromSerial();
                break;
            case 0xFB:
                readSensorConfigureFromSerial();
                break;
            case 'b'://0xFC:
                runSimulatedLedShow();
                break;
            case 0xFD:
                triggerImageTaking();
                break;

            default:
                return;
        }
    }
}

//Pulses definion: set of onOff

//1. byte: MODE SELECTION: Flash configure(0xFA), Sensor configure(0xFB)

//   Flash configures
// 2. byte: delay after signal < 1000ms
// 3. byte: light on time < 10ms
// 4. byte: delay delay after next < 1000ms
// 5. byte: light on time < 10ms
// IF 0x00 sent - means end of message





//SAVE SETTINGS TO EEPROM
void readFlashSequenceFromSerial() {
    Serial.write("ReadFlash");
    byte buffer[FLASH_SEQUNCE_SIZE]; //TODO: Maybe to global variable, maybe leaking
    byte readBytes = Serial.readBytesUntil(0x00, (char *) buffer, FLASH_SEQUNCE_SIZE);

    if (readBytes > 2) {
        updateFlashSequenceToEeprom(&readBytes);
    }
}


void readSensorConfigureFromSerial() {
    byte buffer[3]; //TODO: Maybe to global variable, maybe leaking
    Serial.readBytes((char *) buffer, 3);
    if (!validateBuffer(buffer)) {
        return;
    }
    updateSensorConfigToEeprom(buffer);
}

//Unused bits should be zero
boolean validateBuffer(byte *buffer) {
    return bitRead(buffer[0], 1) == 0
           && bitRead(buffer[1], 7) == 0
           && bitRead(buffer[1], 6) == 0
           && bitRead(buffer[1], 5) == 0;
}

void runSimulatedLedShow() {
    //blinkSimulation();
    simulateLedSequence();
    //blinkSimulation();
}

void simulateLedSequence() {
    byte duration;
    for (byte i = 0; i < FLASH_SEQUNCE_SIZE; i++) {
        duration = flashSequence[i];
        if (i % 2) {
            turnSimulationLedOn(duration);
        } else {
            waitSimulationFor(duration);
        }
    }
}

void turnSimulationLedOn(int duration) {
    digitalWrite(SIMULATION_LED, HIGH);
    delay(duration * SIMULATION_MULTIPLIER);
    digitalWrite(SIMULATION_LED, LOW);
}

void waitSimulationFor(int duration) {
    delay(duration * SIMULATION_MULTIPLIER);
}

void blinkSimulation() {
    for (byte i = 0; i < 10; i++) {
        digitalWrite(SIMULATION_LED, HIGH);
        delay(SIMULATION_MULTIPLIER);
        digitalWrite(SIMULATION_LED, LOW);
        delay(SIMULATION_MULTIPLIER);
    }
}


byte bitShiftCombine(byte high, byte low) {
    byte combined = high;
    combined = combined << 8;
    combined |= low;
    return combined;
}