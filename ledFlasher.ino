#include <EEPROM.h>
//const byte EEPROM_START = 0;
const int SERIAL_BAUDRATE = 9600;
const byte MAX_LED_ON = 10; //10ms hopefully
const byte HOLD_CAMERA_PIN_UP = 30;
const byte FLASH_SEQUNCE_SIZE = 8; //4 delays, 4 ons,
const byte EEPROM_SETTINGS_OFFSET = 10;
const byte EEPROM_TRIGGER_LEVEL_HIGH = EEPROM_SETTINGS_OFFSET + 1;
const byte EEPROM_TRIGGER_LEVEL_LOW = EEPROM_SETTINGS_OFFSET + 2;
const byte SIMULATION_LED = 13;
const int SIMULATION_MULTIPLIER = 200;

const unsigned int TRIGGER_LEVEL = 1024;

//Pins
const byte LED_PIN = 10;

byte[FLASH_SEQUNCE_SIZE] flashSequence;

//Settings
bit isAnalog;
bit isThresholdMode;
bit isTriggerAboveLimit;
byte port;
unsigned int triggerLevel;
unsigned int startValue;

void setup()
{
    readDefaultsFromEeprom();
    updateTriggering();
    initOutputs();
    initInputs();
    initSerial();

}

void initInputs() {
    if (isAnalog && isThresholdMode) {
        startValue = readAnalog(port);
    }
}

void initOutputs()
{
    pinMode(LED_PIN, OUTPUT);
}

void loop()
{
    readSerial();

    if (shouldTriggerLedFlash())
    {
        triggerCamera()
        triggerFlashing();
    }
}

boolean shouldTriggerLedFlash()
{
    if (isFiredOnce)
    {
        return false;
    }
    if (readTrigger())
    {
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
        return readDigital(port) == HIGH;
    }
}

void triggerCamera()
{
    digitalWrite(CAMERA_PIN, HIGH);
    waitFor(HOLD_CAMERA_PIN_UP);
    digitalWrite(CAMERA_PIN, LOW);
}

void triggerFlashing()
{
    for (byte i = 0; i < FLASH_SEQUNCE_SIZE; i++ )
    {
        int time = flashSequence[i];
        if (i % 2 == 0)
        {
            waitFor(time);
        }
        else
        {
            turnLedOn(time);
        }
    }
}

void turnLedOn(int time)
{
    if (time > MAX_LED_ON)
    {
        time = 1;
    }
    digitalWrite(LED_PIN, HIGH);
    waitFor(time);
    digitalWrite(LED_PIN, LOW);
}

void readDefaultsFromEeprom()
{
    for (byte i = 0; i < FLASH_SEQUNCE_SIZE; i++)
    {
        flashSequence {i} = EEPROM.read(i);
    }
}

void updateFlashSequence(byte[] newFlashSequnce)
{
    for (byte i = 0; i < FLASH_SEQUNCE_SIZE; i++)
    {
        if (newFlashSequnce {i} != flashSequence {i})
        {
            EEPROM.write(i, flashSequence {i});
        }
    }
    flashSequence = newFlashSequnce;
}

void updateTriggering()
{
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
    if ( combined > MAX_TRIGGER_LEVEL) {
        combined = MAX_TRIGGER_LEVEL;
    }
    triggerLevel = combined;
}
void updateSensorConfig()
{
    //TODO Sensor port & trigger level
}

void initSerial()
{
    Serial.begin(SERIAL_BAUDRATE);
}
void readSerial()
{
    while (Serial.available() > 0)
    {
        byte modeSelection = Serial.read();
        switch (modeSelection)
        {
        case 0xFA:
            readFlashSequenceFromSerial();
            break;
        case 0xFB:
            readSensorConfigureFromSerial();
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

//    Sensor configure
// 2.byte:
//     -1bit Analog/digital mode
//     -1bit Threshold/limit mode
//     -1bit Below/above limit trigger
//     -1bit Reserved
//     -4bit PORT number - use all bits!

// 3-4. bytes 1024 Trigger level (only needs 11bits)



//SAVE SETTINGS TO EEPROM
void readFlashSequenceFromSerial()
{
    byte[FLASH_SEQUNCE_SIZE] buffer;
    byte readBytes = Serial.readBytesUntil(0x00, buffer, FLASH_SEQUNCE_SIZE );

    if (readBytes > 2)
    {
        updateFlashSequence(readBytes);
    }
}
void readSensorConfigureFromSerial()
{
    byte[3] buffer;
    Serial.readBytes(buffer, 3);
    if ( !validateBuffer(buffer)) {
        return;
    }
    saveToEEPROM();
}

//Unused bits should be zero
boolean validateBuffer(byte[] buffer) {
    return bitRead(buffer[0], 1) == 0
           && bitRead(buffer[1], 7) == 0
           && bitRead(buffer[1], 6) == 0
           && bitRead(buffer[1], 5) == 0
}
void runSimulatedLedShow()
{
    blinkSimulation();
    simulateLedSequence();
    blinkSimulation();
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


unsigned int bitShiftCombine( byte high, byte low)
{
    unsigned int combined = high;
    combined = combined << 8;
    combined |= low;
    return combined;
}