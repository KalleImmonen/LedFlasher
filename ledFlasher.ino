#include <EEPROM.h>
//const byte EEPROM_START = 0;
const byte MAX_LED_ON = 10; //10ms hopefully
const byte HOLD_CAMERA_PIN_UP = 30;
const byte FLASH_SEQUNCE_SIZE = 8; //4 delays, 4 ons,

byte {FLASH_SEQUNCE_SIZE} flashSequence;
void setup()
{
    readDefaultsFromEeprom();
    initOutputs();
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
    if (read_analog() > TRIGGER_LEVEL)
    {
        isFiredOnce = true;
        return true;
    }
    return false;
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
    byte i;
    for (byte i = 0; i < FLASH_SEQUNCE_SIZE; i++)
    {
        flashSequence {i} = EEPROM.read(i);
    }
}

void updateFlashSequence(byte {} newFlashSequnce)
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
void updateSensorConfig()
{
    //TODO Sensor port & trigger level
}

void readSerial()
{

    //Pulses definion: set of onOff
    /**
    1. byte: MODE SELECTION: Flash configure(0xFA),Sensor configure(0xFB)

    **Flash configures
    2. byte: delay after signal < 1000ms
    3. byte: light on time < 10ms
    4. byte: delay delay after next < 1000ms
    5. byte: light on time < 10ms
    IF 0x00 sent - means end of message

    **Sensor configure
    2.byte:
        -1bit Analog/digital mode
        -1bit Threshold/limit mode
        -1bit Below/above limit trigger
        -1bit Reserved
        -4bit PORT number - use all bits!

    3-4. bytes 1024 Trigger level (only needs 11bits)


    **/

    //SAVE SETTINGS TO EEPROM
}

