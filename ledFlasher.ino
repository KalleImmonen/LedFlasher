// ledFlasher.ino

void setup()
{
    readDefaultsFromEeprom();
    initOutputs()
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
        triggerFlasing();
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
    waitFor(30)
    digitalWrite(CAMERA_PIN, LOW);
}
void triggerFlasing()
{
    byte size = flashSequence.length;
    for (byte i = 0; i < size; i++ )
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
    digitalWrite(LED_PIN, HIGH);
    waitFor(time);
    digitalWrite(LED_PIN, LOW);
}

void readDefaultsFromEeprom()
{
    read_from_eeprom();
}
void writeDefaultsToEeprom()
{
    //TODO Flash following:
    //Flashing sequnce
    //Sensor port & trigger level
}

void readSerial()
{
    //Pulses definion: set of onOff
    /**
    1. byte: delay after signal < 1000ms
    2. byte: light on time < 10ms
    3. byte: delay delay after next < 1000ms
    4. byte: light on time < 10ms
    IF 0x00 sent - means end of message
    **/

    //SAVE SETTINGS TO EEPROM

}

