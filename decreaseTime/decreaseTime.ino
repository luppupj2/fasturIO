
/*
   Changed the speed from:
   3.088us to 1.576us on the arduino zero
   and
   10.680us to 3.480us on a redBoard
*/

#ifdef ARDUINO_ARCH_AVR
#define REGTYPE uint8_t   // AVR uses 8-bit registers
#else
#define REGTYPE uint32_t
#endif

REGTYPE pin[14];
volatile REGTYPE *mode[14];
volatile REGTYPE *out[14];
volatile REGTYPE *in[14];
int pinState = 0;

void setup() {
  fpinMode(12, OUTPUT);
  Serial.begin(9600);

}

void loop() {
  fdigitalWrite(12, HIGH);
  fpinMode(12,INPUT);
  fpinMode(12,OUTPUT);
  fdigitalWrite(12, LOW);

  //pinMode(12, INPUT);
  //SerialUSB.println(fdigitalRead(12));
}

void fpinMode(int pinNum, int state)
{

  mode[pinNum] = portModeRegister(digitalPinToPort(pinNum));
  out[pinNum] = portOutputRegister(digitalPinToPort(pinNum));
  pin[pinNum] = digitalPinToBitMask(pinNum);

  if (state == OUTPUT)
  {
    *mode[pinNum] |= pin[pinNum];
  }
  else
  {
    *mode[pinNum] &= ~pin[pinNum];
  }
}

void fdigitalWrite(int pinNum, int state)
{
  if (state == HIGH)
  {
    *out[pinNum] |= pin[pinNum];
  }
  else
  {
    *out[pinNum] &= ~pin[pinNum];
  }
}



int fdigitalRead(int pinNum) //only returns false
{

  if (*portInputRegister(digitalPinToPort(pinNum)) & pin[pinNum])
  {
    return HIGH;
  }
  else
  {
    return LOW;
  }
}


/*
   https://github.com/arduino/Arduino/blob/8385aedc642d6cd76b50ac5167307121007e5045/hardware/arduino/avr/cores/arduino/wiring_digital.c
   https://github.com/arduino/Arduino/blob/8385aedc642d6cd76b50ac5167307121007e5045/hardware/arduino/sam/cores/arduino/wiring_digital.c
*/
