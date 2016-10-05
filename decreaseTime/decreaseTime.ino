
#ifdef ARDUINO_ARCH_AVR   // determines size (8 v. 32 bit) depending on the board
#define REGTYPE uint8_t
#else
#define REGTYPE uint32_t
#endif

REGTYPE pin[14];         // determines the possible amount of digital pins

volatile REGTYPE *mode[14];
volatile REGTYPE *out[14];
volatile REGTYPE *in[14];

void setup() {

#ifdef ARDUINO_ARCH_AVR  //configures Serial connection depending on board type                
  Serial.begin(9600);
#else
  SerialUSB.begin(9600);
#endif

  fpinMode(12, OUTPUT);
}

void loop() {
  fdigitalWrite(12, HIGH);
  fpinMode(12, INPUT);
#ifdef ARDUINO_ARCH_AVR  //configures Serial connection depending on board type                
  Serial.println(fdigitalRead(12));
#else
  SerialUSB.println(fdigitalRead(12));
#endif
  fpinMode(12, OUTPUT);
  fdigitalWrite(12, LOW);
  fpinMode(12, INPUT);
#ifdef ARDUINO_ARCH_AVR  //configures Serial connection depending on board type                
  Serial.println(fdigitalRead(12));
#else
  SerialUSB.println(fdigitalRead(12));
#endif
  fpinMode(12, OUTPUT);
}

void fpinMode(int pinNum, int state)    //function to determine pin mode ( pin #, INPUT or OUTPUT)
{

  mode[pinNum] = portModeRegister(digitalPinToPort(pinNum));   //returns the address of DDRB / determines if pin is OUTPUT or INPUT
  out[pinNum] = portOutputRegister(digitalPinToPort(pinNum));  //returns the adress of PORTB, PORTC, or PORTD / Used to execute INPUT/OUTPUT commands
  pin[pinNum] = digitalPinToBitMask(pinNum);                   //returns the value that shifts 1 to the left by arguement EX: pin 5 = 0b00100000

  if (state == OUTPUT)                                         // if the state set to OUTPUT
  {
    *mode[pinNum] |= pin[pinNum];                              // if a pin is set to output then change its mode
    // EX:(Pin 5 OUTPUT) mode = 0b00000000 |= 0b00100000 == 0b00100000
  }
  else
  {
    *mode[pinNum] &= ~pin[pinNum];                             // if a pin is set to output then change its mode
    // EX:(Pin 5 INPUT) mode = 0b00000001 &= 1b11011111 == 0b00000001
    // ~ means not/the opposite of
  }
}

void fdigitalWrite(int pinNum, int state)                      // function to determine digital State(pin#, HIGH or LOW)
{
  if (state == HIGH)
  {
    *out[pinNum] |= pin[pinNum];                               //if the pin is set to HIGH then change its HIGH
    //EX:(Pin 5 HIGH) out = 0b00000000 |= 0b00100000 == 0b00100000
  }
  else
  {
    *out[pinNum] &= ~pin[pinNum];                             //if the pin is set to LOW then change it to LOW
    // EX:(Pin 5 INPUT) out = 0b00000001 &= 1b11011111 == 0b00000001
    // ~ means not/the opposite of,  1b11011111 means that all pins were set
    // to low except 5 and 0b00000001 means that the last pin was supposed to be set to HIGH
  }
}
int fdigitalRead(int pinNum) 
{

  if (*portInputRegister(digitalPinToPort(pinNum)) & pin[pinNum])         // if they are equal then return high if not return low 
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
