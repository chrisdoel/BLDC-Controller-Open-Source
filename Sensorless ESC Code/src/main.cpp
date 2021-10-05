#define _DISABLE_ARDUINO_TIMER0_INTERRUPT_HANDLER_

#include <Arduino.h>
#include <wiring.c>

#define H3Pwm OCR2B
#define H2Pwm OCR1B
#define H1Pwm OCR0B

#define L3Pwm OCR2A
#define L2Pwm OCR1A
#define L1Pwm OCR0A

#define EN PORTD2

#define SENSE3 PORTC3
#define SENSE2 PORTC2
#define SENSE1 PORTC1

#define lowMax 0x00
#define lowMin 0xFF

#define highMax 0xFF
#define highMin 0x00

//Stores when the latest commutation occurred
uint16_t commutationStartTime = 0;

//Stores a 16 bit timer value, this is incremented at 31250hz
volatile uint16_t timer = 0;

//Stores a 16 bit timer value, this is incremented at 31250hz
volatile uint16_t watchdog = 0;

//Stores the last time the throttle was sampled
uint16_t lastThrottleSample = 0;

//The maximum time for a commutation change to occur before the commutation is halted (30ms)
uint16_t timeout = 1072;

//The current step of the commutation sequence
uint8_t commutation = 0;

//Whether the startup procedure has successfully been achieved
bool running = 0;

//The PWM duty cycle to apply to the mosfets
uint8_t pwmVal = 0;

//The intensity of regenerative braking
uint8_t brakeIntensity = 20;

uint16_t currentThrottle = 0;

//Amount of change to apply to the pwm value at 50hz
//This will be used for artificial acceleration to ensure power is not applied too quickly
uint8_t throttleAcceleration = 2;

//There are only 6 timers, of which all are utilised for PWM.
//Therefore a "software timer" has been made by incrementing a
//variable at the overflow of one of the timers.
//Resulting in a 31.25kHz timer
ISR (TIMER0_OVF_vect){
  timer++;
  watchdog++;
}

//Disable all PWM output to mosfets
void COMMDisable(){
  L1Pwm = lowMin;
  L2Pwm = lowMin;
  L3Pwm = lowMin;

  H1Pwm = highMin;
  H2Pwm = highMin;
  H3Pwm = highMin;
  currentThrottle = 0;
}

void COMM0(uint8_t pwm){
  if(pwmVal)
    L1Pwm = lowMin;
  else
    L1Pwm = lowMin - brakeIntensity;
  L2Pwm = lowMax; //L2 On
  L3Pwm = lowMin;

  H1Pwm = pwm; //H1 On
  H2Pwm = highMin;
  H3Pwm = highMin;
}

void COMM1(uint8_t pwm){
  if(pwmVal)
    L1Pwm = lowMin;
  else
    L1Pwm = lowMin - brakeIntensity;
  L2Pwm = lowMin;
  L3Pwm = lowMax; //L3 On

  H1Pwm = pwm; //H1 On
  H2Pwm = highMin;
  H3Pwm = highMin;
}

void COMM2(uint8_t pwm){
  L1Pwm = lowMin;
  if(pwmVal)
    L2Pwm = lowMin;
  else
    L2Pwm = lowMin - brakeIntensity;
  L3Pwm = lowMax; //L3 On

  H1Pwm = highMin;
  H2Pwm = pwm; //H2 On
  H3Pwm = highMin;
}

void COMM3(uint8_t pwm){
  L1Pwm = lowMax; //L1 On
  if(pwmVal)
    L2Pwm = lowMin;
  else
    L2Pwm = lowMin - brakeIntensity;
  L3Pwm = lowMin;

  H1Pwm = highMin;
  H2Pwm = pwm; //H2 On
  H3Pwm = highMin;
}

void COMM4(uint8_t pwm){
  L1Pwm = lowMax; //L1 On
  L2Pwm = lowMin;
  if(pwmVal)
    L3Pwm = lowMin;
  else
    L3Pwm = lowMin - brakeIntensity;

  H1Pwm = highMin;
  H2Pwm = highMin;
  H3Pwm = pwm; //H3 On
}

void COMM5(uint8_t pwm){
  L1Pwm = lowMin;
  L2Pwm = lowMax; //L2 On
  if(pwmVal)
    L3Pwm = lowMin;
  else
    L3Pwm = lowMin - brakeIntensity;

  H1Pwm = highMin;
  H2Pwm = highMin;
  H3Pwm = pwm; //H3 On
}

bool checkRise(uint8_t port){
  uint8_t total = 0;
  for(int i = 0; i < 10; i++) //Sample the pin 10 times to prevent false positives 
    total += (PINC >> port) & 1;
  if(total == 10) //If all 10 samples were high then we can assume a transition has occurred
    return true;
  return false;
}

bool checkFall(uint8_t port){
  uint8_t total = 0;
  for(int i = 0; i < 10; i++) //Sample the pin 10 times to prevent false positives 
    total += (PINC >> port) & 1;
  if(total == 0) //If all 10 samples were low then we can assume a transition has occurred
    return true;
  return false;
}


bool startup(){
  uint8_t startupPower = 30; //The PWM duty cycle to apply during startup

  uint16_t commutationDelay = 144; //The number of ticks at 31.25khz for each commutation time
  uint16_t accelleration = 0; //The reduction to apply to commutationDelay after each commutation step
  uint8_t commutationChanges = 0;
  uint8_t commutationIterations = 12; //How many commutation steps to perform

  commutationStartTime = timer;
  commutation = 0;
  COMM0(startupPower);

  while(commutationChanges < commutationIterations){
    //Serial.print(commutation);
    switch(commutation){
      case(0):
        if(timer - commutationStartTime > commutationDelay){ //Check if no BEMF crossing has been detected within a certian time frame
          COMM1(startupPower);
          commutationStartTime = timer; //Reset timer for BEMF crossing
          commutation = 1; //Prepare to attempt next commutation
          commutationDelay -= accelleration; //Decrease time frame for open loop BEMF commutation
          commutationChanges++;
        }
        break;
      case(1):
        if(timer - commutationStartTime > commutationDelay){
          COMM2(startupPower);
          commutationStartTime = timer;
          commutation = 2;
          commutationDelay -= accelleration;
          commutationChanges++;
        }
        break;
      case(2):
        if(timer - commutationStartTime > commutationDelay){
          COMM3(startupPower);
          commutationStartTime = timer;
          commutation = 3;
          commutationDelay -= accelleration;
          commutationChanges++;
        }
        break;
      case(3):
        if(timer - commutationStartTime > commutationDelay){
          COMM4(startupPower);
          commutationStartTime = timer;
          commutation = 4;
          commutationDelay -= accelleration;
          commutationChanges++;
        }
        break;
      case(4):
        if(timer - commutationStartTime > commutationDelay){
          COMM5(startupPower);
          commutationStartTime = timer;
          commutation = 5;
          commutationDelay -= accelleration;
          commutationChanges++;
        }
        break;
      case(5):
        if(timer - commutationStartTime > commutationDelay){
          COMM0(startupPower);
          commutationStartTime = timer;
          commutation = 0;
          commutationDelay -= accelleration;
          commutationChanges++;
        }
        break;
      default:
        COMMDisable();
    }
  }

  while((timer - commutationStartTime) < timeout){ //Observe the expected back emf after manually comutating the motor
    if(checkRise(SENSE3)){ //If a zero crossing event occurred we can start the normal commutation
      watchdog = 0;
      commutation = 1;
      COMM1(startupPower);
      Serial.println("Got BEMF");
      currentThrottle = startupPower;
      pwmVal = startupPower;
      return true;
    }
  }
  Serial.println("Couldnt start");
  return false;
}

//Commutation for when the startup procedure was successful
void activeCommutate(){
  switch(commutation){
    case(0):
      if(checkRise(SENSE3)){  //Check if the zero crossing event has occurred
        watchdog = 0; //Record the time of the zero crossing event
        commutation = 1; //Move to the next commutation step
        COMM1(pwmVal); //Set the outputs of the MOSFETs
      }
      break;

    case(1):
      if(checkFall(SENSE2)){
        watchdog = 0;
        commutation = 2;
        COMM2(pwmVal);
      }
      break;

    case(2):
      if(checkRise(SENSE1)){
        watchdog = 0;
        commutation = 3;
        COMM3(pwmVal);
      }
      break;

    case(3):
      if(checkFall(SENSE3)){
        watchdog = 0;
        commutation = 4;
        COMM4(pwmVal);
      }
      break;

    case(4):
      if(checkRise(SENSE2)){
        watchdog = 0;
        commutation = 5;
        COMM5(pwmVal);
      }
      break;

    case(5):
      if(checkFall(SENSE1)){
        watchdog = 0;
        commutation = 0;
        COMM0(pwmVal);
      }
      break;

    default:
      COMMDisable();
  }

  if(watchdog > timeout) {//Check if there has been no BEMF detected in the last 30ms
    uint16_t now = timer;
    running = false; //We can safely assume the motor has stopped rotating
  }
}

void setup() {
  noInterrupts();
  ADCSRA &= ~((1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2)); //Clear ADC prescaler bits
  ADCSRA |= (1 << ADPS0) | (1 << ADPS2); //Set adc prescaler to 32 = 20 microsecond analogRead
  
  //TIMER0
  TCCR0A = 0; //Set timer0 config to 0
  TCCR0B = 0;
  //Setup Phase correct PWM
  TCCR0A |= (1 << COM0A1); //Non-inverted low side PWM
  TCCR0A |= (1 << COM0B0);
  TCCR0A |= (1 << COM0B1); //Invert High side PWM
  TCCR0A |= (1 << WGM00);
  TCCR0B = (1 << CS00); //Prescaler divided by 1, 31.37255khz frequency

  //TIMER1
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A |= (1 << COM1A1);
  TCCR1A |= (1 << COM1B0);
  TCCR1A |= (1 << COM1B1);
  TCCR1A |= (1 << WGM10);
  TCCR1B = (1 << CS10);

  //TIMER2
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A |= (1 << COM2A1);
  TCCR2A |= (1 << COM2B0);
  TCCR2A |= (1 << COM2B1);
  TCCR2A |= (1 << WGM20);
  TCCR2B = (1 << CS20);

   //Zero duty cycle pwm
  OCR0A = 0x00;
  OCR0B = 0x00;
  OCR1A = 0x00;
  OCR1B = 0x00;
  OCR2A = 0x00;
  OCR2B = 0x00;

  //Setup PWM output ports
  DDRB = B00001110; //Set PORTB 1, 2 and 3 to output
  DDRD = B01101100; //Set PORTD 5 and 6 to output

  //Setup throttle input port
  DDRC = B00000000; //Set PORTC to input

  //Enable interrupts
  interrupts();
  Serial.begin(115200);
  digitalWrite(2, HIGH); //Enable ir2136

  pinMode(A1, INPUT_PULLUP); //Set BEMF input pins to pull up as the lm339n is open drain output
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
}

void getThrottle(){
  uint16_t targetThrottle = analogRead(A7);

  if(targetThrottle >= 15){
    targetThrottle = map(targetThrottle, 0, 1023, 0, 255);
    if(currentThrottle < targetThrottle && uint16_t(currentThrottle) + throttleAcceleration <= 255) //Artifical acceleration applied
      currentThrottle += throttleAcceleration;
    else //Instant deacceleration
      currentThrottle = targetThrottle;
  }
  else
    currentThrottle = 0;
  pwmVal = currentThrottle;
  
  if(pwmVal < 15)//prevent pulses of less than a microsecond (specified in ir2136 datasheet)
    pwmVal = 0;
}

void loop() {
  if(timer - lastThrottleSample > 625){ //Sample the throttle at 50hz
    getThrottle();
    lastThrottleSample = timer;
  }

  if(pwmVal > 20 && !running)
    running = startup();
  else if(running)
    activeCommutate();
  else
    COMMDisable();
}
