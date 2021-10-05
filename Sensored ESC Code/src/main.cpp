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

#define HALL3 PORTC3
#define HALL2 PORTC2
#define HALL1 PORTC1

#define lowMax 0x00
#define lowMin 0xFF

#define highMax 0xFF
#define highMin 0x00

static uint16_t currentThrottle = 0;

//Amount of throttle change to apply to the pwm value at 50hz
//This will be used for artificial acceleration to ensure power is not applied too quickly
static uint8_t throttleAcceleration = 5;

//Stores a 16 bit timer value, this is incremented at 31250hz
volatile uint16_t timer = 0;

//Stores a 16 bit timer value, this is incremented at 31250hz
//This value is used to ensure the motor is not stuck on the same commutation for too long (e.g. stalled)
volatile uint16_t watchdog = 0;

//Stores the last time the throttle was sampled
uint16_t lastThrottleSample = 0;

//The intensity of regenerative braking
static uint8_t brakeIntensity = 0;

//The latest recorded commutation step
uint8_t lastCommStep = 0;

//The maximum time for a commutation change to occur before the commutation is halted (30ms)
static uint16_t timeout = 1072;

uint8_t pwmVal = 0;

//There are only 6 timers, of which all are utilised for PWM.
//Therefore a "software timer" has been made by incrementing a
//variable at the overflow of one of the timers.
//Resulting in a 31.25kHz timer
ISR (TIMER0_OVF_vect){
  timer++;
  watchdog++;
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

  Serial.begin(9600);

  //Enable ir2136
  digitalWrite(2, HIGH);
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

void resetWatchdog(uint8_t commStep){
  if(lastCommStep != commStep){
    watchdog = 0;
    lastCommStep = commStep;
  }
}

void getThrottle(){
  uint16_t throttleIn = analogRead(A7);
  uint16_t targetThrottle;

  //targetThrottle = throttleIn;
  //Scale throttle input to be within the range of 0-255
  if(throttleIn > 860){
    targetThrottle = 255;
  }
  else if(throttleIn > 180)
    targetThrottle = map(throttleIn, 180, 860, 0, 255);
  else
    targetThrottle = 0;

  if(targetThrottle >= 15){
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

  Serial.print(throttleIn);
  Serial.print(" ");
  Serial.print(targetThrottle);
  Serial.print(" ");
  Serial.print(pwmVal);
  Serial.print("\n");
}

void loop() {
  if(timer - lastThrottleSample > 625){ //Sample the throttle at 50hz
    getThrottle();
    lastThrottleSample = timer;
  }
  
  uint8_t hallIn = ((PINC >> HALL3) & 1) << 2 | ((PINC >> HALL2) & 1) << 1 | ((PINC >> HALL1) & 1);
  switch(hallIn){
    case(B101):
      resetWatchdog(B101);       
      COMM0(pwmVal);
      break;
    case(B001):
      resetWatchdog(B001); 
      COMM1(pwmVal);
      break;
    case(B011):
      resetWatchdog(B011); 
      COMM2(pwmVal);
      break;
    case(B010):
      resetWatchdog(B010); 
      COMM3(pwmVal);
      break;
    case(B110):
      resetWatchdog(B110); 
      COMM4(pwmVal);
      break;
    case(B100):
      resetWatchdog(B100); 
      COMM5(pwmVal);
      break;
    default: //All low
      COMMDisable();
  }
  if(watchdog > timeout){
    COMMDisable();
  }
}