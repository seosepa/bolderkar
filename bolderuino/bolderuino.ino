
// Greetingz Commander

#include <Servo.h>
Servo throttleServo;  // Servo for throttle control

#define SRC_NEUTRAL 1500
#define SRC_MAX 2000
#define SRC_MIN 1000
#define TRC_NEUTRAL 1500
#define TRC_MAX 2000
#define TRC_MIN 1000
#define ERROR_center 50
#define pERROR 100

uint16_t unSteeringMin = SRC_MIN + pERROR;
uint16_t unSteeringMax = SRC_MAX - pERROR;
uint16_t unSteeringCenter = SRC_NEUTRAL;

uint16_t unThrottleMin = TRC_MIN + pERROR;
uint16_t unThrottleMax = TRC_MAX - pERROR;
uint16_t unThrottleCenter = TRC_NEUTRAL;

#define PWM_MIN 0
#define PWM_MAX 255

#define pinThrottleServo 5
#define pinMd10Pwm 6
#define pinMd10Direction 7

unsigned long nosignalsafety = 0;

// Assign your channel in pins
#define THROTTLE_IN_PIN 2
#define STEERING_IN_PIN 3

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define THROTTLE_FLAG 1
#define STEERING_FLAG 2

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t unThrottleInShared;
volatile uint16_t unSteeringInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t ulThrottleStart;
uint32_t ulSteeringStart;

uint8_t gThrottle = 0;
uint8_t gSteering = 0;

#define DIRECTION_STOP 0
#define DIRECTION_FORWARD 1
#define DIRECTION_REVERSE 2

#define DIRECTION_LEFT 0
#define DIRECTION_RIGHT 1

uint8_t gThrottleDirection = DIRECTION_STOP;
uint8_t gSteeringDirection = DIRECTION_LEFT;

#define IDLE_MAX 50

void setup()
{
  Serial.begin(9600);
  Serial.println("mounting swap partition");

  attachInterrupt(0 /* INT0 = THROTTLE_IN_PIN */,calcThrottle,CHANGE);
  attachInterrupt(1 /* INT1 = STEERING_IN_PIN */,calcSteering,CHANGE);

  throttleServo.attach(5);

  pinMode(pinMd10Pwm,OUTPUT);
  pinMode(pinMd10Direction,OUTPUT);
}

void loop()
{
  static uint16_t unThrottleIn;
  static uint16_t unSteeringIn;
  static uint8_t bUpdateFlags;

  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables
    
    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;

    if(bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = unThrottleInShared;
    }

    if(bUpdateFlags & STEERING_FLAG)
    {
      unSteeringIn = unSteeringInShared;
    }
    
    bUpdateFlagsShared = 0;

    interrupts(); 
  }

  if(bUpdateFlags & THROTTLE_FLAG)
  {
    unThrottleIn = constrain(unThrottleIn,unThrottleMin,unThrottleMax);
    
    if(unThrottleIn > (unThrottleCenter + ERROR_center))
    {
      gThrottle = map(unThrottleIn,(unThrottleCenter + ERROR_center),unThrottleMax,PWM_MIN,PWM_MAX);
      gThrottleDirection = DIRECTION_FORWARD;
    }
    else if (unThrottleIn < (unThrottleCenter - ERROR_center))
    {
      gThrottle = map(unThrottleIn,unThrottleMin,(unThrottleCenter- ERROR_center),PWM_MAX,PWM_MIN);
      gThrottleDirection = DIRECTION_REVERSE;
    }
    else
    {
      gThrottleDirection = DIRECTION_STOP;
      gThrottle=0;
    }

    // servo
    throttleSpeed(gThrottle);

  }

  if(bUpdateFlags & STEERING_FLAG)
  {
    gSteeringDirection = DIRECTION_LEFT;
    unSteeringIn = constrain(unThrottleIn,unThrottleMin,unThrottleMax);
    
    if(unSteeringIn > (unSteeringCenter + ERROR_center))
    {
      gSteering = map(unSteeringIn,(unSteeringCenter + ERROR_center),unSteeringMax,PWM_MIN,PWM_MAX);
      gSteeringDirection = DIRECTION_LEFT;
    }
    else if (unSteeringIn < (unSteeringCenter - ERROR_center))
    {
      gSteering = map(unSteeringIn,unSteeringMin,(unSteeringCenter- ERROR_center),PWM_MAX,PWM_MIN);
      gSteeringDirection = DIRECTION_RIGHT;
    }
    else
    {
      gSteering = 0;
    }

    md10rpmSpeed(gSteering,gSteeringDirection);
  }
  
  // no signal == no motor output
  if(bUpdateFlags == 0) {
    // Check if last signal was later then 50 ms ago
    if (nosignalsafety < (millis() - 50)) {
      md10rpmSpeed(0, 0);
      Serial.println("safety - no signal");
    } 
  }

  bUpdateFlags = 0;
}


void calcThrottle()
{
  // signal interrupt, reset the nosignalsafety
  nosignalsafety = millis();
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(THROTTLE_IN_PIN) == HIGH)
  {
    ulThrottleStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

void calcSteering()
{
  // signal interrupt, reset the nosignalsafety
  nosignalsafety = millis();
  if(digitalRead(STEERING_IN_PIN) == HIGH)
  {
    ulSteeringStart = micros();
  }
  else
  {
    unSteeringInShared = (uint16_t)(micros() - ulSteeringStart);
    bUpdateFlagsShared |= STEERING_FLAG;
  }
}

void md10rpmSpeed(int rpm, int mDirection) {
  // rpm devide by 2 for better control
  analogWrite(pinMd10Pwm,rpm / 2); 
  digitalWrite(pinMd10Direction,mDirection);
}


void throttleSpeed(int pos) {
    throttleServo.write(pos);
}

