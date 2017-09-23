
// Greetingz Commander


#include <Wire.h>
#include <Servo.h>
Servo throttleServo;  // Servo for throttle control

#define SRC_NEUTRAL 1500
#define SRC_MAX 2000
#define SRC_MIN 1000
#define TRC_NEUTRAL 1500
#define TRC_MAX 1900
#define TRC_MIN 1000
#define ERROR_center 50
#define ERROR_stop_center 20
#define pERROR 0

uint16_t unSteeringMin = SRC_MIN + pERROR;
uint16_t unSteeringMax = SRC_MAX - pERROR;
uint16_t unSteeringCenter = SRC_NEUTRAL;

uint16_t unThrottleMin = TRC_MIN + pERROR;
uint16_t unThrottleMax = TRC_MAX - pERROR;
uint16_t unThrottleCenter = TRC_NEUTRAL;

#define PWM_STEERING_MIN 0
#define PWM_STEERING_MAX 255
#define PWM_THROTTLE_MIN 10
#define PWM_THROTTLE_MAX 255

unsigned long nosignalsafety = 0;

// In/Outputs
#define THROTTLE_ENABLE_PIN A0
#define THROTTLE_DIRECTION_PIN A1
// A4 & A5 for arduino communication
#define THROTTLE_IN_PIN 2
#define STEERING_IN_PIN 3
#define STEERING_OUT_PWM_PIN 6
#define STEERING_OUT_DIRECTION_PIN 7
#define THROTTLE_OUT_SPEEDSERVO_PIN 9
#define LIMIT_STEER_LEFT_PIN 10
#define LIMIT_STEER_RIGHT_PIN 11
#define LASER_STEER_PIN 15

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define THROTTLE_FLAG 1
#define STEERING_FLAG 2

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;
volatile uint16_t unThrottleInShared;
volatile uint16_t unSteeringInShared;
volatile uint16_t unLastThrottleInShared;
volatile uint16_t unLastSteeringInShared;

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

void setup()
{
  Serial.begin(9600);
  Serial.println("mounting swap partition");
  //Wire.begin();
  attachInterrupt(0 /* INT0 = THROTTLE_IN_PIN */,calcThrottle,CHANGE);
  attachInterrupt(1 /* INT1 = STEERING_IN_PIN */,calcSteering,CHANGE);

  throttleServo.attach(THROTTLE_OUT_SPEEDSERVO_PIN);

  pinMode(THROTTLE_ENABLE_PIN,OUTPUT);
  pinMode(THROTTLE_DIRECTION_PIN,OUTPUT);
  pinMode(STEERING_OUT_PWM_PIN,OUTPUT);
  pinMode(STEERING_OUT_DIRECTION_PIN,OUTPUT);

  unLastThrottleInShared = TRC_NEUTRAL;
}

void loop()
{
  static uint16_t unThrottleIn;
  static uint16_t unSteeringIn;
  static int16_t unThrottleDiff;
  static uint8_t bUpdateFlags;

  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables
    
    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;

    if(bUpdateFlags & THROTTLE_FLAG)
    {
      // do check with last value to make sure its not garbage or to little change

      unThrottleDiff = unThrottleInShared - unLastThrottleInShared;
      if (unThrottleDiff < 0) {
        unThrottleDiff = unThrottleDiff*-1;  
      }
      if (unThrottleDiff > 250) {
        bUpdateFlags = 0;
      }

      unThrottleIn = unThrottleInShared;
    }

    if(bUpdateFlags & STEERING_FLAG)
    {
      unSteeringIn = unSteeringInShared;
    }
    
    bUpdateFlagsShared = 0;
    unLastThrottleInShared = unThrottleInShared;

    interrupts(); 
  }

  if(bUpdateFlags & THROTTLE_FLAG)
  {
    unThrottleIn = constrain(unThrottleIn,unThrottleMin,unThrottleMax);
    
    if(unThrottleIn > (unThrottleCenter + ERROR_center))
    {
      gThrottle = map(unThrottleIn,(unThrottleCenter + ERROR_center),unThrottleMax,PWM_THROTTLE_MIN,PWM_THROTTLE_MAX);
      gThrottleDirection = DIRECTION_FORWARD;
    }
    else if (unThrottleIn < (unThrottleCenter - ERROR_center))
    {
      gThrottle = map(unThrottleIn,unThrottleMin,(unThrottleCenter- ERROR_center),PWM_THROTTLE_MAX,PWM_THROTTLE_MIN);
      gThrottleDirection = DIRECTION_REVERSE;
    }
    
    // Prevent flapping of the relays
    if(unThrottleIn < (unThrottleCenter + ERROR_stop_center) && unThrottleIn > (unThrottleCenter - ERROR_stop_center))
    {
      gThrottle = 0;
      gThrottleDirection = DIRECTION_STOP;
    }

    // servo
    throttleSpeed(gThrottle);
    // direction & enable relays
    throttleDirection(gThrottleDirection);
  }

  if(bUpdateFlags & STEERING_FLAG)
  {
    gSteeringDirection = DIRECTION_LEFT;
    unSteeringIn = constrain(unSteeringIn,unThrottleMin,unThrottleMax);
    
    if(unSteeringIn > (unSteeringCenter + ERROR_center))
    {
      gSteering = map(unSteeringIn,(unSteeringCenter + ERROR_center),unSteeringMax,PWM_STEERING_MIN,PWM_STEERING_MAX);
      gSteeringDirection = DIRECTION_LEFT;
    }
    else if (unSteeringIn < (unSteeringCenter - ERROR_center))
    {
      gSteering = map(unSteeringIn,unSteeringMin,(unSteeringCenter- ERROR_center),PWM_STEERING_MAX,PWM_STEERING_MIN);
      gSteeringDirection = DIRECTION_RIGHT;
    }
    else
    {
      gSteering = 0;
    }

    if (gSteering > 100) {
      sendWire(gSteeringDirection + 1);
    }
    
    md10rpmSpeed(gSteering,gSteeringDirection);
  }
  
  // no signal == no motor output
  if(bUpdateFlags == 0) {
    // Check if last signal was later then 50 ms ago
    if (nosignalsafety < (millis() - 50)) {
      md10rpmSpeed(0, 0);
      throttleSpeed(0);
      throttleDirection(DIRECTION_STOP);
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
 switch(mDirection)
  {
  case DIRECTION_LEFT:
    if(digitalRead(LIMIT_STEER_LEFT_PIN) == LOW) {
      // rpm devide by 2 for better control
      analogWrite(STEERING_OUT_PWM_PIN,rpm / 1.5);
      digitalWrite(STEERING_OUT_DIRECTION_PIN,mDirection);

    } else {
      analogWrite(STEERING_OUT_PWM_PIN,0);
    }
    break;
  case DIRECTION_RIGHT:
    if(digitalRead(LIMIT_STEER_RIGHT_PIN) == LOW) {
      // rpm devide by 2 for better control
      analogWrite(STEERING_OUT_PWM_PIN,rpm / 1.5);
      digitalWrite(STEERING_OUT_DIRECTION_PIN,mDirection);
    } else {
      analogWrite(STEERING_OUT_PWM_PIN,0);
    }
    break;
  }
  if (rpm > 10) {
      digitalWrite(LASER_STEER_PIN,HIGH);
  } else {
      digitalWrite(LASER_STEER_PIN,LOW);
  }
}

void throttleSpeed(int pos) {
  // pos 10 keeps the servo silent :)
  if (pos < 10) {
     pos = 10;
  }
  throttleServo.write(pos);
}

void throttleDirection(int gDirection) {
 switch(gDirection)
  {
  case DIRECTION_FORWARD:
    digitalWrite(THROTTLE_ENABLE_PIN,LOW);
    digitalWrite(THROTTLE_DIRECTION_PIN,HIGH);
    break;
  case DIRECTION_REVERSE:
    digitalWrite(THROTTLE_ENABLE_PIN,LOW);
    digitalWrite(THROTTLE_DIRECTION_PIN,LOW);
    break;
  case DIRECTION_STOP:
    digitalWrite(THROTTLE_ENABLE_PIN,HIGH);
    digitalWrite(THROTTLE_DIRECTION_PIN,HIGH);
    break;
  }
}

void sendWire(int value) {
  //Wire.beginTransmission(8); // transmit to device #8
  //Wire.write(value);
  //Serial.println(value);
  //Wire.endTransmission();    // stop transmitting
}
