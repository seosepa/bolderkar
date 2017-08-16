
// Greetingz Commander

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

#define GEAR_NONE 1
#define GEAR_IDLE 1
#define GEAR_FULL 2

#define pinMd10Pwm 6
#define pinMd10Direction 7
uint32_t nosignalsafety = 0;

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
uint8_t gGear = GEAR_NONE;
uint8_t gOldGear = GEAR_NONE;

#define DIRECTION_STOP 0
#define DIRECTION_FORWARD 1
#define DIRECTION_REVERSE 2

uint8_t gThrottleDirection = DIRECTION_STOP;
uint8_t gDirection = DIRECTION_STOP;
uint8_t gOldDirection = DIRECTION_STOP;

#define IDLE_MAX 50

unsigned long pulse_time  ;

void setup()
{
  Serial.begin(9600);

  Serial.println("hello");

  attachInterrupt(0 /* INT0 = THROTTLE_IN_PIN */,calcThrottle,CHANGE);
  attachInterrupt(1 /* INT1 = STEERING_IN_PIN */,calcSteering,CHANGE);
  
  pulse_time =millis() ;;
  pinMode(pinMd10Pwm,OUTPUT);
  pinMode(pinMd10Direction,OUTPUT);
}

void loop()
{
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained
  // between calls to loop.
  static uint16_t unThrottleIn;
  static uint16_t unSteeringIn;
  // local copy of update flags
  static uint8_t bUpdateFlags;
// fail_safe();
  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables
     pulse_time =millis() ;
      // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;

    // in the current code, the shared values are always populated
    // so we could copy them without testing the flags
    // however in the future this could change, so lets
    // only copy when the flags tell us we can.

    if(bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = unThrottleInShared;
    }

    if(bUpdateFlags & STEERING_FLAG)
    {
      unSteeringIn = unSteeringInShared;
    }

    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in bUpdateFlags
    bUpdateFlagsShared = 0;

    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
    // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
    // service routines own these and could update them at any time. During the update, the
    // shared copies may contain junk. Luckily we have our local copies to work with :-)
  }

    // we are checking to see if the channel value has changed, this is indicated 
    // by the flags. For the simple pass through we don't really need this check,
    // but for a more complex project where a new signal requires significant processing
    // this allows us to only calculate new values when we have new inputs, rather than
    // on every cycle.
    if(bUpdateFlags & THROTTLE_FLAG)
    {
      // A good idea would be to check the before and after value, 
      // if they are not equal we are receiving out of range signals
      // this could be an error, interference or a transmitter setting change
      // in any case its a good idea to at least flag it to the user somehow
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
      gThrottleDirection =DIRECTION_STOP;
      gThrottle=0;
      }
  
      if(gThrottle < IDLE_MAX)
      {
        gGear = GEAR_IDLE;
      }
      else
      {
        gGear = GEAR_FULL;
      }
    }
  
    if(bUpdateFlags & STEERING_FLAG)
    {
      uint8_t directionForPwm = 0;
      
      gDirection = gThrottleDirection;

      switch(gDirection)
      {
      case DIRECTION_FORWARD:
        directionForPwm = 0;
        break;
      case DIRECTION_REVERSE:
        directionForPwm = 1;
        break;
      }
      
      md10rpmSpeed(gThrottle,directionForPwm);
     
    }
  
  // no signal == no motor output
  if(bUpdateFlags == 0) {
      Serial.println(nosignalsafety);
    if (nosignalsafety < 10) {
      md10rpmSpeed(0, 0);
      Serial.println("safety no signal");
    } else {
      nosignalsafety = nosignalsafety - 1;
    }
  }

  bUpdateFlags = 0;
}


void calcThrottle()
{
  // signal interrupt, reset the nosignalsafety
  nosignalsafety = 100;
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
  nosignalsafety = 100;
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
