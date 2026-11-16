//  STRUCTURES
struct Motor {            // one L298N channel
  int  in1, in2;          // direction pins
  int  en;                // PWM enable pin
  bool invert;            // true if the wheel spins backwards on "forward"
};

struct Sensor {           // one HC-SR04
  int trig, echo;
};

struct Maneuver {         // a pivot: direction + calibrated time
  int dir;                // +1 = right (clockwise), -1 = left
  int ms;                 // time at TURN_SPEED
};

struct Walls {            // result of looking at the three sensors
  bool front, left, right;
};

//  INDEXES (used to address the arrays)
enum MotorId  { LEFT_MOTOR = 0, RIGHT_MOTOR = 1, NUM_MOTORS = 2 };
enum SensorId { FRONT = 0, RIGHT = 1, LEFT = 2, NUM_SENSORS = 3 };   // = ping order

//  HARDWARE TABLES (arrays of structures)
const Motor motors[NUM_MOTORS] = {
  //  in1 in2  en  invert     (set invert=false if that wheel runs backwards)
  {   2,  3,  10, true },     // LEFT_MOTOR  (ENA)
  {   4,  5,  11, true }      // RIGHT_MOTOR (ENB)
};

const Sensor sensors[NUM_SENSORS] = {
  //  trig echo
  {   7,   6 },               // FRONT
  {   9,   8 },               // RIGHT
  {  13,  12 }                // LEFT
};

const char *const sensorNames[NUM_SENSORS] = { "F", "R", "L" };

//  TUNABLES (calibrate these!)
const int BASE_SPEED = 105;      // 0-255 straight driving
const int TURN_SPEED = 100;      // speed while turning in place
const int MIN_SPEED  = 0;        // below this the motors just whine/stall
const int BACK_SPEED = 120;      // reverse speed when making room to turn

const int KICK_SPEED = 200;      // brief full-power burst to break static friction
const int KICK_MS    = 30;

// Distances in cm
const int WALL_PRESENT   = 12;   // side sensor closer than this = wall exists
const int FRONT_BLOCKED  = 9;    // front sensor closer than this = wall ahead
const int SIDE_TARGET    = 5;    // ideal gap to right wall
const int TURN_CLEARANCE = 10;   // free space needed in front before pivoting

// Back-off: up to BACK_PULSES short reverse pulses, re-measuring after each
const int BACK_PULSES    = 4;
const int BACK_PULSE_MS  = 45;

// Timing in ms
const int TURN_90_MS     = 130;
const int TURN_180_MS    = 460;
const int ADVANCE_MS     = 260;  // creep forward so wheels are over the cell centre
const int CELL_ENTER_MS  = 300;  // drive into the new corridor after a turn
const int SENSOR_GAP_MS  = 10;   // pause between pings of different sensors

const float KP = 12.0;           // steering strength for wall centring

// Pre-defined turns (passed to turn() by pointer)
const Maneuver TURN_RIGHT_90 = { +1, TURN_90_MS  };
const Maneuver TURN_LEFT_90  = { -1, TURN_90_MS  };
const Maneuver TURN_AROUND   = { +1, TURN_180_MS };

//  MOTOR FUNCTIONS

// Drive one motor through a pointer to its Motor struct. speed: -255..255
void setMotor(const Motor *m, int speed) {
  bool fwd = (speed >= 0) ^ m->invert;
  digitalWrite(m->in1, fwd ? HIGH : LOW);
  digitalWrite(m->in2, fwd ? LOW  : HIGH);
  analogWrite(m->en, constrain(abs(speed), 0, 255));
}

void drive(int left, int right) {
  setMotor(&motors[LEFT_MOTOR],  left);
  setMotor(&motors[RIGHT_MOTOR], right);
}

void stopMotors() { drive(0, 0); }

//  SENSOR FUNCTIONS

long readCm(const Sensor *s) {
  digitalWrite(s->trig, LOW);  delayMicroseconds(2);
  digitalWrite(s->trig, HIGH); delayMicroseconds(10);
  digitalWrite(s->trig, LOW);
  unsigned long d = pulseIn(s->echo, HIGH, 8000UL);   // 8 ms timeout ~ 137 cm
  if (d == 0) return 200;                             // nothing heard = far
  return d / 58;                                      // microseconds -> cm
}

long readFront() { return readCm(&sensors[FRONT]); }

// Fill the caller's array `out` (NUM_SENSORS long) with all three distances
void readAll(long *out) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    out[i] = readCm(&sensors[i]);
    if (i < NUM_SENSORS - 1) delay(SENSOR_GAP_MS);    // let the echo die out
  }
}

// Turn raw distances into wall flags. Reads from d[], writes through w.
void detectWalls(const long *d, Walls *w) {
  w->front = d[FRONT] < FRONT_BLOCKED;
  w->right = d[RIGHT] < WALL_PRESENT;
  w->left  = d[LEFT]  < WALL_PRESENT;
}

void printReadings(const long *d) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorNames[i]);
    Serial.print(':');
    Serial.print(d[i]);
    Serial.print(i < NUM_SENSORS - 1 ? ' ' : '\n');
  }
}

//  MOVEMENT BUILDING BLOCKS

void driveForward(int ms, int stopDist) {
  drive(BASE_SPEED, BASE_SPEED);
  unsigned long t0 = millis();
  while (millis() - t0 < (unsigned long)ms) {
    if (readFront() < stopDist) {
      stopMotors();
      return;
    }
  }
}

void makeTurnRoom() {
  if (readFront() >= TURN_CLEARANCE) return;

  for (int i = 0; i < BACK_PULSES; i++) {
    drive(-BACK_SPEED, -BACK_SPEED);
    delay(BACK_PULSE_MS);
    stopMotors();
    delay(40);                                        
    if (readFront() >= TURN_CLEARANCE) break;
  }
}


void pivot(const Maneuver *m) {
  drive(m->dir * KICK_SPEED, -m->dir * KICK_SPEED);
  delay(KICK_MS);
  int rest = m->ms - (KICK_MS * KICK_SPEED) / TURN_SPEED;
  if (rest > 0) {
    drive(m->dir * TURN_SPEED, -m->dir * TURN_SPEED);
    delay(rest);
  }
  stopMotors();
  delay(80);
}

// Make room, then pivot
void turn(const Maneuver *m) {
  makeTurnRoom();
  pivot(m);
}


//  DECISIONS FUNCTIONS

// Steering correction (PWM units) for driving down a corridor
int steerCorrection(const long *d, const Walls *w) {
  float error;
  if (w->left) error = d[RIGHT] - d[LEFT];            // both walls: centre between them
  else         error = d[RIGHT] - SIDE_TARGET;        // right wall only: hold target gap
  return constrain((int)(KP * error), -60, 60);
}

void followCorridor(const long *d, const Walls *w) {
  int c = steerCorrection(d, w);                      // c>0 = too far from right wall
  drive(constrain(BASE_SPEED + c, MIN_SPEED, 255),
        constrain(BASE_SPEED - c, MIN_SPEED, 255));
}

// Right-hand rule: priority is RIGHT > FRONT > LEFT > BACK
void decide(const long *d, const Walls *w) {
  if (!w->right) {
    // opening on the right: roll to the cell centre, turn, enter the new corridor
    stopMotors();
    driveForward(ADVANCE_MS, TURN_CLEARANCE);
    stopMotors(); delay(80);
    turn(&TURN_RIGHT_90);
    driveForward(CELL_ENTER_MS, FRONT_BLOCKED);
  }
  else if (!w->front) {
    followCorridor(d, w);
  }
  else if (!w->left) {
    stopMotors(); delay(80);
    turn(&TURN_LEFT_90);
  }
  else {
    stopMotors(); delay(80);
    turn(&TURN_AROUND);
  }
}

//  ARDUINO 
void setup() {
  Serial.begin(9600);

  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(motors[i].in1, OUTPUT);
    pinMode(motors[i].in2, OUTPUT);
    pinMode(motors[i].en,  OUTPUT);
  }
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensors[i].trig, OUTPUT);
    pinMode(sensors[i].echo, INPUT);
  }

  stopMotors();
  delay(2000);   // time to put the robot down on the start square
}

void loop() {
  long  dist[NUM_SENSORS];     // array filled by readAll()
  Walls walls;                 // structure filled by detectWalls()

  readAll(dist);
  printReadings(dist);
  detectWalls(dist, &walls);
  decide(dist, &walls);
}
