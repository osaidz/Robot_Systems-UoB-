#ifndef _DRIVE_h
#define _DRIVE_h


/*
   Welcome to the backend!

   This code is far from optimal, fueled by a few all nighters and loads of coffee.
   You'll find a lot of "patched" code that needs to be improved.

   Robots should be able to calibrate themselves!

*/


/*
   Changing Register bits gets a whole lot easier.

   cbi - set bit in I/O register
   sbi - clear bit in I/O register

   #define _BV(bit) (1 << (bit))
   #define _SFR_BYTE(sfr) _MMIO_BYTE(_SFR_ADDR(sfr))
   #define _MMIO_BYTE(mem_addr) (*(volatile uint8_t *)(mem_addr))
*/

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

//Function Prototypes
void calibrate_sensors(int c_time, int pwm, bool dir);
void Turn(int y_theta);

/*
   -----------------------------------------------------------------
*/

#include "lineSensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "motor.h"
#include "pid.h"

PID_c line_control(1.1, 0.001, 0.01);   //Line Following Parameters
PID_c heading_control(2, 0, 0);  //Driving Straight


#define B1 14
#define B2 30
#define B3 17



bool start;
int leftTurn, rightTurn;
unsigned long line_pose_timer = 0;
int counter=0;
bool opposite_start = 0;

void Initialise() { //Self explanatory

  Serial.begin(9600);
  pinMode(B1, INPUT_PULLUP);
  pinMode(B2, INPUT_PULLUP);


  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);
  pinMode(L_DIR_PIN, OUTPUT);
  pinMode(R_DIR_PIN, OUTPUT);

  for (int x = 0; x <= 3; x++) {
    pinMode(line_sensor[x], INPUT);
  }

  setupEncoder0();
  setupEncoder1();
}


void check_calib() {                  //Calibration_Routine for IR module
  if (digitalRead(B1) == 0) {
    while (digitalRead(B1) == 0);     //Start on depress
    delay(500);
    calibrate_sensors(500, 70, RIGHT);//(Time,Power,Dir)
    delay(500);
    calibrate_sensors(500, 70, LEFT);
  }
  count_e1 = 0;                       //Reset Encoders
  count_e0 = 0;
}

void start_routine() {

  if (digitalRead(B2) == 0) {
    unsigned long ms = millis();
    start = 1;                      //Start Main Loop
    while (digitalRead(B2) == 0) {
      if ((ms + 200) < millis())opposite_start = 1; start = 1; //Flip Line_dir on long press
    }
  }
}





void go_to_xy(int x_coord, int y_coord, int _theta, int power) {

  //heading_control.reset();                                     //Reset PID cache
  get_dist(x_coord, y_coord);                                    //Where is home?
  Turn((int)angle_away);                                         //Turn Towards home

  long pose_update_timer = 0;
  while ((int)dist_away > 90) { //compensation for error accumulated 
    if (millis() - pose_update_timer >= 40) {
      pose_update_timer = millis();
      update_pose(); //Update position
    }

    //cal_dev((int)); //only good at mainting small angles <25 deg
    //int spwm = 0;//1.0 * constrain(min(leftTurn, rightTurn), 0, 25);
    //int lp = heading_control.update((int)angle_away, yaw);

    int lp = constrain(2 * (yaw - angle_away), -30, 30);
    if (lp > 0) Move.Forward(power - lp, power + lp);
    else Move.Forward(power - lp, power + lp);
    //Move.Forward(36,30); //Motor Bias of +6
    get_dist(x_coord, y_coord);                                 //Am I home?
  }
  Move.Stop();
  delay(500);
  Turn(90); 
  Turn(_theta);                                                 //Turn to required angle
  //break;
}



void follow_line(int power, int pos) {

  //Could be a lot better but works

  bool line_detected = detect_line();                           //Is line detected

  if ((millis() - line_pose_timer >= 5) && line_detected) {     //Update Pose
    line_pose_timer = millis();
    update_pose();
  }

  int line = get_line_position(last_dir, line_detected);        //Where is the line?

  pos = (115);                                                  //Follow this location
  if (line > 0 && line < 300) {

    last_dir = (line > pos) ? 1 : 0;                            //Where did I last see the line?
    int lp = constrain(abs(line_control.update(pos, line)), 0, power); //Set corrrection power
    int error = 1 * (pos - line);//lp = constrain(abs(error), 0, 45); //70 is smooth

    if (error>0) Move.Forward(power + lp, power - lp);         //Move in the right direction.
    else Move.Forward(power - lp, power + lp);
  }

  else if (line == 500)Move.Turn_Right(60, 60);                //Get back quick!!
  else if (line == 400)Move.Turn_Left(60, 60);
  if (line == 0) Move.Stop();

}



void only_follow_line() {
  while (1)
  {
    follow_line(40, 115);
    if (!start)break;
  }
}


/*
   Calculate deviation from Desired heading

   Current heading: yaw --> (-180--180)
   Theta: desired  --> (-180--180)

   Checks quadrant ++,-+,+-,--
   Subtracts or adds difference


*/

void cal_dev(int theta)            //Maps deviation from desired angle directly into motor power values.
{
  theta = (theta == 0) ? 1 : theta;     //Hack
  if ((yaw >= 0) && (theta <= 0)) {
    leftTurn  = yaw + abs(theta);
    rightTurn = 255 - leftTurn;    //255 Max power

  } else if ((yaw <= 0) && (theta >= 0)) {
    rightTurn = abs(yaw) + theta;
    leftTurn = 255 - rightTurn;

  } else if (((yaw >= 0) && (theta >= 0)) || ((yaw <= 0) && (theta <= 0))) {
    leftTurn = theta - yaw;

    if (leftTurn > 0) {
      rightTurn = leftTurn;
      leftTurn = 255 - rightTurn;

    } else if (leftTurn < 0) {
      leftTurn = abs(leftTurn);
      rightTurn = 255 - leftTurn;
    } else {
      leftTurn = rightTurn = 0;
    }
  }
}

void Turn(int y_theta) {

  leftTurn = 0;
  rightTurn = 0;
  int power = 0;
  bool turnEnable = true;
  unsigned long update_pose_timer = 0;

  do {
    if (millis() - update_pose_timer >= 40) {
      update_pose_timer = millis();
      update_pose();
    }

    cal_dev(y_theta);


    if ((leftTurn == 0) && (rightTurn == 0)) {
      Move.Stop();
      break;
    }

    else if (leftTurn < rightTurn) {
      //rightTurn = turn_PID(rightTurn);    //minimize error
      power = constrain(leftTurn, 30, 60);
      //rightTurn  = constrain(rightTurn, 30, 60);
      Move.Turn_Left(power, power);
    }
    else if (leftTurn > rightTurn) {
      //leftTurn = turn_PID(leftTurn);    //minimize error
      power  = constrain(rightTurn, 30, 60);
      //leftTurn = constrain(leftTurn, 30, 60);
      Move.Turn_Right(power, power);
    }
    /*
      Serial.print(rightTurn);
      Serial.print("\t");
      Serial.print(leftTurn);
      Serial.print("\t");
      Serial.print(yaw);
      Serial.print("\t");
      Serial.println(y_theta);
    */
  } while (turnEnable);
  turnEnable = false;
  Move.Stop();
  delay(800);
}

int bound(int n)
{
  int tmax = 60;
  int tmin = 20;
  if (n > tmax)
    return tmax;
  else if (n < tmin)
    return tmin;
  else
    // return tmax - (n - tmin); //invert
    return n;
}

void calibrate_turns() {
  Turn(90);
  Turn(180);
  //Turn(130);
  Turn(-90);
  Turn(0);
  Serial.println(yaw);

}

void find_line_by_dS(int power) { //120

  unsigned long update_pose_timer = 0;
  int maintain_yaw = yaw;

  while (!detect_line()) {                 //Keep going forward until line is not detected.
    if (millis() - update_pose_timer >= 10) {
      update_pose_timer = millis();
      update_pose();
    }
    
    int lp = heading_control.update(maintain_yaw, yaw);
    //int lp = constrain(5*(yaw - mang),-30,30);
    if (lp > 0) Move.Forward(power + lp, power - lp);
    else Move.Forward(power - lp, power + lp);
    //Move.Forward(36,30);
  }
  Move.Stop();
  delay(1000);
  if(opposite_start)Turn(-10);opposite_start=0;

}



/*
    ADC Prescalar

   clk = 16 Mhz, 16/(128) -- 125 kHz ADC conversion takes 13 clocks
   125K/13 -- 9.6 kHz ~ analogRead()
   clk/16  --> 58.6 KHz sampling rate.

    ADCSRA (ADC Control and Status Register A)

    ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0

    ADPS2, ADPS1, ADPS0 -- 1|0|0 Division factor:16

    TCCR1B - Timer/Counter1 Control Register B
    Last 3 bits control prescalar (011) --> clk/64 (default) ~PWM: 490Hz Pin 9,10
    Set prescalar to (001) --> clk/1 (No prescaling) ~ 31kHz


*/

void speed_up()
{

  sbi(ADCSRA, ADPS2); //ADPS2 --> 1
  cbi(ADCSRA, ADPS1); //ADPS1 --> 0
  cbi(ADCSRA, ADPS0); //ADPS0 --> 0

  /*

  */
  //sbi(TCCR1B, CS10);
  //cbi(TCCR1B, CS11)
  //cbi(TCCR1B, CS12)
  TCCR1B = TCCR1B & B11111000 | B00000001;
}

void calibrate_sensors(int c_time, int pwm, bool dir) {
  unsigned long ms = millis();
  while ((ms + c_time) > millis()) {
    if (dir == RIGHT)  Move.Turn_Right(pwm, pwm);
    if (dir == LEFT)   Move.Turn_Left(pwm, pwm);

    int reading[3];
    for (int x = 0; x < 3; x++) {
      reading[x] = analogRead(line_sensor[x]);
      sens_min[x] = (reading[x] < sens_min[x]) ? reading[x] : sens_min[x];
      sens_max[x] = (reading[x] > sens_max[x]) ? reading[x] : sens_max[x];

    }

  }

  Move.Stop();



  for (int i = 0; i < 3; i++) {
    //Serial.print(sens_min[i]);
    //Serial.print("\t");
    //Serial.println(sens_max[i]);
  }
}

#endif
