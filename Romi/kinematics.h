#ifndef _KINEMATICS_H
#define _KINEMATICS_H

//Just some basic math


#define deg_rad PI/180
#define rad_deg 180/PI

const byte  Wheel_Dia = 73; // In milimeters, adjusted via trial and error, 75 // 5ms--102
const byte  Wheel_Seperation = 145; //137 135
const int   Counts_Per_Revolution =  1440;
const float MM_Per_Count = 1 / (Counts_Per_Revolution / (Wheel_Dia*PI));  //Milimeter travelled per count
const float ang_multiplier = rad_deg * (MM_Per_Count / Wheel_Seperation); //Scale converts encoder counts to degrees 

float xpos  = 0;
float ypos  = 0;
float theta = 0;
int   x = 0, y = 0, yaw  = 0;

float dist_away  = 0;
float angle_away = 0;

long old_count_e1;
long old_count_e0;


//Approximate Kinematics, Assuming straight line motion.

/*To-do: Maintain velocity 
 * 
 * Serious drawback of not maintaining velocity is that theta(current heading)
 * has to be recalibrated if battery level changes. 
 * 
 */

void update_pose()
{
  long delta_1 = count_e1 - old_count_e1;
  long delta_0 = count_e0 - old_count_e0;

  float avg_dist = ((delta_1 + delta_0) / 2) * MM_Per_Count;

//  noInterrupts(); // Disable interrupts while calculating
  theta = (theta + ((delta_1 - delta_0) * ang_multiplier));
//  interrupts();

  if (theta > 360)theta -= 360;
  else if (theta < 0)theta += 360;

  xpos = xpos + avg_dist * cos(theta * deg_rad);
  ypos = ypos + avg_dist * sin(theta * deg_rad);

  x = (int)(-xpos);
  y = (int)(-ypos);



  yaw = (int)theta; //% 360;
  yaw = (yaw > 180) ? yaw - 360 : yaw; //Maps angles to atan2 range, easier to go home



  old_count_e1 = count_e1;
  old_count_e0 = count_e0;

  Serial.print(x);
  Serial.print("\t");
  Serial.print(y);
  Serial.print("\t");
  Serial.print(yaw);
  Serial.print("\t");
  Serial.print(dist_away);
  Serial.print("\t");
  Serial.println(angle_away);



}

void get_dist(int xd, int yd) {

  float deltaX = xd - (x);
  float deltaY = yd - (y);
  dist_away = sqrt((deltaX * deltaX) + (deltaY * deltaY));
  angle_away = (atan2(deltaY, deltaX) * 180 / PI);

}

//To-do

void ICC_update() {




}


#endif
