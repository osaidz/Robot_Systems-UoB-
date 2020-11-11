/*
       @@@@@@@@@@@&*           %@@@@@%       @@@@@@@@@    @@@@@@@@@  @@@@@@@@
       @@@@@@@@@@@@@@@     #@@@@@@@@@@@@    @@@@@@@@@@   @@@@@@@@@* @@@@@@@@@
       @@@@@@   @@@@@@   /@@@@@%  .@@@@@@    @@@/@@@@@ @@@@@@@@@@    @@@@@@
      &@@@@@##&@@@@@@   @@@@@@(   @@@@@@@   @@@,.@@@@@@@@,.@@@@@    @@@@@@
      @@@@@@@@@@@@@    &@@@@@@    @@@@@@   @@@@  @@@@@@@  @@@@@    (@@@@@
     @@@@@@  @@@@@@*   @@@@@@    @@@@@@   .@@@   @@@@@#  @@@@@@    @@@@@&
   @@@@@@@@   @@@@@@%  .@@@@@@@@@@@@@    @@@@@%  @@@@  @@@@@@@@  @@@@@@@@
  %@@@@@@@&   @@@@@@     #@@@@@@@@      @@@@@@   @@@   @@@@@@@/ @@@@@@@@%

  CodeStub provided by Paul O'Dowd
  -------------------------------------------------------------------------
  Course: Robot Systems, University of Bristol
  Author: Osaid Zahid
  //sometimes I believe compiler ignores all my comments
*/

#include "drive.h" //Magic

#define DEBUG true // flag to turn on/off debugging
#define Serial if(DEBUG)Serial
//Serial.begin()-->922 bytes sketch, 179 bytes SRAM


void setup() {
  Initialise();     //Initialise I/O
  speed_up();       //Faster I/O by changing prescalar, details mentioned in function.
  delay(1000);      //wait
  unsigned long update_pose_timer = 0;

  while (1)
  {

    //Keep going forward until line is not detected.

    start_routine();    //Checks if Button 2 is pressed.


    if (start) {        //It is.



      counter += 1;     //Counts the number of times outside line
      //calibrate_turns();    //Just for testing purposes.
      //go_to_xy(0, 0, 0, 60);break;

      if (counter <= 2)find_line_by_dS(50); //Drive Striaght until line found, Input --> Power
      else {
        //delay(1000);
        go_to_xy(0, 0, 0, 55);              //Return to home position, Input --> xpos, ypos, theta,  Power
        break;                              //I have arrived! End loop
      }


      while (1)
      {
        follow_line(70, 115);              //Follow line, Input --> Power, Position
        if (!start_linetracking)break;      //Lost line after Sweep
      }

    }
    else  check_calib();                    //Checks if Button 1 is pressed in order to start calibration routine.

  }

  Serial.print("I'm out!");

}


void loop()
{
}
