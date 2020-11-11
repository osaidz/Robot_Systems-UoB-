#ifndef _LINESENSOR_H
#define _LINESENSOR_H

#define LINE_LEFT_PIN    A4
#define LINE_CENTRE_PIN  A3
#define LINE_RIGHT_PIN   A2

/*
 *  1. Scale all three sensors to remove sensor bias. 
 *     Reading = ((Current_reading - Min_reading)/(Max_reading - Min_reading)) * Scale --> (0-100)
 *     
 *  2. Weighted Mean to calculate line poition 
 *     Position = (S1*0 + S2*100 + S*200)/(S1+S2+S3) Scale --> (0-300)
 *   
 *  3. Position 115 detectes corners well.
 * 
 */


const int line_sensor[3] = {LINE_RIGHT_PIN, LINE_CENTRE_PIN, LINE_LEFT_PIN}; //Pin definitions 
int sens_max[3] = {0, 0, 0};             //Maximum sensor value
int sens_min[3] = {1023, 1023, 1023};    //Minimum sensor value
unsigned int sum_num = 0, sum_den = 0;   

unsigned long startMillis;
unsigned long elapsed_time;

bool last_dir = 0;                 //Direction to turn if line is lost.  
bool start_linetracking  = 0;      //Romi starts to track line when true. 
bool start_sweep;                  //Starts Line Sweeo timer. 

bool detect_line() {

  bool is_line_detected = 0;
  unsigned int reading = 0;
  unsigned int scaled_sensr[3] = {0};
  sum_num = 0, sum_den = 0;

  for (int x = 0; x < 3; x++) {
    reading = analogRead(line_sensor[x]);
    reading = reading > sens_min[x] ? reading : sens_min[x]; //deal with erronous readings
    scaled_sensr[x] =  reading - sens_min[x];  
    scaled_sensr[x] *= 100;                       // Scale reading to a max of 100
    scaled_sensr[x] /= (sens_max[x] - sens_min[x]);
    if (scaled_sensr[x] >= 80) {                 // Threshold to discard false positives
      is_line_detected = 1;
    }
    sum_num += scaled_sensr[x] * (x) * 100;
    sum_den += scaled_sensr[x];
  }

  return is_line_detected;


}

int get_line_position(bool last_dir, bool is_line_detected) {
  unsigned int line_position = 0;

  if (is_line_detected) {
    line_position = sum_num / sum_den;
    start_linetracking = 1;
  }
  else if (!is_line_detected && !start_linetracking) line_position = 0;


/*
 * Routine to find line by sweeping.  
 * Executes after line is lost. If line is 
 * not found after 2201 ms. Stops line tracking. 
 */

  if (!is_line_detected) {                 

    if (!start_sweep)startMillis = millis();
    elapsed_time =  millis() - startMillis;
    start_sweep =  (elapsed_time < 2201) ? 1 : 0;

    if (elapsed_time > 550) {
      last_dir = !last_dir;

    }

    if (elapsed_time > 1650) {  //550*3
      last_dir = !last_dir;
    }

    if (elapsed_time > 2200) {  //550*4
      start_linetracking = 0;
    }

  }
  else {
    elapsed_time = 0;                      //Reset timer 
    startMillis  = 0;
    start_sweep = 0;
  }



  if (!is_line_detected && start_linetracking && last_dir)line_position = 400;        //Find line by turning opposite to the direction line was lost. 
  else if (!is_line_detected && start_linetracking && !last_dir)line_position = 500;
  return line_position;


}




#endif
