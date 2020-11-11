#ifndef _MOTOR_H
#define _MOTOR_H


#define L_PWM_PIN 10 //Left
#define L_DIR_PIN 16

#define R_PWM_PIN  9 //Right
#define R_DIR_PIN 15

#define RIGHT 1
#define LEFT  0


class Move {
  public:
  
    void Left_Forward(byte pwm)
    { analogWrite(L_PWM_PIN, pwm); digitalWrite(L_DIR_PIN, LOW);
    }
    void Left_Reverse(byte pwm)
    { analogWrite(L_PWM_PIN, pwm); digitalWrite(L_DIR_PIN, HIGH);
    }
    void Right_Forward(byte pwm)
    { analogWrite(R_PWM_PIN, pwm); digitalWrite(R_DIR_PIN, LOW);
    }
    void Right_Reverse(byte pwm)
    { analogWrite(R_PWM_PIN, pwm); digitalWrite(R_DIR_PIN, HIGH);
    }
    
    void Forward(byte pwm_l, byte pwm_r)
    { Left_Forward(pwm_l); Right_Forward(pwm_r);
    }

    void Reverse(byte pwmL, byte pwmR)
    { Left_Reverse(pwmL); Right_Reverse(pwmR);
    }

    void Turn_Right(byte pwm_l, byte pwm_r)
    { Left_Forward( pwm_l); Right_Reverse(pwm_r);
    }

    void Turn_Left(byte pwm_l, byte pwm_r)
    { Right_Forward(pwm_r); Left_Reverse(pwm_l);
    }

    void Stop()
    { digitalWrite(L_PWM_PIN, LOW); digitalWrite(L_DIR_PIN, LOW);
      digitalWrite(R_PWM_PIN, LOW); digitalWrite(R_DIR_PIN, LOW);
    }

  

} Move;


#endif
