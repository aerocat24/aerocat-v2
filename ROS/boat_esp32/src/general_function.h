#pragma once
#include "header.h"

void pwm_write(int pwm_left, int pwm_right){

    Serial.print("PWM left: ");
    Serial.println(pwm_left);
    Serial.print("PWM right: ");
    Serial.println(pwm_right);

    // aplica os pwm nas gpio do esp
    esc_left.write(pwm_left);
    esc_right.write(pwm_right);
}