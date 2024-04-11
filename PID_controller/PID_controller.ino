
/*
* PID Controller Example
* By: Ian Searle


*/



#include <Servo.h>
Servo myservo;

#define KP_pin A0
#define KD_pin A1
#define KI_pin A2
#define set_pin A5
#define actual_pin A4
#define output_pin 9

#define time_step 100


double error, int_sum, last_error;
double Kp, Ki, Kd;

void setup() {
  // put your setup code here, to run once:
  error = 0;
  int_sum = 0;
  last_error = 0;
  myservo.attach(output_pin);
  pinMode(2, INPUT);
}

void loop() {
  if (digitalRead(2)) {
    int actual_pos = analogRead(actual_pin);
    int set_pos = analogRead(set_pin);
    error = set_pos - actual_pos;
    double Kp = KP_calc();
    double Ki = KI_calc(int_sum, error);
    double Kd = KD_calc(error, last_error);
    double sum = (Kp + Ki + Kd) / (double)1023;

    myservo.write(sum);
    delay(time_step);

  } else {
    myservo.write(0);
  }
}




double KP_calc() {
  return analogRead(KP_pin);
}
double KD_calc(double curr_error, double last_error) {
  double error_dif = curr_error - last_error;
  double KD = analogRead(KD_pin);
  return KD * (error_dif / time_step);
}
double KI_calc(double int_sum, double error) {
  double KI = analogRead(KI_pin);
  return KI * (int_sum + (error * time_step));
}

/* convert values from pot to servo */

/*
* PID Controller Example
* By: Ian Searle


*/



#include <Servo.h>
Servo myservo;

#define KP_pin A0
#define KD_pin A1
#define KI_pin A2
#define set_pin A5
#define actual_pin A4
#define output_pin 9

#define time_step 100


double error, int_sum, last_error;
double Kp, Ki, Kd;

void setup() {
  // put your setup code here, to run once:
  error = 0;
  int_sum = 0;
  last_error = 0;
  myservo.attach(output_pin);
  pinMode(2, INPUT);
}

void loop() {
  if (digitalRead(2)) {
    int actual_pos = analogRead(actual_pin);
    int set_pos = analogRead(set_pin);
    error = set_pos - actual_pos;
    double Kp = KP_calc();
    double Ki = KI_calc(int_sum, error);
    double Kd = KD_calc(error, last_error);
    double sum = (Kp + Ki + Kd) / (double)1023;

    myservo.write(sum);
    delay(time_step);

  } else {
    myservo.write(0);
  }
}




double KP_calc() {
  return analogRead(KP_pin);
}
double KD_calc(double curr_error, double last_error) {
  double error_dif = curr_error - last_error;
  double KD = analogRead(KD_pin);
  return KD * (error_dif / time_step);
}
double KI_calc(double int_sum, double error) {
  double KI = analogRead(KI_pin);
  return KI * (int_sum + (error * time_step));
}

/* convert values from pot to servo */
