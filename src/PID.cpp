#include "main.h"
#include "PID.h"

PID::PID(float isetpoint, float ikp, float iki, float ikd, float istarti, float isettle_time, float isettle_error, float itimeout) :
  setpoint(isetpoint),
  kp(ikp),
  ki(iki),
  kd(ikd),
  starti(istarti),
  settle_error(isettle_error),
  settle_time(isettle_time),
  timeout(itimeout)
{};

float PID::compute(float error) {
  if(fabs(error) < starti) {
    accumulated_error += error;
  }

  if ((error > 0 && previous_error < 0) || (error < 0 && previous_error > 0)) {
    accumulated_error = 0;
  }

  output = (kp * error) + (ki * accumulated_error) + (kd * (error - previous_error));
  //printf("P: %f, I: %f, D: %f \n", kp*error, ki*accumulated_error, kd*(error-previous_error));
  previous_error = error;

  if(fabs(error) < settle_error) {
    time_spent_settled += 10;
  } else {
    time_spent_settled = 0;
  }

  time_spent_running += 10;

  return output;
}

bool PID::is_settled(){
  if (time_spent_running>timeout && timeout != 0){
    previous_error = 0;
    return(true);
  }
  if (time_spent_settled>settle_time){
    previous_error = 0;
    return(true);
  }
  return(false);
}