#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) 
{
	this->Kp_ = Kp;
	this->Ki_ = Ki;
	this->Kd_ = Kd;
	p_error = 0;
  	i_error = 0;
  	d_error = 0;
}

void PID::UpdateError(double cte) 
{
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
}

double PID::TotalError() 
{
	return Kp_ * p_error + Ki_ * i_error + Kd_ * d_error;
}

void PID::Reset() 
{
	p_error = 0;
  	i_error = 0;
  	d_error = 0;
}


