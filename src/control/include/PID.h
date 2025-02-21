#ifndef PID_H
#define PID_H

class PID {

public:

    /*

    * Constructor

    */

    PID();

     /*

     * Destructor.

     */

     virtual ~PID();

  /*

     * Initialize PID.

     */

     void setPID(double Kp, double Ki, double Kd);

     /*

  * Update the PID error variables given cross track error.

     * Calculate the total PID error.

     */

     double Control(double cte);

  

 private:

     /*

     * Errors

     */

     double error_proportional_;

     double error_integral_;

     double error_derivative_;

     /*

     * Coefficients

     */

     double Kp_;

     double Ki_;

     double Kd_;

 };

 

 PID::PID():error_proportional_(0.0),error_integral_(0.0),error_derivative_(0.0){}

 

 PID::~PID(){}

 

 void PID::setPID(double Kp, double Ki, double Kd){

    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;

 }

 

 double PID::Control(double cte) {

    error_integral_     += cte;

    error_derivative_    = cte - error_proportional_;

    error_proportional_  = cte;

    return -(Kp_ * error_proportional_ + Ki_ * error_integral_ + Kd_ * error_derivative_);

 }

 #endif /* PID_H */