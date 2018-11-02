/*
 * PidController.cpp
 *
 *  Created on: 23 Feb 2017
 *      Author: sleutene
 */

#include <controller/PidController.hpp>
#include <stdexcept>

namespace controller {

// Set the controller parameters
void PidController::setParameters(const Parameters & parameters)
{
  parameters_ = parameters;
}

// Implements the controller as u(t)=c(e(t))
// c here refers to the "control" function.
double PidController::control(uint64_t timestampMicroseconds, double e_t,
                              double e_dot)
{
    double u = 0.0;
    bool anti_reset_windup = false;

    // TODO: add error derivative smoothing if required ...
    u = parameters_.k_p * e_t + parameters_.k_i * integratedError_ + parameters_.k_d * e_dot;

    // "anti-reset windup"
    if( u < minOutput_ )
    {
      u = minOutput_;
      anti_reset_windup = true;
    } 
    else if( u > maxOutput_ )
    {
       u = maxOutput_;
       anti_reset_windup = true;
    }  
   


    if(!anti_reset_windup)
    {
       // Make sure to use time in seconds.
       uint64_t delta_t_microseconds = timestampMicroseconds - lastTimestampMicroseconds_ ;
       double delta_t = (double)delta_t_microseconds/1000000.0;

       if(delta_t > 0.1)
       {
           resetIntegrator();
       }else
       {
            integratedError_ = integratedError_ + (e_t * delta_t);
       }

  
    }

  // Update.
  lastTimestampMicroseconds_ = timestampMicroseconds;

  return u;

}




// Reset the integrator to zero again.
void PidController::setOutputLimits(double minOutput, double maxOutput)
{
  minOutput_ = minOutput;
  maxOutput_ = maxOutput;
}

/// \brief
void PidController::resetIntegrator()
{
  integratedError_ = 0.0;
}

}  // namespace arp
