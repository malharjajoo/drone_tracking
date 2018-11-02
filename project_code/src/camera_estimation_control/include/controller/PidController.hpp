/*
 * PidController.hpp
 *
 *  Created on: 23 Feb 2017
 *      Author: sleutene
 */

#ifndef PIDCONTROLLER_HPP_
#define PIDCONTROLLER_HPP_

#include <cstdint>
#include <iostream>

namespace controller {

class PidController
{
 public:

  /// \brief A simple struct holding the controller parameters.
  struct Parameters
  {
    double k_p = 0.0;  ///< Proportional gain.
    double k_i = 0.0;  ///< Integral gain.
    double k_d = 0.0;  ///< Differential gain.
  };

  /// \brief Set the controller parameters
  void setParameters(const Parameters & parameters);


  void printParameters()
  {
     std::cout << "\n params = (" << parameters_.k_p << "," << parameters_.k_i << "," << parameters_.k_d << ")\n"; 
  }
  
  /// \brief Implements the controller as u(t)=c(e(t))
  /// @param[in]  timestampMicroseconds  The current time [us].
  /// @param[in]  e     The error signal e=r-y.
  /// @param[in]  e_dot The time derivative of e, de/dt.
  /// \return The control output/system input u.
  double control(uint64_t timestampMicroseconds, double e, double e_dot);
 
  /// \brief Reset the integrator to zero again.
  void resetIntegrator();

  /// \brief Set output limits
  /// @param[in] maxOutput The maximum output.
  /// @param[in] minOutput The minimum output.
  void setOutputLimits(double minOutput, double maxOutput);

 protected:
  Parameters parameters_;  ///< Holding the controller parameters.
  double integratedError_ = 0.0;  ///< The integrated error signal
  double maxOutput_ = 0.0;  ///< The maximum admissible output.
  double minOutput_ = 0.0;  ///< The minimum admissible output.
  uint64_t lastTimestampMicroseconds_ = 0;  ///< timestamp when last called control()
};

}  // namespace arp

#endif /* ARDRONE_PRACTICALS_INCLUDE_ARP_PIDCONTROLLER_HPP_ */
