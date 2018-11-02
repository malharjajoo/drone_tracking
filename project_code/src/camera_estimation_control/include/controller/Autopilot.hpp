/*
 * Autopilot.hpp
 *
 *  Created on: 10 Jan 2017
 *      Author: sleutene
 */

#ifndef ARDRONE_PRACTICALS_INCLUDE_ARP_AUTOPILOT_HPP_
#define ARDRONE_PRACTICALS_INCLUDE_ARP_AUTOPILOT_HPP_

#include <mutex>
#include <Eigen/Core>
#include <atomic>
#include <deque>

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <fstream>
#include <controller/PidController.hpp>




#include "State.hpp"


namespace controller{

  /// \brief A Helper struct to send lists of waypoints.
  class Waypoint {

    public:

    double x; ///< The World frame x coordinate.
    double y; ///< The World frame y coordinate.
    double z; ///< The World frame z coordinate.
    double yaw; ///< The yaw angle of the robot w.r.t. the World frame.
    double posTolerance; ///< The position tolerance: if within, it's considered reached.

    Waypoint(double x, double y, double z, double yaw, double posTolerance):
    x(x), y(y), z(z), yaw(yaw), posTolerance(posTolerance)
    {}

  };



/// \brief The autopilot highlevel interface for commanding the drone manually or automatically.
class Autopilot {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Autopilot(ros::NodeHandle& nh);

    /// \brief These are reverse engineered AR Drone states.
    enum DroneStatus {
    Unknown = 0,
    Inited = 1,
    Landed = 2,
    Flying = 3,
    Hovering = 4,
    Test = 5, // ?
    TakingOff = 6,
    Flying2 = 7,
    Landing = 8,
    Looping = 9 // ?
    };


    void setInitialPIDValues_Trackbar();
    void setInitialPIDValues();
    
    void setPIDParams_Trackbar();

    /// \brief Get the drone status.
    /// \return The status.
    DroneStatus droneStatus();


    /// \brief Request flattrim calibration.
    /// \return True on success.
    /// \note This will only work when landed on ground.
    bool flattrimCalibrate();

    /// \brief Takeoff.
    /// \return True on success.
    /// \note This will only work when landed on ground.
    bool takeoff();

    void hover();
    
    /// \brief Land.
    /// \return True on success.
    /// \note This will only work when in flight.
    bool land();

    /// \brief Get the Battery Percentage.
    /// \return The Percentage
    float batteryStatus();

    /// \brief Turn off all motors and reboot.
    /// \return True on success.
    /// \warning When in flight, this will let the drone drop down to the ground.
    bool estopReset();

    /// \brief Move the drone manually.
    /// @param[in] forward Forward tilt [-1,...,1] scaling the maximum tilt ROS parameter.
    /// @param[in] left Left tilt [-1,...,1] scaling the maximum tilt ROS parameter.
    /// @param[in] up Upward velocity [-1,...,1] scaling the maximum vertical speed ROS parameter.
    /// @param[in] rotateLeft Left yaw velocity [-1,...,1] scaling the maximum yaw rate ROS parameter.
    /// \return True on success.
    /// \note This will only do something when in manual mode and flying.
    bool manualMove(double forward, double left, double up, double rotateLeft);

    /// \brief Are we currently in automatic mode?;
    bool isAutomatic() { return isAutomatic_; }

    /// \brief Set to automatic control mode.
    void setManual();

    // We dont just set the one that has changed, simply set all of them.
    // This may sound redundant but it is better than having a callback for 
    // each pid parameter.
    static void onChange(int v, void* ptr)
    {
        Autopilot* that = (Autopilot*)ptr;
        that->setPIDParams_Trackbar();
    }

    void createPIDTrackBars();

    /// \brief Set to manual control mode.
    void setAutomatic();

    void createWaypointList(std::string filename);
    void printWaypointList();

    /// \brief How many waypoints still have to be flown to?
    /// \return The number of waypoints still not reached.
    int waypointsLeft() 
    {
        std::lock_guard<std::mutex> l(waypointMutex_);
        return waypoints_.size();
    }

    void pidControl(uint64_t timestampMicroseconds ,State stateEstimate);

    std::deque<Waypoint> waypoints_;  ///< A list of waypoints that will be approached, if not empty.


 protected:
    /// \brief Move the drone.
    /// @param[in] forward Forward tilt [-1,...,1] scaling the maximum tilt ROS parameter.
    /// @param[in] left Left tilt [-1,...,1] scaling the maximum tilt ROS parameter.
    /// @param[in] up Upward velocity [-1,...,1] scaling the maximum vertical speed ROS parameter.
    /// @param[in] rotateLeft Left yaw velocity [-1,...,1] scaling the maximum yaw rate ROS parameter.
    /// \return True on success.
    bool move(double forward, double left, double up, double rotateLeft);

    /// \brief Obtain the last navdata package (callback).
    /// @param[out] navdata The navdata message.
    void navdataCallback(const ardrone_autonomy::NavdataConstPtr& msg);


    ros::NodeHandle * nh_;  ///< ROS node handle.
    ros::Publisher pubReset_;  ///< The reset publisher -- use to reset the drone (e-stop).
    ros::Publisher pubTakeoff_;  ///< Publish to take-off the drone.
    ros::Publisher pubLand_;  ///< Publish to land the drone.
    ros::Publisher pubMove_;  ///< Publish to move the drone.
    ros::ServiceClient srvFlattrim_;  ///< To request a flat trim calibration.
    ardrone_autonomy::Navdata lastNavdata_; ///< Store navdata as it comes in asynchronously.
    std::mutex navdataMutex_; ///< We need to lock navdata access due to asynchronous arrival.
    ros::Subscriber subNavdata_; ///< The subscriber for navdata.

    // TODO: check why this needs to be an atomic variable ?
    std::atomic<bool> isAutomatic_; ///< True, if in automatic control mode.

    
    // Currently all waypoints are pre-determined. However, if using a motion planner, waypoint
    // will come asynchronously and hence this is being used.
    std::mutex waypointMutex_;  

    // PID controllers for the drone
    controller::PidController pidPitch_;    // for forward/backward movement
    controller::PidController pidRoll_;     // for left/right movement
    controller::PidController pidVerticalSpeed_; // for up/down movement
    controller::PidController pidYawRate_;       // for yaw orientation.

};

} /// namespace controller


#endif /* ARDRONE_PRACTICALS_INCLUDE_ARP_AUTOPILOT_HPP_ */
