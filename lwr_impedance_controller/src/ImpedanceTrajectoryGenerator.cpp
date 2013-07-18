
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

// Start of user code includes
#include <geometry_msgs/Pose.h>
#include <lwr_impedance_controller/CartImpTrajectory.h>
#include <lwr_impedance_controller/CartesianImpedance.h>
#include <Eigen/Geometry>

#include "xeno_clock/xeno_clock.h"

// End of user code

class ImpedanceTrajectoryGenerator : public RTT::TaskContext {
public:
  ImpedanceTrajectoryGenerator(const std::string & name) : TaskContext(name) {
    CartesianPosition_trig = false;
    ImpedanceTrajectory_trig = false;

	

    this->ports()->addEventPort("CartesianPosition", port_CartesianPosition, boost::bind(&ImpedanceTrajectoryGenerator::CartesianPosition_onData, this, _1)).doc("");
    this->ports()->addEventPort("ImpedanceTrajectory", port_ImpedanceTrajectory, boost::bind(&ImpedanceTrajectoryGenerator::ImpedanceTrajectory_onData, this, _1)).doc("");

    this->ports()->addPort("CartesianPositionCommand", port_CartesianPositionCommand).doc("");
    this->ports()->addPort("CartesianWrenchCommand", port_CartesianWrenchCommand).doc("");
    this->ports()->addPort("CartesianImpedanceCommand", port_CartesianImpedanceCommand).doc("");
    this->ports()->addPort("Tool", port_Tool).doc("");
  }

  ~ImpedanceTrajectoryGenerator(){
  }

  bool configureHook() {
    // Start of user code configureHook
    // TODO Put implementation of configureHook here !!!
    // End of user code
    return true;
  }

  bool startHook() {
    // Start of user code startHook
    setpoint_.impedance.stiffness.force.x = 1000;
    setpoint_.impedance.stiffness.force.y = 1000;
    setpoint_.impedance.stiffness.force.z = 1000;

    setpoint_.impedance.stiffness.torque.x = 100;
    setpoint_.impedance.stiffness.torque.y = 100;
    setpoint_.impedance.stiffness.torque.z = 100;

    setpoint_.impedance.damping.force.x = 0.7;
    setpoint_.impedance.damping.force.y = 0.7;
    setpoint_.impedance.damping.force.z = 0.7;

    setpoint_.impedance.damping.torque.x = 0.70;
    setpoint_.impedance.damping.torque.y = 0.70;
    setpoint_.impedance.damping.torque.z = 0.70;

    setpoint_.wrench.force.x = 0.0;
    setpoint_.wrench.force.y = 0.0;
    setpoint_.wrench.force.z = 0.0;

    setpoint_.wrench.torque.x = 0.0;
    setpoint_.wrench.torque.y = 0.0;
    setpoint_.wrench.torque.z = 0.0;

    if (port_CartesianPosition.read(setpoint_.pose) == RTT::NoData) {
      return false;
    }

    valid_trajectory_ = false;

    // End of user code
    return true;
  }

  void stopHook() {
    // Start of user code stopHook
    // TODO Put implementation of stopHook here !!!
    // End of user code
  }

  void updateHook() {
	if(CartesianPosition_trig &&  true) {
      doGenerate();
      CartesianPosition_trig = false;
    }
	if(ImpedanceTrajectory_trig &&  true) {
      doNewTrajectory();
      ImpedanceTrajectory_trig = false;
    }
  }

private: 

  void doGenerate() {
    // Start of user code Generate
    if (valid_trajectory_ == true) {
      ///do interpolation
      if ((trajectory_.header.stamp
          + trajectory_.trajectory[trajectory_index_].time_from_start)
          <= now()) {
        if (trajectory_.trajectory.size() <= (trajectory_index_ + 1)) {
          // setpoint_ = trajectory_.trajectory[trajectory_index_];
          valid_trajectory_ = false;
        } else {
          last_point_ = trajectory_.trajectory[trajectory_index_];
          ++trajectory_index_;
        }
      }

      if (valid_trajectory_ == true) {
        setpoint_ = sampleInterpolation();
      }
    }

    port_CartesianPositionCommand.write(setpoint_.pose);
    port_CartesianImpedanceCommand.write(setpoint_.impedance);
    port_CartesianWrenchCommand.write(setpoint_.wrench);

    // End of user code
  }
  void doNewTrajectory() {
    // Start of user code NewTrajectory
    if (port_ImpedanceTrajectory.read(trajectory_) == RTT::NewData) {
      if (trajectory_.trajectory.size() == 0) {
        valid_trajectory_ = false;
      } else {
        trajectory_index_ = 0;
        last_point_ = setpoint_;
        last_point_.time_from_start = ros::Duration(0);
//        std::cout << "initial pose : [ " << setpoint_.pose.position.x << " "
//            << setpoint_.pose.position.y << " " << setpoint_.pose.position.z
//            << " ]  [ " << setpoint_.pose.orientation.x << " "
//            << setpoint_.pose.orientation.y << " "
//            << setpoint_.pose.orientation.z << " "
//            << setpoint_.pose.orientation.w << " "
//            << now() << "  "
//            << trajectory_.header.stamp << " "
//            << now() - trajectory_.header.stamp << " ]" << std::endl;
        valid_trajectory_ = true;
      }
    }
    // End of user code
  }

  void CartesianPosition_onData(RTT::base::PortInterface* port) {
    CartesianPosition_trig = true;
  }
  void ImpedanceTrajectory_onData(RTT::base::PortInterface* port) {
    ImpedanceTrajectory_trig = true;
  }

  RTT::InputPort<geometry_msgs::Pose > port_CartesianPosition;
  RTT::InputPort<lwr_impedance_controller::CartImpTrajectory > port_ImpedanceTrajectory;

  RTT::OutputPort<geometry_msgs::Pose > port_CartesianPositionCommand;
  RTT::OutputPort<geometry_msgs::Wrench > port_CartesianWrenchCommand;
  RTT::OutputPort<lwr_impedance_controller::CartesianImpedance > port_CartesianImpedanceCommand;
  RTT::OutputPort<geometry_msgs::Pose > port_Tool;


  bool CartesianPosition_trig;
  bool ImpedanceTrajectory_trig;

  // Start of user code userData
  unsigned int trajectory_index_;
  bool valid_trajectory_;
  lwr_impedance_controller::CartImpTrajectory trajectory_;
  lwr_impedance_controller::CartImpTrajectoryPoint setpoint_, last_point_;

  lwr_impedance_controller::CartImpTrajectoryPoint sampleInterpolation() {
    lwr_impedance_controller::CartImpTrajectoryPoint next_point;

    double timeFromStart =
        (double) (now() - trajectory_.header.stamp).toSec();
    double segStartTime = last_point_.time_from_start.toSec();
    double segEndTime =
        trajectory_.trajectory[trajectory_index_].time_from_start.toSec();

    next_point = setpoint_;

    // interpolate position
    // x
    next_point.pose.position.x = linearlyInterpolate(timeFromStart,
        segStartTime, segEndTime, last_point_.pose.position.x,
        trajectory_.trajectory[trajectory_index_].pose.position.x);
    // y
    next_point.pose.position.y = linearlyInterpolate(timeFromStart,
        segStartTime, segEndTime, last_point_.pose.position.y,
        trajectory_.trajectory[trajectory_index_].pose.position.y);
    // z
    next_point.pose.position.z = linearlyInterpolate(timeFromStart,
        segStartTime, segEndTime, last_point_.pose.position.z,
        trajectory_.trajectory[trajectory_index_].pose.position.z);

    // interpolate orientation

    Eigen::Quaternion<double> start = Eigen::Quaternion<double>(last_point_.pose.orientation.w,
        last_point_.pose.orientation.x, last_point_.pose.orientation.y,
        last_point_.pose.orientation.z);
    Eigen::Quaternion<double> end = Eigen::Quaternion<double>(
        trajectory_.trajectory[trajectory_index_].pose.orientation.w,
        trajectory_.trajectory[trajectory_index_].pose.orientation.x,
        trajectory_.trajectory[trajectory_index_].pose.orientation.y,
        trajectory_.trajectory[trajectory_index_].pose.orientation.z);

    double t = linearlyInterpolate(timeFromStart, segStartTime, segEndTime, 0,
        1);

    Eigen::Quaternion<double> rot = start.slerp(t, end);

    next_point.pose.orientation.x = rot.x();
    next_point.pose.orientation.y = rot.y();
    next_point.pose.orientation.z = rot.z();
    next_point.pose.orientation.w = rot.w();


    /*
     // x
     next_point.pose.orientation.x = linearlyInterpolate(timeFromStart,
     segStartTime, segEndTime, last_point_.pose.orientation.x,
     trajectory_.trajectory[trajectory_index_].pose.orientation.x);
     // y
     next_point.pose.orientation.y = linearlyInterpolate(timeFromStart,
     segStartTime, segEndTime, last_point_.pose.orientation.y,
     trajectory_.trajectory[trajectory_index_].pose.orientation.y);
     // z
     next_point.pose.orientation.z = linearlyInterpolate(timeFromStart,
     segStartTime, segEndTime, last_point_.pose.orientation.z,
     trajectory_.trajectory[trajectory_index_].pose.orientation.z);
     // w
     next_point.pose.orientation.w = linearlyInterpolate(timeFromStart,
     segStartTime, segEndTime, last_point_.pose.orientation.w,
     trajectory_.trajectory[trajectory_index_].pose.orientation.w);
     */
    //interpolate stiffness
    // x
    next_point.impedance.stiffness.force.x = linearlyInterpolate(timeFromStart,
        segStartTime, segEndTime, last_point_.impedance.stiffness.force.x,
        trajectory_.trajectory[trajectory_index_].impedance.stiffness.force.x);

    next_point.impedance.stiffness.force.y = linearlyInterpolate(timeFromStart,
        segStartTime, segEndTime, last_point_.impedance.stiffness.force.y,
        trajectory_.trajectory[trajectory_index_].impedance.stiffness.force.y);

    next_point.impedance.stiffness.force.z = linearlyInterpolate(timeFromStart,
        segStartTime, segEndTime, last_point_.impedance.stiffness.force.z,
        trajectory_.trajectory[trajectory_index_].impedance.stiffness.force.z);

    next_point.impedance.stiffness.torque.x = linearlyInterpolate(timeFromStart,
        segStartTime, segEndTime, last_point_.impedance.stiffness.torque.x,
        trajectory_.trajectory[trajectory_index_].impedance.stiffness.torque.x);

    next_point.impedance.stiffness.torque.y = linearlyInterpolate(timeFromStart,
        segStartTime, segEndTime, last_point_.impedance.stiffness.torque.y,
        trajectory_.trajectory[trajectory_index_].impedance.stiffness.torque.y);

    next_point.impedance.stiffness.torque.z = linearlyInterpolate(timeFromStart,
        segStartTime, segEndTime, last_point_.impedance.stiffness.torque.z,
        trajectory_.trajectory[trajectory_index_].impedance.stiffness.torque.z);

    next_point.impedance.damping =
        trajectory_.trajectory[trajectory_index_].impedance.damping;
    next_point.wrench = trajectory_.trajectory[trajectory_index_].wrench;

    return next_point;
  }

  double linearlyInterpolate(double time,
                             double startTime,
                             double endTime,
                             double startValue,
                             double endValue) {
    return startValue
        + (time - startTime) * (endValue - startValue) / (endTime - startTime);
  }
  // End of user code

};

ORO_CREATE_COMPONENT(ImpedanceTrajectoryGenerator)

