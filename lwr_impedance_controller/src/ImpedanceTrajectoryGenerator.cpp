
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

// Start of user code includes
#include <geometry_msgs/typekit/Types.h>
#include <lwr_impedance_controller/CartImpTrajectory.h>
#include <lwr_impedance_controller/CartesianImpedance.h>
// End of user code

class ImpedanceTrajectoryGenerator : public RTT::TaskContext {
public:
  ImpedanceTrajectoryGenerator(const std::string & name) : TaskContext(name) {
    CartesianPosition_trig = false;
    ImpedanceTrajectory_trig = false;

  }

  ~ImpedanceTrajectoryGenerator(){
  }

  bool configureHook() {
  	this->ports()->addEventPort("CartesianPosition", port_CartesianPosition, boost::bind(&ImpedanceTrajectoryGenerator::CartesianPosition_onData, this, _1)).doc("");
  	this->ports()->addEventPort("ImpedanceTrajectory", port_ImpedanceTrajectory, boost::bind(&ImpedanceTrajectoryGenerator::ImpedanceTrajectory_onData, this, _1)).doc("");

	this->ports()->addPort("CartesianPositionCommand", port_CartesianPositionCommand).doc("");
	this->ports()->addPort("CartesianWrenchCommand", port_CartesianWrenchCommand).doc("");
	this->ports()->addPort("CartesianImpedanceCommand", port_CartesianImpedanceCommand).doc("");


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

	if(port_CartesianPosition.read(setpoint_.pose) == RTT::NoData) {
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
	if(valid_trajectory_ == true) {
	  ///do interpolation
	  if((trajectory_.header.stamp + trajectory_.trajectory[trajectory_index_].time_from_start) <= ros::Time::now() ) {
	    if(trajectory_.trajectory.size() <= (trajectory_index_ + 1)) {
	     // setpoint_ = trajectory_.trajectory[trajectory_index_];
	      valid_trajectory_ = false;
	    } else {
	      last_point_ = trajectory_.trajectory[trajectory_index_];
	      ++trajectory_index_;
        }
	  }

	  if(valid_trajectory_ == true) {
//	    setpoint_ = sampleInterpolation();
	  }

	  ++time_;
	}

	port_CartesianPositionCommand.write(setpoint_.pose);
	port_CartesianImpedanceCommand.write(setpoint_.impedance);
	port_CartesianWrenchCommand.write(setpoint_.wrench);

	// End of user code
  }
  void doNewTrajectory() {
    // Start of user code NewTrajectory
	// TODO Put implementation of handler here !!!
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

  bool CartesianPosition_trig;
  bool ImpedanceTrajectory_trig;

  // Start of user code userData
  unsigned int time_, trajectory_index_;
  bool valid_trajectory_;
  lwr_impedance_controller::CartImpTrajectory trajectory_;
  lwr_impedance_controller::CartImpTrajectoryPoint setpoint_, last_point_;
  // End of user code

};

ORO_CREATE_COMPONENT(ImpedanceTrajectoryGenerator)

