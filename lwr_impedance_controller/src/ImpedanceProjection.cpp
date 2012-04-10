/*
 * ImpedanceProjection.cpp
 *
 *  Created on: 09-04-2012
 *      Author: konradb3
 */

#include <rtt/Component.hpp>

#include "ImpedanceProjection.h"
#include <tf_conversions/tf_kdl.h>

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 7, 7> Matrix7d;

namespace lwr {

ImpedanceProjector::ImpedanceProjector(const std::string& name) : TaskContext(name, PreOperational) {
  // command inputs
  this->addPort("CartesianImpedanceCommand", port_cartesian_impedance_command);
  this->addPort("ToolFrame", port_tool_frame);
  // meansurment inputs
  this->addEventPort("CommandPeriod", port_command_period);
  this->addPort("Jacobian", port_jacobian);
  this->addPort("JointPosition", port_jnt_pos_msr);
  // commands outputs
  this->addPort("JointEffortCommand", port_jnt_trq_cmd);
  this->addPort("FriJointImpedance", port_joint_impedance_command);
  this->addPort("JointPositionCommand", port_joint_position_command);

}

ImpedanceProjector::~ImpedanceProjector() {

}

bool ImpedanceProjector::configureHook() {
	return true;
}

void ImpedanceProjector::cleanupHook() {

}

bool ImpedanceProjector::startHook() {

	jnt_trq_cmd.resize(7);

	//set detault stiffness and damping
	cartesian_impedance_command.stiffness.force.x = 500.0;
	cartesian_impedance_command.stiffness.force.y = 500.0;
	cartesian_impedance_command.stiffness.force.z = 500.0;

	cartesian_impedance_command.stiffness.torque.x = 20.0;
	cartesian_impedance_command.stiffness.torque.y = 20.0;
	cartesian_impedance_command.stiffness.torque.z = 20.0;

	cartesian_impedance_command.damping.force.x = 40.0;
	cartesian_impedance_command.damping.force.y = 40.0;
	cartesian_impedance_command.damping.force.z = 40.0;

	cartesian_impedance_command.damping.torque.x = 0.80;
	cartesian_impedance_command.damping.torque.y = 0.80;
	cartesian_impedance_command.damping.torque.z = 0.80;

	if(port_jnt_pos_msr.read(jnt_pos) == RTT::NoData) {
	  RTT::Logger::log(RTT::Logger::Error) << "No position data " << RTT::endlog();
		return false;
	}
	
	if(port_command_period.read(dt) == RTT::NoData) {
	  RTT::Logger::log(RTT::Logger::Error) << "No comand period data " << RTT::endlog();
		return false;
	}
	
	geometry_msgs::Pose tool_frame_msg;
	
	if(port_tool_frame.read(tool_frame_msg) == RTT::NoData) {
	  RTT::Logger::log(RTT::Logger::Error) << "No tool data " << RTT::endlog();
	  return false;
	}

	return true;
}

void ImpedanceProjector::stopHook() {

}

void ImpedanceProjector::updateHook() {

  Matrix7d Kj, Dj;
  Vector6d Kc, Dc;
  Vector7d rt, dq;

  port_command_period.read(dt);
  port_jacobian.read(jacobian);
  port_cartesian_impedance_command.read(cartesian_impedance_command);
  port_jnt_pos_msr.read(jnt_pos);
  port_joint_position_command.read(jnt_pos_cmd);

  Kj(0) = cartesian_impedance_command.stiffness.force.x;
  Kj(1) = cartesian_impedance_command.stiffness.force.y;
  Kj(2) = cartesian_impedance_command.stiffness.force.z;
  
  Kj(3) = cartesian_impedance_command.stiffness.torque.x;
  Kj(4) = cartesian_impedance_command.stiffness.torque.x;
  Kj(5) = cartesian_impedance_command.stiffness.torque.x;

  Kj = jacobian.data.transpose() * Kc.asDiagonal() * jacobian.data;

  Dj(0) = cartesian_impedance_command.damping.force.x;
  Dj(1) = cartesian_impedance_command.damping.force.y;
  Dj(2) = cartesian_impedance_command.damping.force.z;
  
  Dj(3) = cartesian_impedance_command.damping.torque.x;
  Dj(4) = cartesian_impedance_command.damping.torque.x;
  Dj(5) = cartesian_impedance_command.damping.torque.x;

  Dj = jacobian.data.transpose() * Dc.asDiagonal() * jacobian.data;

  for(size_t i = 0; i < 7; i++) {
    imp.stiffness[i] = Kj(i, i);
    imp.damping[i] = Dj(i, i);
    Kj(i, i) = 0.0;
    Dj(i, i) = 0.0;
  }

  for(size_t i = 0; i < 7; i++) {
    dq(i) = jnt_pos_cmd[i] - jnt_pos[i]; 
  }

  rt = Kj * dq;

  for(size_t i = 0; i < 7; i++) {
    jnt_trq_cmd[i] = rt(i); 
  }

	port_joint_impedance_command.write(imp);
  port_jnt_trq_cmd.write(jnt_trq_cmd);
}

}
ORO_CREATE_COMPONENT(lwr::ImpedanceProjector)
