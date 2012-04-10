/*
 * SpringCoupler.cpp
 *
 *  Created on: 31-08-2011
 *      Author: konradb3
 */

#include <rtt/Component.hpp>

#include "CartWrench.h"
#include <tf_conversions/tf_kdl.h>

namespace lwr {

CartWrench::CartWrench(const std::string& name) : TaskContext(name, PreOperational) {
  // command inputs
  this->addPort("CartesianWrenchCommand", port_cartesian_wrench_command);
  this->addPort("CartesianImpedanceCommand", port_cartesian_impedance_command);
  this->addPort("CartesianPositionCommand", port_cartesian_position_command);

  this->addPort("DesiredCartesianPosition", port_desired_cartesian_position);
  this->addPort("ToolFrame", port_tool_frame);

  this->addEventPort("CommandPeriod", port_command_period);

  // meansurment inputs
  this->addPort("Jacobian", port_jacobian);
  this->addPort("CartesianPosition", port_cart_pos_msr);
  this->addPort("JointPosition", port_jnt_pos_msr);
  // commands outputs
  this->addPort("JointEffortCommand", port_jnt_trq_cmd);
  this->addPort("FriJointImpedance", port_joint_impedance_command);
  this->addPort("JointPositionCommand", port_joint_position_command);

}

CartWrench::~CartWrench() {

}

bool CartWrench::configureHook() {
	return true;
}

void CartWrench::cleanupHook() {

}

bool CartWrench::startHook() {

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

	//set commanded force to zero
	cartesian_wrench_command.force.x = 0.0;
	cartesian_wrench_command.force.y = 0.0;
	cartesian_wrench_command.force.z = 0.0;
	cartesian_wrench_command.torque.x = 0.0;
	cartesian_wrench_command.torque.y = 0.0;
	cartesian_wrench_command.torque.z = 0.0;
  
	if(port_cart_pos_msr.read(cart_pos_msr) == RTT::NoData) {
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
	
	tf::PoseMsgToKDL(tool_frame_msg, tool_frame);
	tf::PoseMsgToKDL(cart_pos_msr, cart_pos_ref);
  cart_pos_ref = cart_pos_ref * tool_frame;
	cart_pos_old = cart_pos_ref;

	for(size_t i = 0; i < 7; i++) {
	  imp.stiffness[i] = 0.0;
	  imp.damping[i] = 0.0;
	}
   
	port_joint_impedance_command.write(imp);

	return true;
}

void CartWrench::stopHook() {

  port_jnt_pos_msr.read(jnt_pos);
  port_joint_position_command.write(jnt_pos);

	for(size_t i = 0; i < 7; i++) {
	  imp.stiffness[i] = 200.0;
	  imp.damping[i] = 0.8;
	}
	
	port_joint_impedance_command.write(imp);
}

void CartWrench::updateHook() {

  port_joint_impedance_command.write(imp);

  port_command_period.read(dt);

  port_jacobian.read(jacobian);
  port_cartesian_impedance_command.read(cartesian_impedance_command);
  port_cartesian_wrench_command.read(cartesian_wrench_command);
  
  if(port_cartesian_position_command.read(cart_pos_cmd) == RTT::NewData){
    tf::PoseMsgToKDL(cart_pos_cmd, cart_pos_ref);
  }
  
  port_cart_pos_msr.read(cart_pos_msr);

  KDL::Frame pos_msr, spring;

  tf::PoseMsgToKDL(cart_pos_msr, pos_msr);

  pos_msr = pos_msr * tool_frame;

  // calculate stiffness component

  spring = pos_msr.Inverse() * cart_pos_ref;

  KDL::Twist tw = KDL::diff(KDL::Frame::Identity(), spring);

  Eigen::VectorXd trq(7);
  Eigen::VectorXd wrench2(6);

  KDL::Wrench wrench;

  wrench(0) = tw.vel.x() * cartesian_impedance_command.stiffness.force.x;
  wrench(1) = tw.vel.y() * cartesian_impedance_command.stiffness.force.y;
  wrench(2) = tw.vel.z() * cartesian_impedance_command.stiffness.force.z;
  wrench(3) = tw.rot.x() * cartesian_impedance_command.stiffness.torque.x;
  wrench(4) = tw.rot.y() * cartesian_impedance_command.stiffness.torque.y;
  wrench(5) = tw.rot.z() * cartesian_impedance_command.stiffness.torque.z;

  // calculate damping component
  KDL::Frame cart_pos_diff = cart_pos_old.Inverse() * pos_msr;
  cart_pos_old = pos_msr;

  KDL::Twist vel = KDL::diff(KDL::Frame::Identity(), cart_pos_diff);

  wrench(0) -= ((vel.vel.x() / dt)) * cartesian_impedance_command.damping.force.x;
  wrench(1) -= ((vel.vel.y() / dt)) * cartesian_impedance_command.damping.force.y;
  wrench(2) -= ((vel.vel.z() / dt)) * cartesian_impedance_command.damping.force.z;
  wrench(3) -= ((vel.rot.x() / dt)) * cartesian_impedance_command.damping.torque.x;
  wrench(4) -= ((vel.rot.y() / dt)) * cartesian_impedance_command.damping.torque.y;
  wrench(5) -= ((vel.rot.z() / dt)) * cartesian_impedance_command.damping.torque.z;

	//add additional force/torque
  wrench(0) += cartesian_wrench_command.force.x;
  wrench(1) += cartesian_wrench_command.force.y;
  wrench(2) += cartesian_wrench_command.force.z;
  wrench(3) += cartesian_wrench_command.torque.x;
  wrench(4) += cartesian_wrench_command.torque.y;
  wrench(5) += cartesian_wrench_command.torque.z;

  wrench = tool_frame * wrench;

  for(size_t i = 0; i < 6; i++)
    wrench2(i) = wrench(i);

  trq = jacobian.data.transpose() * wrench2;

  for(size_t i = 0; i < 7; i++)
    jnt_trq_cmd[i] = trq(i);

//  RTT::Logger::log(RTT::Logger::Error) << "wrench : " << wrench[0] << " : " << wrench[1] << " : " << wrench[2] << " : " << wrench[3] << " : " << wrench[4] << " : " << wrench[5] << RTT::endlog();

  port_jnt_trq_cmd.write(jnt_trq_cmd);
  
  geometry_msgs::Pose desired_pos;
  
  tf::PoseKDLToMsg(cart_pos_ref, desired_pos);
  port_desired_cartesian_position.write(desired_pos);

}

}
ORO_CREATE_COMPONENT(lwr::CartWrench)
