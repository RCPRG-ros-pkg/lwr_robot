/*
 * SpringCoupler.cpp
 *
 *  Created on: 31-08-2011
 *      Author: konradb3
 */

#include <rtt/Component.hpp>

#include "VelocityProjection.h"
#include <tf_conversions/tf_kdl.h>
#include <Eigen/Dense>

namespace lwr {

VelocityProjector::VelocityProjector(const std::string& name) : TaskContext(name, PreOperational) {
  // command inputs

  this->addPort("CartesianVelocityCommand", port_cartesian_velocity_command);
  this->addPort("NullspaceVelocityCommand", port_nullspace_velocity_command);
  this->addPort("MassMatrix", port_mass_matrix);
  this->addPort("ToolFrame", port_tool_frame);
  this->addEventPort("CommandPeriod", port_command_period);
  // meansurment inputs
  this->addPort("Jacobian", port_jacobian);
  this->addPort("DesiredJointPosition", port_jnt_pos_des);
  // commands outputs
  this->addPort("JointPositionCommand", port_joint_position_command);
}

VelocityProjector::~VelocityProjector() {

}

bool VelocityProjector::configureHook() {
	return true;
}

void VelocityProjector::cleanupHook() {

}

bool VelocityProjector::startHook() {

  jnt_pos_cmd.resize(7);

	if(port_command_period.read(dt) == RTT::NoData) {
	  RTT::Logger::log(RTT::Logger::Error) << "No comand period data " << RTT::endlog();
		return false;
	}
	
	geometry_msgs::Pose tool_frame_msg;
	
	if(port_tool_frame.read(tool_frame_msg) == RTT::NoData) {
	  RTT::Logger::log(RTT::Logger::Error) << "No tool data " << RTT::endlog();
	 // return false;
	}
	
	//tf::PoseMsgToKDL(tool_frame_msg, tool_frame);

	return true;
}

void VelocityProjector::stopHook() {

}

void VelocityProjector::updateHook() {

  Matrix77d M, Mi;
  Matrix76d Ji;
  Vector7d qd_dot, q0_dot;
  Vector6d xc_dot;
  
  port_command_period.read(dt);
  port_jacobian.read(jacobian);
  
  xc_dot = Vector6d::Zero();
  
  if(port_cartesian_velocity_command.read(cart_vel_cmd) == RTT::NewData){
    xc_dot(0) = cart_vel_cmd.linear.x;
    xc_dot(1) = cart_vel_cmd.linear.y;
    xc_dot(2) = cart_vel_cmd.linear.z;
    xc_dot(3) = cart_vel_cmd.angular.x;
    xc_dot(4) = cart_vel_cmd.angular.y; 
    xc_dot(5) = cart_vel_cmd.angular.z; 
  }
  
  q0_dot = Vector7d::Zero();
  
  if(port_nullspace_velocity_command.read(null_vel_cmd) == RTT::NewData){
      for(size_t i = 0; i < 7; i++) {
        q0_dot(i) = null_vel_cmd[i];
      }
  }

  port_mass_matrix.read(M);
  
  port_jnt_pos_des.read(jnt_pos_des);

  for(size_t i = 0; i < 7; i++)
    q0_dot(i) = -(jnt_pos_des[i]/17.0);

  Mi = M.inverse();
  Ji = Mi * jacobian.data.transpose() * (jacobian.data * Mi * jacobian.data.transpose()).inverse();
 
  qd_dot = Ji * xc_dot + (Matrix77d::Identity() - Ji * jacobian.data) * q0_dot;
  
  for(size_t i = 0; i < 7; i++) { 
    jnt_pos_cmd[i] = jnt_pos_des[i] + qd_dot(i) * dt;
  }
  
  port_joint_position_command.write(jnt_pos_cmd);
}

}
ORO_CREATE_COMPONENT(lwr::VelocityProjector)
