/*
 * CartImpTrajectory.cpp
 *
 *  Created on: 31-08-2011
 *      Author: konradb3
 */

#include <rtt/Component.hpp>

#include "CartImpTrajectory.h"
#include <tf_conversions/tf_kdl.h>

namespace lwr {

CartImpTrajectory::CartImpTrajectory(const std::string& name) : TaskContext(name, PreOperational) {
  this->addPort("TrajectoryCommand", port_cart_imp_trajectory_cmd);
  this->addPort("CartesianPosition", port_cart_pos_msr);
  this->addPort("DesiredCartesianPosition", port_desired_cartesian_position);
  
  this->addPort("CartesianImpedanceCommand", port_cartesian_impedance_cmd);
  this->addPort("CartesianWrenchCommand", port_cart_wrench_cmd);
  this->addPort("CartesianPositionCommand", port_cart_position_cmd);
  
  this->addPort("ToolFrame", port_tool_frame);

}

CartImpTrajectory::~CartImpTrajectory() {

}

bool CartImpTrajectory::configureHook() {
	return true;
}

void CartImpTrajectory::cleanupHook() {

}

bool CartImpTrajectory::startHook() {

  setpoint_.impedance.stiffness.force.x = 500;
  setpoint_.impedance.stiffness.force.y = 500;
  setpoint_.impedance.stiffness.force.z = 500;

  setpoint_.impedance.stiffness.torque.x = 20;
  setpoint_.impedance.stiffness.torque.y = 20;
  setpoint_.impedance.stiffness.torque.z = 20;

  setpoint_.impedance.damping.force.x = 40.0;
	setpoint_.impedance.damping.force.y = 40.0;
	setpoint_.impedance.damping.force.z = 40.0;

	setpoint_.impedance.damping.torque.x = 0.80;
	setpoint_.impedance.damping.torque.y = 0.80;
	setpoint_.impedance.damping.torque.z = 0.80;

  setpoint_.wrench.force.x = 0.0;
  setpoint_.wrench.force.y = 0.0;
  setpoint_.wrench.force.z = 0.0;

  setpoint_.wrench.torque.x = 0.0;
  setpoint_.wrench.torque.y = 0.0;
  setpoint_.wrench.torque.z = 0.0;

  setpoint_.velocity.linear.x = 0.0;
  setpoint_.velocity.linear.y = 0.0;
  setpoint_.velocity.linear.z = 0.0;
  
  setpoint_.velocity.angular.x = 0.0;
  setpoint_.velocity.angular.y = 0.0;
  setpoint_.velocity.angular.z = 0.0;

  tool_frame.M = KDL::Rotation::RPY(0.0, -1.57079633, 0.0);
  tool_frame.p = KDL::Vector(-0.115, 0.0, -0.055);
  
  if(port_cart_pos_msr.read(setpoint_.pose) == RTT::NoData) {
	  return false;
	}
  
  geometry_msgs::Pose tool_frame_msg;
  
  tf::PoseKDLToMsg(tool_frame, tool_frame_msg);
  
  port_tool_frame.write(tool_frame_msg);
  
  tf::PoseMsgToKDL(setpoint_.pose, cart_pos_cmd);
  
  cart_pos_cmd = cart_pos_cmd * tool_frame;
  
  tf::PoseKDLToMsg(cart_pos_cmd, setpoint_.pose);
  
  last_point_ = setpoint_;

  valid_trajectory_ = false;
  dt_ = 0.001;

	return true;
}

void CartImpTrajectory::stopHook() {

}

void CartImpTrajectory::updateHook() {

  if(port_cart_imp_trajectory_cmd.read(trajectory_tmp_) == RTT::NewData) {
    if(trajectory_tmp_.trajectory.size() == 0) {
      valid_trajectory_ = false;
    } else {
      trajectory_ = trajectory_tmp_;
      trajectory_index_ = 0;
      port_desired_cartesian_position.read(setpoint_.pose);
      last_point_ = setpoint_;
      last_point_.time_from_start = ros::Duration(0);
      time_ = 0;
      
      valid_trajectory_ = true;
    }
  }

  if(valid_trajectory_ == true) {
    ///do interpolation
    if(trajectory_.trajectory[trajectory_index_].time_from_start.toSec() <= ((double)time_ * dt_)) {
      if(trajectory_.trajectory.size() <= (trajectory_index_ + 1)) {
       // setpoint_ = trajectory_.trajectory[trajectory_index_];
        valid_trajectory_ = false;
      } else {
        last_point_ = trajectory_.trajectory[trajectory_index_];
        ++trajectory_index_;
      }
    }

    if(valid_trajectory_ == true) {
      setpoint_ = sampleInterpolation();
    }
    
    ++time_;
  }

  port_cart_position_cmd.write(setpoint_.pose);
  port_cartesian_impedance_cmd.write(setpoint_.impedance);
  port_cart_wrench_cmd.write(setpoint_.wrench);
}

double CartImpTrajectory::linearlyInterpolate(double time, 
						    double startTime, 
						    double endTime, 
						    double startValue, 
						    double endValue) {
  return startValue + 
    (time - startTime)*
    (endValue - startValue)/(endTime - startTime);
}

lwr_impedance_controller::CartImpTrajectoryPoint 
CartImpTrajectory::sampleInterpolation() {
    lwr_impedance_controller::CartImpTrajectoryPoint next_point;

    double timeFromStart = (double)time_ * dt_;
    double segStartTime = last_point_.time_from_start.toSec();
    double segEndTime = trajectory_.trajectory[trajectory_index_].time_from_start.toSec();
    
 //   RTT::Logger::log(RTT::Logger::Error) << "trj " << trajectory_index_ << " : " << trajectory_.trajectory[trajectory_index_].pose.position.x << " y " << trajectory_.trajectory[trajectory_index_].pose.position.x << " z " << trajectory_.trajectory[trajectory_index_].pose.position.x << RTT::endlog();
    
    // interpolate position
    // x
    next_point.pose.position.x = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       last_point_.pose.position.x, trajectory_.trajectory[trajectory_index_].pose.position.x);
    // y
    next_point.pose.position.y = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       last_point_.pose.position.y, trajectory_.trajectory[trajectory_index_].pose.position.y); 
    // z
    next_point.pose.position.z = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       last_point_.pose.position.z, trajectory_.trajectory[trajectory_index_].pose.position.z); 
       
    // interpolate orientation
    // x
    next_point.pose.orientation.x = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       last_point_.pose.orientation.x, trajectory_.trajectory[trajectory_index_].pose.orientation.x); 
    // y
    next_point.pose.orientation.y = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       last_point_.pose.orientation.y, trajectory_.trajectory[trajectory_index_].pose.orientation.y);
    // z
    next_point.pose.orientation.z = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       last_point_.pose.orientation.z, trajectory_.trajectory[trajectory_index_].pose.orientation.z); 
    // w
    next_point.pose.orientation.w = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       last_point_.pose.orientation.w, trajectory_.trajectory[trajectory_index_].pose.orientation.w); 

    //interpolate stiffness
    // x
    next_point.impedance.stiffness.force.x = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       last_point_.impedance.stiffness.force.x, trajectory_.trajectory[trajectory_index_].impedance.stiffness.force.x); 
       
    next_point.impedance.stiffness.force.y = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       last_point_.impedance.stiffness.force.y, trajectory_.trajectory[trajectory_index_].impedance.stiffness.force.y); 
       
    next_point.impedance.stiffness.force.y = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       last_point_.impedance.stiffness.force.z, trajectory_.trajectory[trajectory_index_].impedance.stiffness.force.z);
       
    next_point.impedance.stiffness.torque.x = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       last_point_.impedance.stiffness.torque.x, trajectory_.trajectory[trajectory_index_].impedance.stiffness.torque.x); 
       
    next_point.impedance.stiffness.torque.y = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       last_point_.impedance.stiffness.torque.y, trajectory_.trajectory[trajectory_index_].impedance.stiffness.torque.y);
       
    next_point.impedance.stiffness.torque.z = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       last_point_.impedance.stiffness.torque.z, trajectory_.trajectory[trajectory_index_].impedance.stiffness.torque.z);  

    next_point.wrench = trajectory_.trajectory[trajectory_index_].wrench;

//  RTT::Logger::log(RTT::Logger::Error) << "msr : " << next_point.pose.position.x << " y " << next_point.pose.position.y << " z " << next_point.pose.position.z << RTT::endlog();
  
  return next_point;
}

}
ORO_CREATE_COMPONENT(lwr::CartImpTrajectory)
