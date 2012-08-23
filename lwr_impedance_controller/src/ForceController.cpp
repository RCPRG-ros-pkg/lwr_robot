/*
 * SpringCoupler.cpp
 *
 *  Created on: 31-08-2011
 *      Author: konradb3
 */

#include <rtt/Component.hpp>

#include "ForceController.h"
#include <tf_conversions/tf_kdl.h>
#include <Eigen/Dense>

namespace lwr {

ForceController::ForceController(const std::string& name) : TaskContext(name, PreOperational) {
  // command inputs

  this->addPort("CartesianVelocityCommand", port_cartesian_velocity_command);
  this->addEventPort("CartesianWrench", port_cartesian_wrench);

}

ForceController::~ForceController() {

}

bool ForceController::configureHook() {
	return true;
}

void ForceController::cleanupHook() {

}

bool ForceController::startHook() {
	return true;
}

void ForceController::stopHook() {

}

void ForceController::updateHook() {

  port_cartesian_wrench.read(cartesian_wrench);

  cartesian_twist.linear.x = 0.0; // -(1.0/100.0) * cartesian_wrench.force.x;
  cartesian_twist.linear.y = 0.0; // -(1.0/100.0) * cartesian_wrench.force.y;
  cartesian_twist.linear.z = -(1.0/60.0) * cartesian_wrench.force.z;
  
  cartesian_twist.angular.x = 0.0; //-(1.0/5.0) * cartesian_wrench.torque.x;
  cartesian_twist.angular.y = 0.0; -(1.0/5.0) * cartesian_wrench.torque.y;
  cartesian_twist.angular.z = 0.0; -(1.0/5.0) * cartesian_wrench.torque.z;

  port_cartesian_velocity_command.write(cartesian_twist);
}

}
ORO_CREATE_COMPONENT(lwr::ForceController)
