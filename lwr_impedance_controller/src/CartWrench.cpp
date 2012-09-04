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
  this->addPort("MassMatrix", port_mass_matrix);
  
  // commands outputs
  this->addPort("JointEffortCommand", port_jnt_trq_cmd);
  this->addPort("FriJointImpedance", port_joint_impedance_command);
  this->addPort("JointPositionCommand", port_joint_position_command);
  this->addPort("NullspaceEffortCommand", port_nullspace_torque_command);

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
  jnt_pos.resize(7);
  null_trq_cmd.resize(7);

  //set detault stiffness and damping
  cartesian_impedance_command.stiffness.force.x = 1000.0;
  cartesian_impedance_command.stiffness.force.y = 1000.0;
  cartesian_impedance_command.stiffness.force.z = 1000.0;

  cartesian_impedance_command.stiffness.torque.x = 100.0;
  cartesian_impedance_command.stiffness.torque.y = 100.0;
  cartesian_impedance_command.stiffness.torque.z = 100.0;

  cartesian_impedance_command.damping.force.x = 0.7;
  cartesian_impedance_command.damping.force.y = 0.7;
  cartesian_impedance_command.damping.force.z = 0.7;

  cartesian_impedance_command.damping.torque.x = 0.7;
  cartesian_impedance_command.damping.torque.y = 0.7;
  cartesian_impedance_command.damping.torque.z = 0.7;

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
    tool_frame = KDL::Frame::Identity();
  } else {
    tf::PoseMsgToKDL(tool_frame_msg, tool_frame);
  }
  tf::PoseMsgToKDL(cart_pos_msr, cart_pos_ref);
//  cart_pos_ref = cart_pos_ref * tool_frame;
  cart_pos_old = cart_pos_ref;

  for(size_t i = 0; i < 7; i++) {
    imp.stiffness[i] = 0.0;
    imp.damping[i] = 0.0;
    null_trq_cmd[i] = 0;
  }
  
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

double jointLimitTrq(double hl, double ll, double ls, double r_max, double q) {

  if(q > (hl - ls)) {
    return -1 * ((q - hl + ls)/ls) * ((q - hl + ls)/ls) * r_max;
  } else if(q < (ll + ls)) {
    return ((ll + ls - q)/ls) * ((ll + ls - q)/ls) * r_max;
  } else {
    return 0.0;
  }
  
}

void CartWrench::updateHook() {

  Matrix77d Kj, Dj;
  Vector6d Kc, Dxi, v, K0;
  Matrix77d M, Mi, N, N3;
  Matrix76d Ji, jT;
  Vector7d r0;
  Matrix66d A ,A1, Kc1, Dc, Q;

  port_joint_impedance_command.write(imp);
  port_command_period.read(dt);
  port_jacobian.read(jacobian);
  port_cartesian_impedance_command.read(cartesian_impedance_command);
  port_cartesian_wrench_command.read(cartesian_wrench_command);
  port_jnt_pos_msr.read(jnt_pos);
  port_mass_matrix.read(M);
  
  if(port_cartesian_position_command.read(cart_pos_cmd) == RTT::NewData){
    tf::PoseMsgToKDL(cart_pos_cmd, cart_pos_ref);
  }
  
  if(port_nullspace_torque_command.read(null_trq_cmd) == RTT::NewData) {
    for(size_t i = 0; i<7; i++)
      r0(i) = null_trq_cmd[i];
  }
  
  port_cart_pos_msr.read(cart_pos_msr);

//  jacobian.changeRefFrame(tool_frame.Inverse());

  KDL::Frame pos_msr, spring;

  tf::PoseMsgToKDL(cart_pos_msr, pos_msr);

//  pos_msr = pos_msr * tool_frame;

  Kc(0) = cartesian_impedance_command.stiffness.force.x;
  Kc(1) = cartesian_impedance_command.stiffness.force.y;
  Kc(2) = cartesian_impedance_command.stiffness.force.z;
  
  Kc(3) = cartesian_impedance_command.stiffness.torque.x;
  Kc(4) = cartesian_impedance_command.stiffness.torque.y;
  Kc(5) = cartesian_impedance_command.stiffness.torque.z;
  
  Dxi(0) = cartesian_impedance_command.damping.force.x;
  Dxi(1) = cartesian_impedance_command.damping.force.y;
  Dxi(2) = cartesian_impedance_command.damping.force.z;
  
  Dxi(3) = cartesian_impedance_command.damping.torque.x;
  Dxi(4) = cartesian_impedance_command.damping.torque.y;
  Dxi(5) = cartesian_impedance_command.damping.torque.z;
  
  jT = jacobian.data.transpose();
  
  Mi = M.inverse();
//  Ji = Mi * jacobian.data.transpose() * (jacobian.data * Mi * jacobian.data.transpose()).inverse();
//  N = (Matrix77d::Identity() - Ji * jacobian.data);
  
  A = (jacobian.data * Mi * jT).inverse();
  N3 = (Matrix77d::Identity() - jT * A * jacobian.data * Mi);
  
  es.compute(Kc.asDiagonal(), A);
  K0 = es.eigenvalues();
  Q = es.eigenvectors().inverse();

  Dc = Q.transpose() * Dxi.asDiagonal() * K0.cwiseSqrt().asDiagonal() * Q;
  
  // calculate stiffness component

  spring = pos_msr.Inverse() * cart_pos_ref;

  KDL::Twist tw = KDL::diff(KDL::Frame::Identity(), spring);

  Eigen::VectorXd trq(7);
  Eigen::VectorXd wrench2(6);

  KDL::Wrench wrench;

  wrench2(0) = tw.vel.x() * cartesian_impedance_command.stiffness.force.x;
  wrench2(1) = tw.vel.y() * cartesian_impedance_command.stiffness.force.y;
  wrench2(2) = tw.vel.z() * cartesian_impedance_command.stiffness.force.z;
  wrench2(3) = tw.rot.x() * cartesian_impedance_command.stiffness.torque.x;
  wrench2(4) = tw.rot.y() * cartesian_impedance_command.stiffness.torque.y;
  wrench2(5) = tw.rot.z() * cartesian_impedance_command.stiffness.torque.z;

  // calculate damping component
  KDL::Frame cart_pos_diff = cart_pos_old.Inverse() * pos_msr;
  cart_pos_old = pos_msr;

  KDL::Twist vel = KDL::diff(KDL::Frame::Identity(), cart_pos_diff);

  v(0) = ((vel.vel.x() / dt));
  v(1) = ((vel.vel.y() / dt));
  v(2) = ((vel.vel.z() / dt));
  v(3) = ((vel.rot.x() / dt));
  v(4) = ((vel.rot.y() / dt));
  v(5) = ((vel.rot.z() / dt));

  wrench2(0) -= Dc.diagonal()(0) * v(0);
  wrench2(1) -= Dc.diagonal()(1) * v(1);
  wrench2(2) -= Dc.diagonal()(2) * v(2);
  wrench2(3) -= Dc.diagonal()(3) * v(3);
  wrench2(4) -= Dc.diagonal()(4) * v(4);
  wrench2(5) -= Dc.diagonal()(5) * v(5);
  
  //add additional force/torque
  wrench2(0) += cartesian_wrench_command.force.x;
  wrench2(1) += cartesian_wrench_command.force.y;
  wrench2(2) += cartesian_wrench_command.force.z;
  wrench2(3) += cartesian_wrench_command.torque.x;
  wrench2(4) += cartesian_wrench_command.torque.y;
  wrench2(5) += cartesian_wrench_command.torque.z;
  
  r0(0) = jointLimitTrq(2.96, -2.96, 0.26, 10.0, jnt_pos[0]);
  r0(1) = jointLimitTrq(2.09, -2.09, 0.26, 10.0, jnt_pos[1]);
  r0(2) = jointLimitTrq(2.96, -2.96, 0.26, 10.0, jnt_pos[2]);
  r0(3) = jointLimitTrq(2.09, -2.09, 0.26, 10.0, jnt_pos[3]);
  r0(4) = jointLimitTrq(2.96, -2.96, 0.26, 10.0, jnt_pos[4]);
  r0(5) = jointLimitTrq(2.09, -2.09, 0.26, 10.0, jnt_pos[5]);
  r0(6) = jointLimitTrq(2.96, -2.96, 0.26, 10.0, jnt_pos[6]);

  trq = jT * wrench2 + N3 * r0;

  for(size_t i = 0; i < 7; i++)
    jnt_trq_cmd[i] = trq(i);

  // transform stiffness and damping to joint space
  Kj = jT * Kc.asDiagonal() * jacobian.data;

//  Dj = jacobian.data.transpose() * Dc * jacobian.data;

  for(size_t i = 0; i < 7; i++) {
    imp.stiffness[i] = Kj(i, i);
    imp.damping[i] = 0.0; //Dj(i, i);
  }

  port_joint_position_command.write(jnt_pos);
  port_joint_impedance_command.write(imp);
  port_jnt_trq_cmd.write(jnt_trq_cmd);
  
  geometry_msgs::Pose desired_pos;
  
  tf::PoseKDLToMsg(cart_pos_ref, desired_pos);
  port_desired_cartesian_position.write(desired_pos);

}

}
ORO_CREATE_COMPONENT(lwr::CartWrench)
