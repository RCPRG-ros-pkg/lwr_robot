/*
 * CartWrench.h
 *
 *  Created on: 31-08-2011
 *      Author: konradb3
 */

#ifndef CARTWRENCH_H_
#define CARTWRENCH_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <lwr_fri/typekit/Types.h>
#include <sensor_msgs/typekit/Types.h>
#include <geometry_msgs/typekit/Types.h>
#include <lwr_impedance_controller/CartesianImpedance.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/MatrixFunctions>

typedef Eigen::Matrix<double, 7, 6> Matrix76d;
typedef Eigen::Matrix<double, 7, 7> Matrix77d;
typedef Eigen::Matrix<double, 6, 6> Matrix66d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;


namespace lwr {

using namespace RTT;

class CartWrench : public RTT::TaskContext {
public:
  CartWrench(const std::string& name);
  virtual ~CartWrench();

  virtual bool configureHook();
  virtual bool startHook();

  virtual void updateHook();
  virtual void stopHook();
  virtual void cleanupHook();
private:

  OutputPort<std::vector<double> >  port_jnt_trq_cmd;
  OutputPort<lwr_fri::FriJointImpedance> port_joint_impedance_command;
  OutputPort<std::vector<double> > port_joint_position_command;

  OutputPort<geometry_msgs::Pose> port_desired_cartesian_position;
  
  InputPort<KDL::Jacobian>  port_jacobian;
  InputPort<geometry_msgs::Pose> port_cart_pos_msr;
  InputPort<geometry_msgs::Pose> port_cartesian_position_command;
  InputPort<geometry_msgs::Pose> port_tool_frame;
  InputPort<lwr_impedance_controller::CartesianImpedance> port_cartesian_impedance_command;
  InputPort<geometry_msgs::Wrench> port_cartesian_wrench_command;
  InputPort<double> port_command_period;
  InputPort<std::vector<double> > port_jnt_pos_msr;
  InputPort<std::vector<double> > port_nullspace_torque_command;
  InputPort<Matrix77d> port_mass_matrix;
  
  double dt;
  std::vector<double> jnt_trq_cmd;
  std::vector<double> jnt_pos;
  std::vector<double> null_trq_cmd;
  lwr_fri::FriJointImpedance imp;
  KDL::Jacobian jacobian;
  KDL::Frame tool_frame;
  lwr_impedance_controller::CartesianImpedance cartesian_impedance_command;
  geometry_msgs::Wrench cartesian_wrench_command;
  
  geometry_msgs::Pose cart_pos_msr, cart_pos_cmd;
  KDL::Frame cart_pos_ref;
  KDL::Frame cart_pos_old;

  Eigen::GeneralizedSelfAdjointEigenSolver<Matrix66d> es;
  
};

}

#endif /* CARTWRENCH_H_ */
