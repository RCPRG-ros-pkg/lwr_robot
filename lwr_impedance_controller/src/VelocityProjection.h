/*
 * VelocityProjector.h
 *
 *  Created on: 31-08-2011
 *      Author: konradb3
 */

#ifndef VelocityProjector_H_
#define VelocityProjector_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <sensor_msgs/typekit/Types.h>
#include <geometry_msgs/typekit/Types.h>

typedef Eigen::Matrix<double, 7, 6> Matrix76d;
typedef Eigen::Matrix<double, 7, 7> Matrix77d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

namespace lwr {

using namespace RTT;

class VelocityProjector : public RTT::TaskContext {
public:
	VelocityProjector(const std::string& name);
	virtual ~VelocityProjector();

	virtual bool configureHook();
	virtual bool startHook();

	virtual void updateHook();
	virtual void stopHook();
	virtual void cleanupHook();
private:
	OutputPort<std::vector<double> > port_joint_position_command;
	
	OutputPort<geometry_msgs::Pose> port_desired_cartesian_position;
	
	InputPort<KDL::Jacobian>  port_jacobian;
  InputPort<geometry_msgs::Twist> port_cartesian_velocity_command;
  InputPort<geometry_msgs::Pose> port_tool_frame;
  InputPort<double> port_command_period;
  InputPort<std::vector<double> > port_jnt_pos_des;
  InputPort<std::vector<double> > port_nullspace_velocity_command;
  InputPort<Matrix77d> port_mass_matrix;
  
  double dt;
	std::vector<double> jnt_pos_des;
	std::vector<double> jnt_pos_cmd;
	std::vector<double> null_vel_cmd;
	geometry_msgs::Twist cart_vel_cmd;
	KDL::Twist cart_vel;
	KDL::Jacobian jacobian;
	KDL::Frame tool_frame;
	
};

}

#endif /* VelocityProjector_H_ */
