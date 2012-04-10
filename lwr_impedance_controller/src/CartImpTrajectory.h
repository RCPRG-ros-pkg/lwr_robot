/*
 * CartImpTrajectory.h
 *
 *  Created on: 31-08-2011
 *      Author: konradb3
 */

#ifndef CARTIMPTRAJECTORY_H_
#define CARTIMPTRAJECTORY_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <lwr_fri/typekit/Types.h>
#include <sensor_msgs/typekit/Types.h>
#include <geometry_msgs/typekit/Types.h>
#include <lwr_impedance_controller/CartImpTrajectory.h>
#include <lwr_impedance_controller/CartesianImpedance.h>

namespace lwr {

using namespace RTT;

class CartImpTrajectory : public RTT::TaskContext {
public:
	CartImpTrajectory(const std::string& name);
	virtual ~CartImpTrajectory();

	virtual bool configureHook();
	virtual bool startHook();

	virtual void updateHook();
	virtual void stopHook();
	virtual void cleanupHook();
private:

  double linearlyInterpolate(double time, 
						    double startTime, 
						    double endTime, 
						    double startValue, 
						    double endValue);
  lwr_impedance_controller::CartImpTrajectoryPoint sampleInterpolation();
  
	InputPort<lwr_impedance_controller::CartImpTrajectory> port_cart_imp_trajectory_cmd;
  InputPort<geometry_msgs::Pose> port_cart_pos_msr;
  InputPort<geometry_msgs::Pose> port_desired_cartesian_position;

  OutputPort<lwr_impedance_controller::CartesianImpedance>  port_cartesian_impedance_cmd;
	OutputPort<geometry_msgs::Wrench> port_cart_wrench_cmd;
  OutputPort<geometry_msgs::Pose> port_cart_position_cmd;
  OutputPort<geometry_msgs::Pose> port_tool_frame;
	
  lwr_impedance_controller::CartImpTrajectoryPoint setpoint_;
  lwr_impedance_controller::CartImpTrajectory trajectory_tmp_;
  lwr_impedance_controller::CartImpTrajectory trajectory_;

  geometry_msgs::Pose cart_pos_msr;
  KDL::Frame cart_pos_old, tool_frame, cart_pos_cmd;

  bool valid_trajectory_;
  unsigned int trajectory_index_;
  lwr_impedance_controller::CartImpTrajectoryPoint last_point_;
  unsigned int time_;
  double dt_;

};

}

#endif /* CARTIMPTRAJECTORY_H_ */
