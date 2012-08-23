/*
 * ImpedanceProjector.h
 *
 *  Created on: 31-08-2011
 *      Author: konradb3
 */

#ifndef ImpedanceProjector_H_
#define ImpedanceProjector_H_

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

namespace lwr {

using namespace RTT;

class ImpedanceProjector : public RTT::TaskContext {
public:
	ImpedanceProjector(const std::string& name);
	virtual ~ImpedanceProjector();

	virtual bool configureHook();
	virtual bool startHook();

	virtual void updateHook();
	virtual void stopHook();
	virtual void cleanupHook();
private:

	OutputPort<std::vector<double> >  port_jnt_trq_cmd;
	OutputPort<lwr_fri::FriJointImpedance> port_joint_impedance_command;
	InputPort<std::vector<double> > port_joint_position_command;

	InputPort<KDL::Jacobian>  port_jacobian;
	InputPort<geometry_msgs::Pose> port_cart_pos_msr;
  InputPort<geometry_msgs::Pose> port_tool_frame;
	InputPort<lwr_impedance_controller::CartesianImpedance> port_cartesian_impedance_command;
  InputPort<double> port_command_period;
  InputPort<std::vector<double> > port_jnt_pos_msr;
  
  double dt;
	std::vector<double> jnt_trq_cmd;
	std::vector<double> jnt_pos, jnt_pos_cmd;
	lwr_fri::FriJointImpedance imp;
	KDL::Jacobian jacobian;
	KDL::Frame tool_frame;
	lwr_impedance_controller::CartesianImpedance cartesian_impedance_command;
	
};

}

#endif /* ImpedanceProjector_H_ */
