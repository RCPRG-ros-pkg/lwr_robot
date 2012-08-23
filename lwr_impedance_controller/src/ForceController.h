/*
 * ForceController.h
 *
 *  Created on: 31-08-2011
 *      Author: konradb3
 */

#ifndef ForceController_H_
#define ForceController_H_

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

class ForceController : public RTT::TaskContext {
public:
	ForceController(const std::string& name);
	virtual ~ForceController();

	virtual bool configureHook();
	virtual bool startHook();

	virtual void updateHook();
	virtual void stopHook();
	virtual void cleanupHook();
private:

	InputPort<geometry_msgs::Wrench>  port_cartesian_wrench;
  OutputPort<geometry_msgs::Twist> port_cartesian_velocity_command;
  
  double dt;
	geometry_msgs::Wrench cartesian_wrench;
	geometry_msgs::Twist cartesian_twist;
	
};

}

#endif /* ForceController_H_ */
