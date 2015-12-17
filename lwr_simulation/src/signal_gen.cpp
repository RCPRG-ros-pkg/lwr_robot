#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include "Eigen/Dense"

#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>

#include <kdl_parser/kdl_parser.hpp>

typedef Eigen::Matrix<double, 7, 7> Matrix77d;


class LWRControllerSignal : public RTT::TaskContext
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::VectorXd pos_cmd_out_;
    RTT::OutputPort<Eigen::VectorXd > port_pos_cmd_out_;
    double angle_;

    LWRControllerSignal(const std::string& name) : 
        TaskContext(name),
        angle_(0)
    {
        this->ports()->addPort("JointPositionCmdOut", port_pos_cmd_out_).doc("");
        pos_cmd_out_.resize(7);
        pos_cmd_out_.setZero();
        port_pos_cmd_out_.setDataSample(pos_cmd_out_);
    }

    virtual ~LWRControllerSignal() {
    }

    virtual bool startHook() {
        return true;
    }

    virtual void updateHook() {
        pos_cmd_out_(1) = std::sin(angle_);
        pos_cmd_out_(3) = std::sin(angle_);
        port_pos_cmd_out_.write(pos_cmd_out_);
        angle_ += 0.001;
    }

    virtual bool configureHook() {
        return true;
    }
};

ORO_LIST_COMPONENT_TYPE(LWRControllerSignal)
ORO_CREATE_COMPONENT_LIBRARY();

