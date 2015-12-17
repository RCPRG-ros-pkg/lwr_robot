
#ifndef LWR_CONTROLLER_HH
#define LWR_CONTROLLER_HH

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

#include <lwr_fri/CartesianImpedance.h>
#include <lwr_fri/FriJointImpedance.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>

#include <kuka_lwr_fri/friComm.h>

typedef Eigen::Matrix<double, 7, 7> Matrix77d;


class LWRController : public RTT::TaskContext
{
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// \brief Constructor
  public: LWRController(const std::string& name);

  /// \brief Destructor
  public: virtual ~LWRController();
  public: virtual bool startHook();
  public: virtual bool gazeboConfigureHook(gazebo::physics::ModelPtr model);
  public: virtual void updateHook();
  public: virtual bool configureHook();
  public: virtual void gazeboUpdateHook(gazebo::physics::ModelPtr model);

  private: void GetRobotChain();

  private:
    //! Synchronization
    RTT::os::MutexRecursive gazebo_mutex_;

    ros::NodeHandle* rosnode_;
    
    std::string robotPrefix_;
    std::vector<gazebo::physics::JointPtr>  joints_;
    std::string chain_start, chain_end;
    
    
    KDL::Chain chain_;
    KDL::ChainDynParam *dyn;
    KDL::ChainFkSolverPos_recursive *fk;
    KDL::ChainJntToJacSolver *jc;

    std::string base_frame_;

    Eigen::Matrix<double, 7, 1> joint_pos_;
    Eigen::Matrix<double, 7, 1> joint_pos_cmd_;
    Eigen::Matrix<double, 7, 1> joint_vel_;
    Eigen::Matrix<double, 7, 1> stiffness_;
    Eigen::Matrix<double, 7, 1> damping_;
    Eigen::Matrix<double, 7, 1> trq_cmd_;
    Eigen::Matrix<double, 7, 1> trq_;
    
    std::vector<double> jnt_pos_;
    std::vector<double> jnt_trq_;
    std::vector<double> jnt_vel_;

    Eigen::VectorXd jnt_pos_cmd_;
    Eigen::VectorXd jnt_trq_cmd_;

    KDL::Frame T_old;
    
    RTT::InputPort<lwr_fri::CartesianImpedance > port_CartesianImpedanceCommand;
    RTT::InputPort<geometry_msgs::Wrench > port_CartesianWrenchCommand;
    RTT::InputPort<geometry_msgs::Pose > port_CartesianPositionCommand;
    RTT::InputPort<lwr_fri::FriJointImpedance > port_JointImpedanceCommand;
    RTT::InputPort<Eigen::VectorXd > port_JointPositionCommand;
    RTT::InputPort<Eigen::VectorXd > port_JointTorqueCommand;

    RTT::OutputPort<geometry_msgs::Wrench > port_CartesianWrench;
    RTT::OutputPort<tFriRobotState > port_RobotState;
    RTT::OutputPort<tFriIntfState > port_FRIState;
    RTT::OutputPort<std::vector<double> > port_JointVelocity;
    RTT::OutputPort<geometry_msgs::Twist > port_CartesianVelocity;
    RTT::OutputPort<geometry_msgs::Pose > port_CartesianPosition;
    RTT::OutputPort<Matrix77d > port_MassMatrix;
    RTT::OutputPort<KDL::Jacobian > port_Jacobian;
    RTT::OutputPort<std::vector<double> > port_JointTorque;
    RTT::OutputPort<std::vector<double> > port_JointPosition;
};

#endif

