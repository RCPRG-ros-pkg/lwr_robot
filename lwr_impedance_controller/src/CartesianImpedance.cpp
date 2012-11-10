
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

// Start of user code includes
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <tf_conversions/tf_kdl.h>

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
typedef Eigen::Matrix<double, 4, 1> Vector4d;
// End of user code

class CartesianImpedance : public RTT::TaskContext {
public:
  CartesianImpedance(const std::string & name) : TaskContext(name) {
    JointPosition_trig = false;
    CartesianPosition_trig = false;
    Jacobian_trig = false;
    MassMatrix_trig = false;
    CartesianPositionCommand_trig = false;


  	this->ports()->addEventPort("JointPosition", port_JointPosition, boost::bind(&CartesianImpedance::JointPosition_onData, this, _1)).doc("");
  	this->ports()->addEventPort("CartesianPosition", port_CartesianPosition, boost::bind(&CartesianImpedance::CartesianPosition_onData, this, _1)).doc("");
  	this->ports()->addEventPort("Jacobian", port_Jacobian, boost::bind(&CartesianImpedance::Jacobian_onData, this, _1)).doc("");
  	this->ports()->addEventPort("MassMatrix", port_MassMatrix, boost::bind(&CartesianImpedance::MassMatrix_onData, this, _1)).doc("");
  	this->ports()->addEventPort("CartesianPositionCommand", port_CartesianPositionCommand, boost::bind(&CartesianImpedance::CartesianPositionCommand_onData, this, _1)).doc("");
	this->ports()->addPort("CartesianWrenchCommand", port_CartesianWrenchCommand).doc("");
	this->ports()->addPort("CartesianImpedanceCommand", port_CartesianImpedanceCommand).doc("");
	this->ports()->addPort("Tool", port_Tool).doc("");
	this->ports()->addPort("CartesianVelocity", port_CartesianVelocity).doc("");

	this->ports()->addPort("JointTorqueCommand", port_JointTorqueCommand).doc("");
	this->ports()->addPort("JointImpedanceCommand", port_JointImpedanceCommand).doc("");
	this->ports()->addPort("JointPositionCommand", port_JointPositionCommand).doc("");
  }

  ~CartesianImpedance(){
  }

  bool configureHook() {

    // Start of user code configureHook
		tau_.resize(7);
		jnt_pos_.resize(7);
		j_.resize(7);
		tau_.resize(7);
		jnt_trq_cmd_.resize(7);
		// End of user code
    return true;
  }

  bool startHook() {
    // Start of user code startHook
		T_T = KDL::Frame::Identity();
		// End of user code
    return true;
  }

  void stopHook() {
    // Start of user code stopHook
		// End of user code
  }

  void updateHook() {
	if(Jacobian_trig && MassMatrix_trig && CartesianPosition_trig && JointPosition_trig && CartesianPositionCommand_trig &&  true) {
      doImpedanceControl();
      Jacobian_trig = false;
      MassMatrix_trig = false;
      CartesianPosition_trig = false;
      JointPosition_trig = false;
      CartesianPositionCommand_trig = false;
    }
  }

private:

  void doImpedanceControl() {
    // Start of user code ImpedanceControl
		KDL::Frame T, T_D, T_C, T_S;
		lwr_impedance_controller::CartesianImpedance cart_imp;
		lwr_fri::FriJointImpedance jnt_imp;
		geometry_msgs::Pose cart_pos, cart_pos_cmd, tool_pos;
		geometry_msgs::Twist cart_vel;
		Matrix77d Kj, Dj;
		Vector6d Kc, Dxi,K0;
		Matrix77d M, Mi, N;
		Matrix76d Ji, jT;
		Vector7d tau;
		Vector6d F;
		Vector4d e;
		Matrix66d A, A1, Kc1, Dc, Q;

		Eigen::GeneralizedSelfAdjointEigenSolver<Matrix66d> es;

		port_JointPosition.read(jnt_pos_);
		port_MassMatrix.read(M);
		port_Jacobian.read(j_);
		port_CartesianPosition.read(cart_pos);
		port_CartesianPositionCommand.read(cart_pos_cmd);
		port_CartesianImpedanceCommand.read(cart_imp);
		port_CartesianVelocity.read(cart_vel);

		tf::PoseMsgToKDL(cart_pos, T);
		tf::PoseMsgToKDL(cart_pos_cmd, T_D);

		if (port_Tool.read(tool_pos) == RTT::NewData) {
			tf::PoseMsgToKDL(tool_pos, T_T);
		}

		// compute cartesian position of tool
		T_C = T * T_T;

		// copy impedance command data to internal structure
		Kc(0) = cart_imp.stiffness.force.x;
		Kc(1) = cart_imp.stiffness.force.y;
		Kc(2) = cart_imp.stiffness.force.z;

		Kc(3) = cart_imp.stiffness.torque.x;
		Kc(4) = cart_imp.stiffness.torque.y;
		Kc(5) = cart_imp.stiffness.torque.z;

		Dxi(0) = cart_imp.damping.force.x;
		Dxi(1) = cart_imp.damping.force.y;
		Dxi(2) = cart_imp.damping.force.z;

		Dxi(3) = cart_imp.damping.torque.x;
		Dxi(4) = cart_imp.damping.torque.y;
		Dxi(5) = cart_imp.damping.torque.z;

		// calculate transpose of jacobian
		jT = j_.data.transpose();

		// calculate inverse of manipulator mass matrix
		Mi = M.inverse();
		// calculate jacobian pseudo inverse
		Ji = Mi * jT * (j_.data * Mi * jT).inverse();
		// calculate null-space projection matrix
		N = (Matrix77d::Identity() - Ji * j_.data);
		// calculate cartesian mass matrix
		A = (j_.data * Mi * jT).inverse();

		// compute dumping matrix

		es.compute(Kc.asDiagonal(), A);
		K0 = es.eigenvalues();
		Q = es.eigenvectors().inverse();

		Dc = Q.transpose() * Dxi.asDiagonal() * K0.cwiseSqrt().asDiagonal() * Q;

		// compute length of spring
		T_S = T_C.Inverse() * T_D;

		T_S.M.GetQuaternion(e(0), e(1), e(2), e(3));

		// calculate spring force
		F(0) = Kc(0) * T_S.p[0];
		F(1) = Kc(1) * T_S.p[1];
		F(2) = Kc(2) * T_S.p[2];

		F(3) = Kc(3) * e(0);
		F(4) = Kc(4) * e(1);
		F(5) = Kc(5) * e(2);

		// compute dumping force
		F(0) -= Dc.diagonal()(0) * cart_vel.linear.x;
		F(1) -= Dc.diagonal()(1) * cart_vel.linear.y;
		F(2) -= Dc.diagonal()(2) * cart_vel.linear.z;

		F(3) -= Dc.diagonal()(3) * cart_vel.angular.x;
		F(4) -= Dc.diagonal()(4) * cart_vel.angular.y;
		F(5) -= Dc.diagonal()(5) * cart_vel.angular.z;

		// transform cartesian force to joint torques
		tau = jT * F;
		// project stiffness to joint space for local stiffness control
		Kj = jT * Kc.asDiagonal() * j_.data;

		for(unsigned int i = 0; i < 7; i++) {
			jnt_trq_cmd_[i] = tau(i);
		  jnt_imp.stiffness[i] = Kj(i, i);
		  jnt_imp.damping[i] = 0.0;
		}

		port_JointImpedanceCommand.write(jnt_imp);
		port_JointTorqueCommand.write(jnt_trq_cmd_);
		port_JointPositionCommand.write(jnt_pos_);
		// End of user code
  }

  void JointPosition_onData(RTT::base::PortInterface* port) {
    JointPosition_trig = true;
  }
  void CartesianPosition_onData(RTT::base::PortInterface* port) {
    CartesianPosition_trig = true;
  }
  void Jacobian_onData(RTT::base::PortInterface* port) {
    Jacobian_trig = true;
  }
  void MassMatrix_onData(RTT::base::PortInterface* port) {
    MassMatrix_trig = true;
  }
  void CartesianPositionCommand_onData(RTT::base::PortInterface* port) {
    CartesianPositionCommand_trig = true;
  }

  RTT::InputPort<std::vector<double> > port_JointPosition;
  RTT::InputPort<geometry_msgs::Pose > port_CartesianPosition;
  RTT::InputPort<KDL::Jacobian > port_Jacobian;
  RTT::InputPort<Matrix77d > port_MassMatrix;
  RTT::InputPort<geometry_msgs::Pose > port_CartesianPositionCommand;
  RTT::InputPort<geometry_msgs::Wrench > port_CartesianWrenchCommand;
  RTT::InputPort<lwr_impedance_controller::CartesianImpedance > port_CartesianImpedanceCommand;
  RTT::InputPort<geometry_msgs::Pose > port_Tool;
  RTT::InputPort<geometry_msgs::Twist > port_CartesianVelocity;

  RTT::OutputPort<std::vector<double> > port_JointTorqueCommand;
  RTT::OutputPort<lwr_fri::FriJointImpedance > port_JointImpedanceCommand;
  RTT::OutputPort<std::vector<double> > port_JointPositionCommand;


  bool JointPosition_trig;
  bool CartesianPosition_trig;
  bool Jacobian_trig;
  bool MassMatrix_trig;
  bool CartesianPositionCommand_trig;

  // Start of user code userData
	KDL::Jacobian j_;
	std::vector<double> jnt_pos_, tau_, jnt_trq_cmd_;
	KDL::Frame T_T;
	// End of user code

};

ORO_CREATE_COMPONENT(CartesianImpedance)

