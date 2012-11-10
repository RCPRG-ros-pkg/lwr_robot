
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

// Start of user code includes
#include <kdl/jacobian.hpp>
#include <Eigen/Dense>

typedef Eigen::Matrix<double, 7, 6> Matrix76d;
typedef Eigen::Matrix<double, 7, 7> Matrix77d;
typedef Eigen::Matrix<double, 6, 6> Matrix66d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
// End of user code

class NullSpaceProjection : public RTT::TaskContext {
public:
  NullSpaceProjection(const std::string & name) : TaskContext(name) {
    Jacobian_trig = false;
    MassMatrix_trig = false;
    NullSpaceTorque_trig = false;


  	this->ports()->addEventPort("Jacobian", port_Jacobian, boost::bind(&NullSpaceProjection::Jacobian_onData, this, _1)).doc("");
  	this->ports()->addEventPort("MassMatrix", port_MassMatrix, boost::bind(&NullSpaceProjection::MassMatrix_onData, this, _1)).doc("");
  	this->ports()->addEventPort("NullSpaceTorque", port_NullSpaceTorque, boost::bind(&NullSpaceProjection::NullSpaceTorque_onData, this, _1)).doc("");

	this->ports()->addPort("JointTorqueCommand", port_JointTorqueCommand).doc("");
  }

  ~NullSpaceProjection(){
  }

  bool configureHook() {

    // Start of user code configureHook
	torque_in.resize(7);
	torque_out.resize(7);
	// End of user code
    return true;
  }

  bool startHook() {
    // Start of user code startHook
    // TODO Put implementation of startHook here !!!
    // End of user code
    return true;
  }

  void stopHook() {
    // Start of user code stopHook
	// TODO Put implementation of stopHook here !!!
	// End of user code
  }

  void updateHook() {
	if(Jacobian_trig && MassMatrix_trig && NullSpaceTorque_trig &&  true) {
      doProjection();
      Jacobian_trig = false;
      MassMatrix_trig = false;
      NullSpaceTorque_trig = false;
    }
  }

private:

  void doProjection() {
    // Start of user code Projection
	Matrix77d M, Mi, N;
	Matrix76d Ji, jT;
	Vector7d r0;
	Eigen::Map<Vector7d> tau_0(&torque_in[0]);
	Eigen::Map<Vector7d> tau(&torque_out[0]);

	port_Jacobian.read(J);
	port_MassMatrix.read(M);
	port_NullSpaceTorque.read(torque_in);

	jT = J.data.transpose();
	Mi = M.inverse();
	Ji = Mi * jT * (J.data * Mi * jT).inverse();
	N = (Matrix77d::Identity() - Ji * J.data);

	tau = N * tau_0;

	port_JointTorqueCommand.write(torque_out);
	// End of user code
  }

  void Jacobian_onData(RTT::base::PortInterface* port) {
    Jacobian_trig = true;
  }
  void MassMatrix_onData(RTT::base::PortInterface* port) {
    MassMatrix_trig = true;
  }
  void NullSpaceTorque_onData(RTT::base::PortInterface* port) {
    NullSpaceTorque_trig = true;
  }

  RTT::InputPort<KDL::Jacobian > port_Jacobian;
  RTT::InputPort<Matrix77d > port_MassMatrix;
  RTT::InputPort<std::vector<double> > port_NullSpaceTorque;

  RTT::OutputPort<std::vector<double> > port_JointTorqueCommand;


  bool Jacobian_trig;
  bool MassMatrix_trig;
  bool NullSpaceTorque_trig;

  // Start of user code userData
  KDL::Jacobian J;
  std::vector<double> torque_in, torque_out;
  // End of user code

};

ORO_CREATE_COMPONENT(NullSpaceProjection)

