
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

// Start of user code includes
#include <vector>
#include <Eigen/Dense>
#include <std_msgs/Float32.h>

typedef Eigen::Matrix<double, 6, 7> Matrix67d;
typedef Eigen::Matrix<double, 7, 7> Matrix77d;
typedef Eigen::Matrix<double, 6, 6> Matrix66d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
// End of user code

class SingularityAvoidance : public RTT::TaskContext {
public:
  SingularityAvoidance(const std::string & name) : TaskContext(name) {
    JointPosition_trig = false;

    prop_threshold = 0.04;
    prop_gain = 10.0;
	
    this->addProperty("threshold", prop_threshold);
    this->addProperty("gain", prop_gain);

    this->ports()->addEventPort("JointPosition", port_JointPosition, boost::bind(&SingularityAvoidance::JointPosition_onData, this, _1)).doc("");

    this->ports()->addPort("JointTorqueCommand", port_JointTorqueCommand).doc("");
    this->ports()->addPort("Manipulability", port_Manipulability).doc("");
  }

  ~SingularityAvoidance(){
  }

  bool configureHook() {
    // Start of user code configureHook
	jointTorqueCommand.resize(7);
	jointPosition.resize(7);
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
	if(JointPosition_trig &&  true) {
      doAvoidance();
      JointPosition_trig = false;
    }
  }

private:

  void doAvoidance() {
    // Start of user code Avoidance

	static int x = 100;
	std_msgs::Float32 manip;
	Eigen::Map<Vector7d> phi(&jointPosition[0]);
	Eigen::Map<Vector7d> tau(&jointTorqueCommand[0]);

	port_JointPosition.read(jointPosition);

	tau = manipulablityG(phi, prop_threshold, 10.0);

	if(--x < 0) {
		manip.data = manipulability(phi);
		port_Manipulability.write(manip);
//		std::cout << "manipulability : " << manipulability(phi) << std::endl;
//		std::cout << "torque : " << tau.transpose() << std::endl << std::endl;
		x = 20;
	}

	port_JointTorqueCommand.write(jointTorqueCommand);

	// End of user code
  }

  void JointPosition_onData(RTT::base::PortInterface* port) {
    JointPosition_trig = true;
  }

  RTT::InputPort<std::vector<double> > port_JointPosition;

  RTT::OutputPort<std::vector<double> > port_JointTorqueCommand;
  RTT::OutputPort<std_msgs::Float32 > port_Manipulability;

  double prop_threshold;
  double prop_gain;

  bool JointPosition_trig;

  // Start of user code userData
  std::vector<double> jointTorqueCommand;
  std::vector<double> jointPosition;

  Matrix67d jacobian(Vector7d phi) {
	  Matrix67d jac;
	  const double a_3 = 0.4;
	  const double a_5 = 0.39;
	  #include "jac.cpp"
	  return jac;
  }

  double manipulability(const Vector7d& phi) {
	  Matrix67d J = jacobian(phi);
	  Matrix66d Z = J*J.transpose();
	  return sqrt(Z.determinant());
  }

  double manipulablityV(const Vector7d& phi, const double m0, const double ks) {
	  double mkin = manipulability(phi);

	  if(mkin > m0) {
		  return 0;
	  } else {
		  return ks * (mkin - m0);
	  }
  }

  Vector7d manipulablityG(const Vector7d& phi, const double m0, const double ks) {
	  const double h = 0.0000001;
	  Vector7d tau;
	  Vector7d phi0;
	  Vector7d phi1;
	  for (unsigned int i = 0; i < 7; i++) {
		  phi0 = phi1 = phi;
		  phi0(i) -= h;
		  phi1(i) += h;
		  tau(i) = (manipulablityV(phi1, m0, ks) - manipulablityV(phi0, m0, ks))/(2.0*h);
	  }
	  return tau;
  }
  // End of user code

};

ORO_CREATE_COMPONENT(SingularityAvoidance)

