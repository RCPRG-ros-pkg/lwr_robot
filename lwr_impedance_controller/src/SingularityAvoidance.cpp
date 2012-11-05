
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

// Start of user code includes
#include <vector>
// End of user code

class SingularityAvoidance : public RTT::TaskContext {
public:
  SingularityAvoidance(const std::string & name) : TaskContext(name) {
    JointPosition_trig = false;

    threshold_prop = 0.4;
    gain_prop = 10.0;
  }

  ~SingularityAvoidance(){
  }

  bool configureHook() {
  	this->ports()->addEventPort("JointPosition", port_JointPosition, boost::bind(&SingularityAvoidance::JointPosition_onData, this, _1)).doc("");

	this->ports()->addPort("JointTorqueCommand", port_JointTorqueCommand).doc("");

	this->addProperty("threshold", threshold_prop);
	this->addProperty("gain", gain_prop);

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
	// TODO Put implementation of handler here !!!
	// End of user code
  }

  void JointPosition_onData(RTT::base::PortInterface* port) {
    JointPosition_trig = true;
  }

  RTT::InputPort<std::vector<double> > port_JointPosition;

  RTT::OutputPort<std::vector<double> > port_JointTorqueCommand;

  double threshold_prop;
  double gain_prop;

  bool JointPosition_trig;

  // Start of user code userData
  std::vector<double> jointTorqueCommand;
  std::vector<double> jointPosition;
  // End of user code

};

ORO_CREATE_COMPONENT(SingularityAvoidance)

