
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

// Start of user code includes
// TODO Put includes here.
// End of user code

class JointLimitsAvoidance : public RTT::TaskContext {
public:
  JointLimitsAvoidance(const std::string & name) : TaskContext(name) {
    JointPosition_trig = false;

  }

  ~JointLimitsAvoidance(){
  }

  bool configureHook() {
  	this->ports()->addEventPort("JointPosition", port_JointPosition, boost::bind(&JointLimitsAvoidance::JointPosition_onData, this, _1)).doc("");

	this->ports()->addPort("JointTorqueCommand", port_JointTorqueCommand).doc("");


    // Start of user code configureHook
    // TODO Put implementation of configureHook here !!!
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


  bool JointPosition_trig;

  // Start of user code userData
  // TODO userData !!!
  // End of user code

};

ORO_CREATE_COMPONENT(JointLimitsAvoidance)

