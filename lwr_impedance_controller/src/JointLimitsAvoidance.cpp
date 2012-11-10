
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


  	this->ports()->addEventPort("JointPosition", port_JointPosition, boost::bind(&JointLimitsAvoidance::JointPosition_onData, this, _1)).doc("");

	this->ports()->addPort("JointTorqueCommand", port_JointTorqueCommand).doc("");
  }

  ~JointLimitsAvoidance(){
  }

  bool configureHook() {

    // Start of user code configureHook
	jnt_pos_.resize(7);
	jnt_trq_.resize(7);
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
	port_JointPosition.read(jnt_pos_);

	jnt_trq_[0] = jointLimitTrq(2.96, -2.96, 0.26, 10.0, jnt_pos_[0]);
	jnt_trq_[1] = jointLimitTrq(2.09, -2.09, 0.26, 10.0, jnt_pos_[1]);
	jnt_trq_[2] = jointLimitTrq(2.96, -2.96, 0.26, 10.0, jnt_pos_[2]);
	jnt_trq_[3] = jointLimitTrq(2.09, -2.09, 0.26, 10.0, jnt_pos_[3]);
	jnt_trq_[4] = jointLimitTrq(2.96, -2.96, 0.26, 10.0, jnt_pos_[4]);
	jnt_trq_[5] = jointLimitTrq(2.09, -2.09, 0.26, 10.0, jnt_pos_[5]);
	jnt_trq_[6] = jointLimitTrq(2.96, -2.96, 0.26, 10.0, jnt_pos_[6]);

	port_JointTorqueCommand.write(jnt_trq_);
	// End of user code
  }

  void JointPosition_onData(RTT::base::PortInterface* port) {
    JointPosition_trig = true;
  }

  RTT::InputPort<std::vector<double> > port_JointPosition;

  RTT::OutputPort<std::vector<double> > port_JointTorqueCommand;


  bool JointPosition_trig;

  // Start of user code userData
  std::vector<double> jnt_pos_;
  std::vector<double> jnt_trq_;

  double jointLimitTrq(double hl, double ll, double ls, double r_max, double q) {
    if(q > (hl - ls)) {
      return -1 * ((q - hl + ls)/ls) * ((q - hl + ls)/ls) * r_max;
    } else if(q < (ll + ls)) {
      return ((ll + ls - q)/ls) * ((ll + ls - q)/ls) * r_max;
    } else {
      return 0.0;
    }
  }
  // End of user code

};

ORO_CREATE_COMPONENT(JointLimitsAvoidance)

