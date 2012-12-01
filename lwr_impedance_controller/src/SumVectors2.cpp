
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

// Start of user code includes
// TODO Put includes here.
// End of user code

class SumVectors2 : public RTT::TaskContext {
public:
  SumVectors2(const std::string & name) : TaskContext(name) {
    Input2_trig = false;
    Input1_trig = false;

	

    this->ports()->addEventPort("Input2", port_Input2, boost::bind(&SumVectors2::Input2_onData, this, _1)).doc("");
    this->ports()->addEventPort("Input1", port_Input1, boost::bind(&SumVectors2::Input1_onData, this, _1)).doc("");

    this->ports()->addPort("Output", port_Output).doc("");
  }

  ~SumVectors2(){
  }

  bool configureHook() {
    // Start of user code configureHook
    in1_.resize(7);
    in2_.resize(7);
    out_.resize(7);
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
	if(Input1_trig && Input2_trig &&  true) {
      doSum();
      Input1_trig = false;
      Input2_trig = false;
    }
  }

private:

  void doSum() {
    // Start of user code Sum
    port_Input1.read(in1_);
    port_Input2.read(in2_);

    if(in1_.size() == in2_.size()) {
      out_.resize(in1_.size());
      for(unsigned int i = 0; i < in1_.size(); i++) {
        out_[i] = in1_[i] + in2_[i];
      }
      port_Output.write(out_);
    } else {
      RTT::log(RTT::Error) << "vector size mismatch !!!" << RTT::endlog();
    }
  // End of user code
  }

  void Input2_onData(RTT::base::PortInterface* port) {
    Input2_trig = true;
  }
  void Input1_onData(RTT::base::PortInterface* port) {
    Input1_trig = true;
  }

  RTT::InputPort<std::vector<double> > port_Input2;
  RTT::InputPort<std::vector<double> > port_Input1;

  RTT::OutputPort<std::vector<double> > port_Output;


  bool Input2_trig;
  bool Input1_trig;

  // Start of user code userData
  std::vector<double> in1_, in2_, out_;
  // End of user code

};

ORO_CREATE_COMPONENT(SumVectors2)

