
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

// Start of user code includes
// TODO Put includes here.
// End of user code

class CartesianImpedance : public RTT::TaskContext {
public:
  CartesianImpedance(const std::string & name) : TaskContext(name) {

  }

  ~CartesianImpedance(){
  }

  bool configureHook() {



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
  }

private:







  // Start of user code userData
  // TODO userData !!!
  // End of user code

};

ORO_CREATE_COMPONENT(CartesianImpedance)

