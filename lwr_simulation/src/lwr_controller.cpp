#include <kdl_parser/kdl_parser.hpp>
#include <lwr_simulator/lwr_controller.h>

////////////////////////////////////////////////////////////////////////////////
// Constructor
LWRController::LWRController(std::string const& name) : 
    TaskContext(name),
    robotPrefix_("lwr")
{
    // Add required gazebo interfaces
    this->provides("gazebo")->addOperation("configure",&LWRController::gazeboConfigureHook,this,RTT::ClientThread);
    this->provides("gazebo")->addOperation("update",&LWRController::gazeboUpdateHook,this,RTT::ClientThread);

  this->addProperty("robotPrefix", robotPrefix_);

  this->ports()->addPort("JointPositionCommand", port_JointPositionCommand).doc("");
  this->ports()->addPort("JointTorqueCommand", port_JointTorqueCommand).doc("");

  this->ports()->addPort("CartesianWrench", port_CartesianWrench).doc("");
  this->ports()->addPort("JointVelocity", port_JointVelocity).doc("");
  this->ports()->addPort("CartesianVelocity", port_CartesianVelocity).doc("");
  this->ports()->addPort("CartesianPosition", port_CartesianPosition).doc("");
  this->ports()->addPort("MassMatrix", port_MassMatrix).doc("");
  this->ports()->addPort("Jacobian", port_Jacobian).doc("");
  this->ports()->addPort("JointTorque", port_JointTorque).doc("");
  this->ports()->addPort("JointPosition", port_JointPosition).doc("");
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
LWRController::~LWRController()
{
}

void LWRController::updateHook()
{
  // Synchronize with gazeboUpdate()
  RTT::os::MutexLock lock(gazebo_mutex_);
}

bool LWRController::startHook()
{
  return true;
}

bool LWRController::configureHook()
{
  return true;
}

bool LWRController::gazeboConfigureHook(gazebo::physics::ModelPtr model)
{
  if(model.get() == NULL) {return false;}
  
  // get parameter name
//  if (_sdf->HasElement("robotPrefix"))
//    this->robotPrefix = _sdf->GetElement("robotPrefix")->GetValueString();
  chain_start = std::string("calib_") + this->robotPrefix_ + "_arm_base_link";
//  if (_sdf->HasElement("baseLink"))
//    this->chain_start = _sdf->GetElement("baseLink")->GetValueString();

  chain_end = this->robotPrefix_ + "_arm_7_link";
//  if (_sdf->HasElement("toolLink"))
//    this->chain_end = _sdf->GetElement("toolLink")->GetValueString();
  
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }
  this->rosnode_ = new ros::NodeHandle();
    
  GetRobotChain();
  
  jnt_pos_.resize(7);
  jnt_vel_.resize(7);

  jnt_pos_cmd_.resize(7);
  jnt_trq_cmd_.resize(7);
  
  for(unsigned int i = 0; i< 7; i++)
  {
    // fill in gazebo joints pointer
    std::string joint_name = this->robotPrefix_ + "_arm_" + (char)(i + 48) + "_joint";
    gazebo::physics::JointPtr joint = model->GetJoint(joint_name);     
    if (joint)
    {
      this->joints_.push_back(joint);
    }
    else
    {
      this->joints_.push_back(gazebo::physics::JointPtr());  // FIXME: cannot be null, must be an empty boost shared pointer
      RTT::log(RTT::Error) << "A joint named " << joint_name.c_str() << " is not part of Mechanism Controlled joints." << RTT::endlog();
    }
    
    //if(_sdf->HasElement(joint_name)) {
    //  double init = 0.0; //_sdf->GetElement(joint_name)->GetValueDouble();
    //  joint->SetAngle(0, init);
    //}
    
    stiffness_(i) = 200.0;
    damping_(i) = 5.0;
    trq_cmd_(i) = 0;
    joint_pos_cmd_(i) = joints_[i]->GetAngle(0).Radian();
  }
  return true;
}

void LWRController::GetRobotChain()
{
  KDL::Tree my_tree;
  std::string robot_desc_string;
  rosnode_->param("robot_description", robot_desc_string, std::string());
  if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
    ROS_ERROR("Failed to construct kdl tree");
  }

  my_tree.getChain(chain_start, chain_end, chain_);
  
  dyn = new KDL::ChainDynParam(chain_, KDL::Vector(0.0, 0.0, -9.81));
  fk = new KDL::ChainFkSolverPos_recursive(chain_);
  jc = new KDL::ChainJntToJacSolver(chain_);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void LWRController::gazeboUpdateHook(gazebo::physics::ModelPtr model)
{
  KDL::Frame f;
  KDL::Jacobian jac(7);
  KDL::JntSpaceInertiaMatrix H(7);
  KDL::JntArray pos(7);
  KDL::JntArray grav(7);
  Matrix77d mass;
  
  for(unsigned int i = 0; i< 7; i++)
  {
    jnt_pos_[i] = pos(i) = joint_pos_(i) = joints_[i]->GetAngle(0).Radian();
    jnt_vel_[i] =joint_vel_(i) = joints_[i]->GetVelocity(0);
  }

  dyn->JntToGravity(pos, grav);
  fk->JntToCart(pos, f);
  /*
  m_msr_data.data.msrCartPos[0] = f.M.data[0];
  m_msr_data.data.msrCartPos[1] = f.M.data[1];
  m_msr_data.data.msrCartPos[2] = f.M.data[2];
  m_msr_data.data.msrCartPos[3] = f.p.data[0];
  
  m_msr_data.data.msrCartPos[4] = f.M.data[3];
  m_msr_data.data.msrCartPos[5] = f.M.data[4];
  m_msr_data.data.msrCartPos[6] = f.M.data[5];
  m_msr_data.data.msrCartPos[7] = f.p.data[1];
  
  m_msr_data.data.msrCartPos[8] = f.M.data[6];
  m_msr_data.data.msrCartPos[9] = f.M.data[7];
  m_msr_data.data.msrCartPos[10] = f.M.data[8];
  m_msr_data.data.msrCartPos[11] = f.p.data[2];
  */
  jc->JntToJac(pos, jac);
  jac.changeRefFrame(KDL::Frame(f.Inverse().M));
  dyn->JntToMass(pos, H);
  for(unsigned int i=0;i<7;i++) {
    for(unsigned int j=0;j<7;j++) {
      mass(i, j) = H.data(i, j);
    }
  }
  
  if (port_JointTorqueCommand.read(jnt_trq_cmd_) == RTT::NewData) {
    for (unsigned int i = 0; i < 7; i++) {
      trq_cmd_(i) = jnt_trq_cmd_(i);
    }
  }
  
  if (port_JointPositionCommand.read(jnt_pos_cmd_) == RTT::NewData) {
      for (unsigned int i = 0; i < 7; i++) {
        joint_pos_cmd_(i) = jnt_pos_cmd_(i);
      }
  }
  
  trq_ = stiffness_.asDiagonal() * (joint_pos_cmd_ - joint_pos_) - damping_.asDiagonal() * joint_vel_ + trq_cmd_;

  for(unsigned int i = 0; i< 7; i++) {
    joints_[i]->SetForce(0, trq_(i));
  }
  
  port_JointPosition.write(jnt_pos_);
  port_JointVelocity.write(jnt_vel_);
  port_JointTorque.write(jnt_trq_);

//  port_CartesianPosition.write(cart_pos);
//  port_CartesianVelocity.write(cart_twist);
//  port_CartesianWrench.write(cart_wrench);

  port_Jacobian.write(jac);
  port_MassMatrix.write(mass);
  
}

ORO_LIST_COMPONENT_TYPE(LWRController)
ORO_CREATE_COMPONENT_LIBRARY();

