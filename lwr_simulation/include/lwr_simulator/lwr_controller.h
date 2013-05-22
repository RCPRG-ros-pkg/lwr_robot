/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: 3D position interface.
 * Author: Sachin Chitta and John Hsu
 * Date: 10 June 2008
 * SVN: $Id$
 */
#ifndef LWR_CONTROLLER_HH
#define LWR_CONTROLLER_HH

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include "physics/physics.hh"
#include "transport/TransportTypes.hh"
#include "common/Time.hh"
#include "common/Plugin.hh"
#include "common/Events.hh"
#include <ros/ros.h>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include "Eigen/Dense"
#include <kuka_lwr_fri/friComm.h>

#include <sys/socket.h> /* for bind socket accept */
#include <unistd.h> /* for close() */
#include <arpa/inet.h>/* for inet_Addr etc*/

namespace gazebo
{

   class LWRController : public ModelPlugin
   {
      public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      /// \brief Constructor
      public: LWRController();

      /// \brief Destructor
      public: virtual ~LWRController();

      /// \brief Load the controller
      public: void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

      /// \brief Update the controller
      protected: virtual void UpdateChild();
      
      void GetRobotChain();
      
      private:
        /*
         *  \brief pointer to ros node
         */
        ros::NodeHandle* rosnode_;
        
        gazebo::physics::ModelPtr parent_model_;
        std::string robotPrefix;
        std::vector<gazebo::physics::JointPtr>  joints_;
        std::string chain_start, chain_end;
          
         // Pointer to the model
        physics::WorldPtr world;

        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;
        
        KDL::Chain chain_;
	      KDL::ChainDynParam *dyn;
	      KDL::ChainFkSolverPos_recursive *fk;
	      KDL::ChainJntToJacSolver *jc;

        std::string base_frame_;
	
	      int cnt;
        
        Eigen::Matrix<double, 7, 1> joint_pos_;
        Eigen::Matrix<double, 7, 1> joint_pos_cmd_;
        Eigen::Matrix<double, 7, 1> joint_vel_;
        Eigen::Matrix<double, 7, 1> stiffness_;
        Eigen::Matrix<double, 7, 1> damping_;
        Eigen::Matrix<double, 7, 1> trq_cmd_;
        Eigen::Matrix<double, 7, 1> trq_;
        
	      int remote_port;
	      std::string remote;
	
        int socketFd;
        struct sockaddr_in localAddr, remoteAddr;

        tFriMsrData m_msr_data;
	      tFriCmdData m_cmd_data;
   };

}

#endif

