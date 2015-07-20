/*
 * Plane1DRecorder.cpp
 * 
 * Gazebo Plugin for recording the Plane state.
 */

// Gazebo Headers
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>

// Boost Headers
#include <boost/bind.hpp>

// Standard Headers
#include <fstream>
#include <stdio.h>

// Custom headers
#include <SSL_PosCtrl.hpp>

namespace gazebo
{
	class Plane1DRec : public ModelPlugin
	{
		private:
		physics::WorldPtr _world;
		physics::ModelPtr _model;
		physics::JointPtr _joint;
		event::ConnectionPtr updateConnection;
		std::ofstream out_file;
		
		public:
		Plane1DRec(){}
		
		~Plane1DRec()
		{
	 		 out_file.close();
		}
		
		void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
		{
			// Store the pointer to the 3pi model
			_model = _parent;
			_world = _model->GetWorld();
			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Plane1DRec::OnUpdate, this, _1));
			this->_joint = this->_model->GetJoint("main_joint");
		}
		
		void Init()
		{
			char file_name[128];
			sprintf(file_name, "ThreePiSim_Plane.csv");
			out_file.open(file_name);
		}
		
		void OnUpdate(const common::UpdateInfo & /*_info*/)
		{
			// write data
			out_file << _world->GetSimTime().Double() << "," << _joint->GetAngle(0).Radian() << ',' << _joint->GetVelocity(0) << std::endl;
		}
	}; 
	
	GZ_REGISTER_MODEL_PLUGIN(Plane1DRec)
}

