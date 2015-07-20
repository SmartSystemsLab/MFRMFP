/*
 * ThreePiRecorder.cpp
 * 
 * Gazebo Plugin for recording the ThreePi state.
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
	class ThreePiRec : public ModelPlugin
	{
		private:
		physics::WorldPtr _world;
		physics::ModelPtr _model;
		physics::JointPtr _Motor_l;
		physics::JointPtr _Motor_r;
		event::ConnectionPtr updateConnection;
		std::ofstream out_file;
		//static int id;
		
		public:
		ThreePiRec()
		{
			//id++;
		}
		
		~ThreePiRec()
		{
			out_file.close();
			//id--;
		}
		
		void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
		{
			// Store the pointer to the 3pi model
			_model = _parent;
			_world = _model->GetWorld();
			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ThreePiRec::OnUpdate, this, _1));
			this->_Motor_l = this->_model->GetJoint("left_wheel_joint");
			this->_Motor_r = this->_model->GetJoint("right_wheel_joint");
		}
		
		void Init()
		{
			char file_name[128];
			sprintf(file_name, "ThreePiSim_Rob.csv");
			out_file.open(file_name);
		}
		
		void OnUpdate(const common::UpdateInfo & /*_info*/)
		{
			math::Pose pose = this->_model->GetWorldPose();
			// write data
			out_file << _world->GetSimTime().Double() << "," << pose.pos.x << ',' << pose.pos.y << ',' << pose.pos.z << ',' << pose.rot.w << ',' << pose.rot.x << ',' << pose.rot.y << ',' << pose.rot.z << "," << _Motor_l->GetVelocity(0) << "," << _Motor_r->GetVelocity(0) << std::endl;
		}
	};
	
	GZ_REGISTER_MODEL_PLUGIN(ThreePiRec)
}

