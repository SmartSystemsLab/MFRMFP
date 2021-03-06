/*
 * ThreePi_controller_PD.cpp
 * 
 * Gazebo Plugin for controlling the ThreePi using a PD controller.
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

namespace gazebo
{
	class ThreePiCTRL_PD : public ModelPlugin
	{
		private:
		physics::WorldPtr _world;
		physics::ModelPtr _model;
		physics::ModelPtr _plane;
		physics::JointPtr _Motor_l;
		physics::JointPtr _Motor_r;
		event::ConnectionPtr updateConnection;
		std::ofstream out_file;
		std::ofstream pose_file;
		
		public:
		ThreePiCTRL_PD()
		{
		}
		
		~ThreePiCTRL_PD()
		{
			out_file.close();
			pose_file.close();
		}
		
		void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
		{
			// Store the pointer to the 3pi model
			_model = _parent;
			// Store the pointer to the world
			_world = _model->GetWorld();
			// Store the pointer to the plae
			_plane = _world->GetModel("1DPlane");
			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ThreePiCTRL_PD::OnUpdate, this, _1));
			this->_Motor_l = this->_model->GetJoint("left_wheel_joint");
			this->_Motor_r = this->_model->GetJoint("right_wheel_joint");
		}
		
		void Init()
		{
			out_file.open("ThreePiSim_out.csv");
			pose_file.open("ThreePiSim_pose.csv");
		}
		
		void OnUpdate(const common::UpdateInfo & /*_info*/)
		{
			// Relevant Constants
			const double KP = 55.0;
			const double KD = 6.0;
			const double v_dl = 0.1;
			const double v_dr = 0.1;
			const double WHEEL_RADIUS = 0.015;
			const double TAU_MAX = 63.55;
			
			// Static variables
			static double lerr_l = 0.0;
			static double lerr_r = 0.0;
			static int iter = 0;
			
			double v_al = this->_Motor_l->GetVelocity(0)*WHEEL_RADIUS;
			double v_ar = this->_Motor_r->GetVelocity(0)*WHEEL_RADIUS;
			
			double err_l = v_dl - v_al;
			double err_r = v_dr - v_ar;
			double derr_l = err_l - lerr_l;
			double derr_r = err_r - lerr_r;
			double tau_l = KP*err_l + KD*lerr_r;
			double tau_r = KP*err_r + KD*lerr_r;
			
			common::Time time = _world->GetSimTime();
			double sim_time = time.Double(); 
			
			if (tau_l > TAU_MAX)
			{
				tau_l = TAU_MAX;
			}
			else if (tau_l < -TAU_MAX)
			{
				tau_l = -TAU_MAX;
			}
			
			if (tau_r > TAU_MAX)
			{
				tau_r = TAU_MAX;
			}
			else if (tau_r < -TAU_MAX)
			{
				tau_r = -TAU_MAX;
			}
			
			//tau_l = 0.05;
			//tau_r = 0.05;
			
			this->_Motor_l->SetForce(0, tau_l);
			this->_Motor_r->SetForce(0, tau_r);
			
			math::Pose threepipose = this->_model->GetWorldPose();
			math::Pose planepose = this->_plane->GetWorldPose();

			// write data
			pose_file << threepipose << ";;" << planepose << std::endl;
			out_file << iter << ','<< sim_time << ',' << v_dl << ',' << v_dr << ',' << v_al << ',' << v_ar << ',' << err_l << ',' << err_r << ',' << derr_l << ',' << derr_r << ',' << tau_l << ',' << tau_r << std::endl;
			
			// update static variables
			lerr_l = err_l;
			lerr_r = err_r;
			iter++;
		}
	};
	
	GZ_REGISTER_MODEL_PLUGIN(ThreePiCTRL_PD)
}

