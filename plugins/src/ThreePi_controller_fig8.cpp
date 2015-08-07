/*
 * ThreePi_controller_fig8.cpp
 * 
 * Gazebo Plugin for driving three pi in a figure 8.
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

// Custom headers
#include <SSL_PosCtrl.hpp>

namespace gazebo
{
	class ThreePiCTRL_POS : public ModelPlugin
	{
		private:
		physics::WorldPtr _world;
		physics::ModelPtr _model;
		physics::JointPtr _Motor_l;
		physics::JointPtr _Motor_r;
		event::ConnectionPtr updateConnection;
		
		public:
		ThreePiCTRL_POS()
		{
		}
		
		~ThreePiCTRL_POS()
		{
		}
		
		void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
		{
			// Store the pointer to the 3pi model
			_model = _parent;
			// Store the pointer to the world
			_world = _model->GetWorld();
			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ThreePiCTRL_POS::OnUpdate, this, _1));
			this->_Motor_l = this->_model->GetJoint("left_wheel_joint");
			this->_Motor_r = this->_model->GetJoint("right_wheel_joint");
		}
		
		void Init()
		{
		}
		
		void OnUpdate(const common::UpdateInfo & /*_info*/)
		{
			// Relevant Constants
			const double KP = 55.0;
			const double KD = 6.0;
			const double WHEEL_RADIUS = 0.015;
			const double TAU_MAX = 63.55;
			const double R = 1.0;
			const double omega = 1.0/32.0;
			
			// calculate desired position
			static int phase = 0;
			double t = _world->GetSimTime().Double();
			double des_pos[2];
			
			if (!phase)
			{
				des_pos[0] = R * sin(
			}
			
			double v_dl;
			double v_dr;
			double state[3];
			
			// Static variables
			static double lerr_l = 0.0;
			static double lerr_r = 0.0;
			static int iter = 0;
			
			// Get the state
			math::Pose pose = this->_model->GetWorldPose();
			state[0] = pose.pos.x;
			state[1] = pose.pos.y;
			state[2] = pose.rot.GetYaw();
			
			// Get Desired velocities
			goto_point(&v_dl, &v_dr, state , des_pos);
			
			// Get Actual velocities
			double v_al = this->_Motor_l->GetVelocity(0)*WHEEL_RADIUS;
			double v_ar = this->_Motor_r->GetVelocity(0)*WHEEL_RADIUS;
			
			// Do Control
			double err_l = v_dl - v_al;
			double err_r = v_dr - v_ar;
			double derr_l = err_l - lerr_l;
			double derr_r = err_r - lerr_r;
			double tau_l = KP*err_l + KD*lerr_r;
			double tau_r = KP*err_r + KD*lerr_r;
			
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
			
			this->_Motor_l->SetForce(0, tau_l);
			this->_Motor_r->SetForce(0, tau_r);
			
			// update static variables
			lerr_l = err_l;
			lerr_r = err_r;
			iter++;
		}
	};
	
	GZ_REGISTER_MODEL_PLUGIN(ThreePiCTRL_POS)
}

