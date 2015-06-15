#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <stdio.h>
#include <fstream>

namespace gazebo
{
  class PDController : public ModelPlugin
  {
    private: physics::WorldPtr _world;
    private: physics::ModelPtr _model;
    private: physics::JointPtr _j1, _j2;
    //private: std::ofstream _output_current, _output_des;
		private: std::ofstream log;

    public: PDController()
    {
    }

    public: ~PDController()
    {
      //_output_current.close();
      //_output_des.close();
			log.close();
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      _model = _parent;

      // store the pointer to the world
      _world = _model->GetWorld();

     // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&PDController::OnUpdate, this, _1));

      // get the joints
      this->_j1 = _model->GetJoint("left_wheel_joint");
      this->_j2 = _model->GetJoint("right_wheel_joint");

    }

    // open up the file for writing 
    public: void Init()
    {
      //_output_current.open("PD_hold.state");
      //_output_des.open("PD_hold.desired");
			log.open("threepiPD.csv");

      // set initial joint positions
      //this->_j1->SetPosition(0, -M_PI_2);
      //this->_j2->SetPosition(0, 0.0);
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      //const double CHANGEME = 0.0; //yo no se
			//log.open("/home/caleb/datalog.txt");

      // set the desired positions and velocities
      //const double J1_DES = 0.0, J2_DES = 0.0;
      const double vJ1_DES = 0.10, vJ2_DES = 0.10;
			static double lerr1 = 0, lerr2 = 0;

      // setup gains 
      const double KP = 1.0, KD = 0.0;

      //_output_current << _j1->GetAngle(0).Radian() << " " << _j2->GetAngle(0).Radian() << std::endl;
      //_output_des << vJ1_DES << " " << vJ2_DES << std::endl;

      // get current positions and velocities
      //double theta1 = _j1->GetAngle(0).Radian();  
      double w1 = _j1->GetVelocity(0);  
      //double theta2 = _j2->GetAngle(0).Radian();  
      double w2 = _j2->GetVelocity(0);  
			
			/*
			double lerr1, lerr2;
			if(J1_DES == 0 || J2_DES == 0){
				if(i == 0){
					lerr1 = 0;
					lerr2 = 0;
					i = i+1
				}
				else{
					lerr1 = perr1;
					lerr2 = perr2;
				}
		    double perr1 = vJ1_DES - dtheta1;
		    double perr2 = vJ2_DES - dtheta2;
				//double derr1 = vJ1_DES - dtheta1;
				//double derr1 = vJ2_DES - dtheta2;
		    double derr1 = (perr1 - lerr1);
		    double derr2 = (perr2 - lerr2);
			}
			else{
				if(i == 0){
					lerr1 = 0;
					lerr2 = 0;
					i = i+1
				}
				else{
					lerr1 = perr1;
					lerr2 = perr2;
				}
		    double perr1 = J1_DES - theta1;
		    double perr2 = J2_DES - theta2;
		    double derr1 = (perr1 - lerr1);
		    double derr2 = (perr2 - lerr2);				
			}
			*/

			double err1 = vJ1_DES - w1*0.015;
			double err2 = vJ2_DES - w2*0.015;
			double derr1 = err1 - lerr1;
			double derr2 = err2 - lerr2;
			lerr1 = err1;
			lerr2 = err2;
		

      const double tau1 = KP*err1+KD*derr1;
      const double tau2 = KP*err2+KD*derr2;

			static int i = 1;
      // set torques

			if(i < 41){
      this->_j1->SetForce(0, tau1);
      this->_j2->SetForce(0, tau2);
			}
			else{
			this->_j1->SetForce(0, 0);
      this->_j2->SetForce(0, 0);
			}
			
			
			//record data
			//gazebo::common::Time t1 = _world->GetSimTime();
			int t1 = 1;
			log << i << "," << t1 << "," << w1 << "," << w2 << "," << err1 << "," << err2 << "," << derr1 << "," << derr2 << "," << vJ1_DES << "," << vJ2_DES << "," << tau1 << "," << tau2 << std::endl;
			i++;
    }

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PDController)
}

