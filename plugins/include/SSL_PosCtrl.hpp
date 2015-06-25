/*
 * SSL_PosCtrl.hpp
 *
 * This header contains code for calculating the desired velocities of a diffy
 * drive robot to navigate to a given position
 */

// Includes
#include <math.h>
#include <gazebo/math/gzmath.hh>

// Macros
#define X_POS 0
#define Y_POS 1
#define THETA 2

#define KL 1
#define KA 1

#define V_MAX 1

// Function Prototypes
/*
 * goto_point
 *
 * Calculates the desired left and right wheel velocities to drive the robot
 * toward some desired position
 *
 * v_l - pointer toward variable to contain the left velocity
 * v_r - pointer toward variable to contain the right velocity
 * state - Array containing state of the robot, [x_pos, y_pos, theta]
 * p_des - Array containing the desired position, [x_pos, y_pos]
 */
void goto_point(double* v_l, double* v_r, double* state, double* p_des);

/*
 * balance_plane_1d
 *
 * Calculates the left and right wheel velocity for the robot swarm to balance
 * the plane they are driving upon.
 *
 * v_l - pointer toward variable to contain the left wheel velocity
 * v_r - pointer toward variable to contain the right wheel velocity
 * rob_pose - pose of the robot
 * plane_pose - pose of the plane
 * m - mass of robot
 * m_t total mass of swarm
 */
void balance_plane_1d(double* v_l, double* v_r, gazebo::math::Pose rob_pose,
	gazebo::math::Pose plane_pose, double m, double m_t);

// Function Definitions
void goto_point(double* v_l, double* v_r, double* state, double* p_des)
{
	double err_x = p_des[X_POS] - state[X_POS];
	double err_y = p_des[Y_POS] - state[Y_POS];
	
	double err_theta = atan2(err_y, err_x) - state[THETA];
	double err_dist = sqrt(err_x*err_x + err_y*err_y);
	
	double lin_vel = KL*err_dist;
	double ang_vel = KA*err_theta;
	
	*v_l = lin_vel - ang_vel;
	*v_r = lin_vel + ang_vel;
	
	if (*v_l > V_MAX)
	{
		*v_l = V_MAX;
	}
	else if (*v_l < -V_MAX)
	{
		*v_l = -V_MAX;
	}
	
	if (*v_r > V_MAX)
	{
		*v_r = V_MAX;
	}
	else if (*v_r < -V_MAX)
	{
		*v_r = -V_MAX;
	}
}

void balance_plane_1d(double* v_l, double* v_r, gazebo::math::Pose rob_pose,
	gazebo::math::Pose plane_pose, double m, double m_t)
{
	   
}

