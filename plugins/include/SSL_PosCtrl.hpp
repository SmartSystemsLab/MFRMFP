/*
 * SSL_PosCtrl.hpp
 *
 * This header contains code for calculating the desired velocities of a diffy
 * drive robot to navigate to a given position
 */

// Includes
#include <math.h>

// Macros
#define X_POS 0
#define Y_POS 1
#define THETA 2

#define KL 1
#define KA 1

// Function Prototypes
/*
 * calc_des_vel
 *
 * Calculates the desired left and right wheel velocities to drive the robot
 * toward some desired position
 *
 * v_l - pointer toward variable to contain the left velocity
 * v_r - pointer toward variable to contain the right velocity
 * state - Array containing state of the robot, [x_pos, y_pos, theta]
 * p_des - Array containing the desired position, [x_pos, y_pos]
 */
void calc_des_vel(double* v_l, double* v_r, double* state, double* p_des);

// Function Definitions
void calc_des_vel(double* v_l, double* v_r, double* state, double* p_des)
{
	double err_x = p_des[X_POS] - state[X_POS];
	double err_y = p_des[Y_POS] - state[Y_POS];
	
	double err_theta = atan2(err_y, err_x) - state[THETA];
	double err_dist = sqrt(err_x*err_x + err_y*err_y);
	
	double lin_vel = KL*err_dist;
	double ang_vel = KA*err_theta;
	
	*v_l = lin_vel - ang_vel;
	*v_r = lin_vel + ang_vel;
}
