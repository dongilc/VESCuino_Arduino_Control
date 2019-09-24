
/*
 * computed_torque_control.h
 *
 *  Created on: March 29, 2018
 *      Author: cdi
 */

#ifndef COMPUTED_TORQUE_CONTROL_H_
#define COMPUTED_TORQUE_CONTROL_H_

#include <math.h>

//
#define CURRENT_LIMIT	20.0

// Declare Variables
#define TO_STRING(var)	#var
#define RAD2DEG			180.0/M_PI  // radian to deg
#define DEG2RAD			M_PI/180.0  // deg to radian
#define RPM2RPS			2.*M_PI/60.0 // rpm to rad/sec
#define DPS2RPM			60.0/360.0  // deg/sec to rpm
#define RPM2DPS			360.0/60.0  // rpm to deg/sec

// CTM Status Enum
typedef enum {
	CTM_NO_ERROR = 0,
	CTM_CURRENT_LIMIT
} CTM_STATUS;

// Computed Torque Method Class
class CTM
{
public:
	//
	float* Dir;	// Wheel Direction Correction
	float* V_dyn;
	float* Motor_Kt;
	float** M_dyn;

	float* rpm;
	float* rps;
	float* rps_lpf;
	float* rad;
	float* rev;
	float* rps_goal;
	float* rad_goal;
	float* vel_cartesian;
	float* vel_cartesian_goal;
	float* pos_cartesian;
	float* pos_cartesian_goal;

	int ctm_cnt = 0;

	int NO_OF_WHEEL;
	int DOF;
	int POLEPAIR;
	float ERPM2RPM;
	float TACHO2REV;
	float ROW;
	float DCW_X;
	float DCW_Y;
	float DT;
	float FREQ;
	//
	//ros::Publisher ctm_pub;
	//amg_p1::CTM_Message ctm_msg;

	CTM(int number_of_wheel, int degree_of_freedom, int polepair_num, float radius_of_wheel, float dcw_x, float dcw_y, int rate_hz,
			int* dir_vector, float (*inertial_vector)[10], float* friction_vector, float* motor_kt)
	{
		// basic parameters
		NO_OF_WHEEL = number_of_wheel;
		DOF = degree_of_freedom;
		POLEPAIR = polepair_num;
		ERPM2RPM = 1./POLEPAIR;
		TACHO2REV = 1. / (3.*2.*POLEPAIR);
		ROW = radius_of_wheel;
		DCW_X = dcw_x;
		DCW_Y = dcw_y;
		DT = 1./rate_hz;
		FREQ = rate_hz;

		// speed calculation variables
		rpm = new float[NO_OF_WHEEL];
		rps = new float[NO_OF_WHEEL];
		rps_lpf = new float[NO_OF_WHEEL];
		rad = new float[NO_OF_WHEEL];
		rev = new float[NO_OF_WHEEL];
		rps_goal = new float[NO_OF_WHEEL];
		rad_goal = new float[NO_OF_WHEEL];
		vel_cartesian = new float[DOF];
		vel_cartesian_goal = new float[DOF];
		pos_cartesian = new float[DOF];
		pos_cartesian_goal = new float[DOF];

		// CTM variables
		Dir = new float[NO_OF_WHEEL];
		V_dyn = new float[NO_OF_WHEEL];
		Motor_Kt = new float[NO_OF_WHEEL];
		M_dyn = new float*[NO_OF_WHEEL];
		for (int i = 0; i < NO_OF_WHEEL; i++)	M_dyn[i] = new float[NO_OF_WHEEL];

		for (int i = 0; i < NO_OF_WHEEL; i++) {
			Dir[i] = dir_vector[i];
			V_dyn[i] = friction_vector[i];
			Motor_Kt[i] = motor_kt[i];
			for (int j = 0; j < NO_OF_WHEEL; j++)	M_dyn[i][j] = inertial_vector[i][j];
		}
	}

	~CTM() {
		delete[] Dir;
		delete[] V_dyn;
		delete[] Motor_Kt;
		for(int i=0; i<NO_OF_WHEEL; i++)	delete[] M_dyn[i];
		delete[] M_dyn;

		delete[] rpm;
		delete[] rps;
		delete[] rps_lpf;
		delete[] rad;
		delete[] rev;
		delete[] rps_goal;
		delete[] rad_goal;
		delete[] vel_cartesian;
		delete[] vel_cartesian_goal;
		delete[] pos_cartesian;
		delete[] pos_cartesian_goal;
	}

	void Init();
	//void RunCTM_JointSpace(float *erpm, int *tachometer, float *current_vel_cartesian, float *target_vel_cartesian, float *curr_out, float *kp_joint, float *kd_joint);
	void RunCTM_Mecanum_JointSpace(float *enc_rps, float *enc_rad, float *current_vel_cartesian, float *target_vel_cartesian, float *curr_out, float *kp_joint, float *kd_joint);
	void RunCTM_Mecanum_TaskSpace(float *enc_rps, float *enc_rad, float *current_vel_cartesian, float *target_vel_cartesian, float *curr_out, float *kp_task, float *kd_task);
	//void RunCTM_TaskSpace(float *erpm, int *tachometer, float *current_vel_cartesian, float *target_vel_cartesian, float *curr_out, float *kp_task, float *kd_task);
	float mat_Integral(float in, float out_last);
	float mat_LowpassFilter(float in, float out_last, float hz);
	float velocity_profile_Filter(float s_goal, float s_now, float v_goal, float v_max, float a_max);
	float min_float(float d1, float d2, float d3);
	float max_float(float d1, float d2, float d3);
};

#endif
