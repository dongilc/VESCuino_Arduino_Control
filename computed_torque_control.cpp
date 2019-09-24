/*
 * computed_torque_control.cpp
 *
 *  Created on: March 29, 2018
 *      Author: cdi
 */

#include "computed_torque_control.h"
#include "Arduino.h"

void CTM::RunCTM_Mecanum_JointSpace(float *enc_rps, float *enc_rad, float *current_vel_cartesian, float *target_vel_cartesian, float *curr_out, float *kp_joint, float *kd_joint)
{
	for (int i = 0; i < NO_OF_WHEEL; i++)
	{
		rps[i] = Dir[i] * enc_rps[i];
		rad[i] = Dir[i] * enc_rad[i];
	}

	// variables
	float thddot_cp[NO_OF_WHEEL];
	float T_M_dyn[NO_OF_WHEEL];
	float T_V_dyn[NO_OF_WHEEL];
	float T_SUM[NO_OF_WHEEL];
	float CURR_CTM[NO_OF_WHEEL];
	float CURR_CTM_LPF[NO_OF_WHEEL];
	int ctm_status;

	// Calculate Cartesian space velociy - Current values : vx(m/s), vy(m/s), wz(rad/s)
	current_vel_cartesian[0] = ROW / 4.*(rps[0] + rps[1] + rps[2] + rps[3]);
	current_vel_cartesian[1] = ROW / 4.*(-rps[0] + rps[1] + rps[2] - rps[3]);
	current_vel_cartesian[2] = ROW / 4. / (DCW_X + DCW_Y)*(-rps[0] + rps[1] - rps[2] + rps[3]);

	// Calculate joint space current velocity
	rps_goal[0] = (target_vel_cartesian[0] - target_vel_cartesian[1] - (DCW_X + DCW_Y)*target_vel_cartesian[2]) / ROW;
	rps_goal[1] = (target_vel_cartesian[0] + target_vel_cartesian[1] + (DCW_X + DCW_Y)*target_vel_cartesian[2]) / ROW;
	rps_goal[2] = (target_vel_cartesian[0] + target_vel_cartesian[1] - (DCW_X + DCW_Y)*target_vel_cartesian[2]) / ROW;
	rps_goal[3] = (target_vel_cartesian[0] - target_vel_cartesian[1] + (DCW_X + DCW_Y)*target_vel_cartesian[2]) / ROW;

	for (int i = 0; i < NO_OF_WHEEL; i++) {
		// Calculate joint space target velocity
		rad_goal[i] = mat_Integral(rps_goal[i], rad_goal[i]);

		// thetaddot_cp calculation
		thddot_cp[i] = kd_joint[i] * (rps_goal[i] - rps[i]) + kp_joint[i] * (rad_goal[i] - rad[i]);
	}
				
	// torque inertia term
	for (int i = 0; i < NO_OF_WHEEL; i++) {
		T_M_dyn[i] = M_dyn[i][0] * thddot_cp[0] + M_dyn[i][1] * thddot_cp[1] + M_dyn[i][2] * thddot_cp[2] + M_dyn[i][3] * thddot_cp[3];
	}
		
	// torque feed-forward due to viscous friction - only considered viscous friction on wheel (BL*wL, BR*wR)
	for (int i = 0; i < NO_OF_WHEEL; i++) {
		T_V_dyn[i] = V_dyn[i] * rps[i];
	}

	// torque out
	for (int i = 0; i < NO_OF_WHEEL; i++) {
		T_SUM[i] = T_M_dyn[i] + T_V_dyn[i];
	}

	// control current out
	for (int i = 0; i < NO_OF_WHEEL; i++) {
		CURR_CTM[i] = Dir[i] * T_SUM[i] / Motor_Kt[i];

		if (CURR_CTM[i] >= CURRENT_LIMIT)
		{
			curr_out[i] = CURRENT_LIMIT;
		}
		else if (CURR_CTM[i] <= -CURRENT_LIMIT)
		{
			curr_out[i] = -CURRENT_LIMIT;
		}
		else {
			curr_out[i] = CURR_CTM[i];
		}
	}

	if (ctm_cnt == FREQ) ctm_cnt = 0;

	ctm_cnt++;
}

void CTM::RunCTM_Mecanum_TaskSpace(float *enc_rps, float *enc_rad, float *current_vel_cartesian, float *target_vel_cartesian, float *curr_out, float *kp_task, float *kd_task)
{
	for (int i = 0; i < NO_OF_WHEEL; i++)
	{
		rps[i] = Dir[i] * enc_rps[i];
		rad[i] = Dir[i] * enc_rad[i];
	}

	// variables
	float Xddot_cp[DOF];
	float thddot_cp[NO_OF_WHEEL];
	float thddot_cp_temp[NO_OF_WHEEL];
	float T_M_dyn[NO_OF_WHEEL];
	float T_V_dyn[NO_OF_WHEEL];
	float T_SUM[NO_OF_WHEEL];
	float CURR_CTM[NO_OF_WHEEL];
	float CURR_CTM_LPF[NO_OF_WHEEL];
	int ctm_status;

	// Calculate Cartesian space velociy - Current values : vx(m/s), vy(m/s), wz(rad/s)
	vel_cartesian[0] = current_vel_cartesian[0] = ROW / 4.*(rps[0] + rps[1] + rps[2] + rps[3]);
	vel_cartesian[1] = current_vel_cartesian[1] = ROW / 4.*(-rps[0] + rps[1] + rps[2] - rps[3]);
	vel_cartesian[2] = current_vel_cartesian[2] = ROW / 4. / (DCW_X + DCW_Y)*(-rps[0] + rps[1] - rps[2] + rps[3]);

	// Calculate Cartesian space position - Current values : px(m), py(m), pz(rad)
	pos_cartesian[0] = ROW / 4.*(rad[0] + rad[1] + rad[2] + rad[3]);
	pos_cartesian[1] = ROW / 4.*(-rad[0] + rad[1] + rad[2] - rad[3]);
	pos_cartesian[2] = ROW / 4. / (DCW_X + DCW_Y)*(-rad[0] + rad[1] - rad[2] + rad[3]);

	// Calculate Cartesian space target velocity, position
	for (int i = 0; i < DOF; i++) {
		vel_cartesian_goal[i] = target_vel_cartesian[i];
		pos_cartesian_goal[i] = mat_Integral(target_vel_cartesian[i], pos_cartesian_goal[i]);
	}
	
	// Xddot_cp calculation
	for (int i = 0; i < DOF; i++)	Xddot_cp[i] = kd_task[i] * (vel_cartesian_goal[i] - vel_cartesian[i]) + kp_task[i] * (pos_cartesian_goal[i] - pos_cartesian[i]);

	// theataddot_cp calculation
	thddot_cp[0] = (Xddot_cp[0] - Xddot_cp[1] - (DCW_X + DCW_Y)*Xddot_cp[2]) / ROW;
	thddot_cp[1] = (Xddot_cp[0] + Xddot_cp[1] + (DCW_X + DCW_Y)*Xddot_cp[2]) / ROW;
	thddot_cp[2] = (Xddot_cp[0] + Xddot_cp[1] - (DCW_X + DCW_Y)*Xddot_cp[2]) / ROW;
	thddot_cp[3] = (Xddot_cp[0] - Xddot_cp[1] + (DCW_X + DCW_Y)*Xddot_cp[2]) / ROW;

	// torque inertia term
	for (int i = 0; i < NO_OF_WHEEL; i++) {
		T_M_dyn[i] = M_dyn[i][0] * thddot_cp[0] + M_dyn[i][1] * thddot_cp[1] + M_dyn[i][2] * thddot_cp[2] + M_dyn[i][3] * thddot_cp[3];
	}

	// torque feed-forward due to viscous friction - only considered viscous friction on wheel (BL*wL, BR*wR)
	for (int i = 0; i < NO_OF_WHEEL; i++) {
		T_V_dyn[i] = V_dyn[i] * rps[i];
	}

	// torque out
	for (int i = 0; i < NO_OF_WHEEL; i++) {
		T_SUM[i] = T_M_dyn[i] + T_V_dyn[i];
	}

	// control current out
	for (int i = 0; i < NO_OF_WHEEL; i++) {
		CURR_CTM[i] = Dir[i] * T_SUM[i] / Motor_Kt[i];

		if (CURR_CTM[i] >= CURRENT_LIMIT)
		{
			curr_out[i] = CURRENT_LIMIT;
		}
		else if (CURR_CTM[i] <= -CURRENT_LIMIT)
		{
			curr_out[i] = -CURRENT_LIMIT;
		}
		else {
			curr_out[i] = CURR_CTM[i];
		}
	}

	if (ctm_cnt == FREQ) ctm_cnt = 0;

	ctm_cnt++;
}

/*
void CTM::RunCTM_TaskSpace(float *erpm, int *tachometer, float *current_vel_cartesian, float *target_vel_cartesian, float *curr_out, float *kp_task, float *kd_task)
{
	for (int i = 0; i < NO_OF_WHEEL; i++) {
		rpm[i] = Dir[i] * erpm[i] * ERPM2RPM;			// Calculate rpm. erpm to rpm
		rps[i] = rpm[i] * RPM2RPS;						// Calculate velocity(rad/sec). rpm to rad/sec
		rps_lpf[i] = mat_LowpassFilter(rps[i], rps_lpf[i], 15.0);	// Calculate velocity lowpass filter
		rev[i] = Dir[i] * (float)tachometer[i] * TACHO2REV;		// value from tacho

#ifdef USE_GET_POSITION_USING_INTEGRAL
		rad[i] = mat_Integral(rps[i], rad[i]);			// Calculate position(rad)	
#else		
		rad[i] = 2.*M_PI*rev[i];
#endif
	}

	// variables
	float thddot_cp[DOF];
	float T_M_dyn[NO_OF_WHEEL];
	float T_V_dyn[NO_OF_WHEEL];
	float T_SUM[NO_OF_WHEEL];

	// Calculate Task space velocity, Current values : vx(m/s), wz(rad/s) / position:x(m), z(rad)
	current_vel_cartesian[0] = ROW / 2.*(rps[0] + rps[1]);
	current_vel_cartesian[1] = ROW / 2. / (DCW_X + DCW_Y)*(-rps[0] + rps[1]);
	pos_cartesian[0] = mat_Integral(current_vel_cartesian[0], pos_cartesian[0]);
	pos_cartesian[1] = mat_Integral(current_vel_cartesian[1], pos_cartesian[1]);

	// Calculate Task space velocity, Target values
	pos_cartesian_goal[0] = mat_Integral(target_vel_cartesian[0], pos_cartesian_goal[0]);
	pos_cartesian_goal[1] = mat_Integral(target_vel_cartesian[1], pos_cartesian_goal[1]);

	// thetaddot_cp calculation
	thddot_cp[0] = kd_task[0] * (target_vel_cartesian[0] - current_vel_cartesian[0]) + kp_task[0] * (pos_cartesian_goal[0] - pos_cartesian[0]);	// x term
	thddot_cp[1] = kd_task[1] * (target_vel_cartesian[1] - current_vel_cartesian[1]) + kp_task[1] * (pos_cartesian_goal[1] - pos_cartesian[1]);	// delta term

	// torque inertia term
	T_M_dyn[0] = M_dyn[0][0] * thddot_cp[0] + M_dyn[0][1] * thddot_cp[1];
	T_M_dyn[1] = M_dyn[1][0] * thddot_cp[0] + M_dyn[1][1] * thddot_cp[1];

	// torque feed-forward due to viscous friction - only considered viscous friction on wheel (BL*wL, BR*wR)
	T_V_dyn[0] = V_dyn[0] * rps[0];
	T_V_dyn[1] = V_dyn[1] * rps[1];

	// torque out
	T_SUM[0] = T_M_dyn[0] + T_V_dyn[0];
	T_SUM[1] = T_M_dyn[1] + T_V_dyn[1];

	// control current out
	curr_out[0] = Dir[0] * T_SUM[0] / Motor_Kt[0];
	curr_out[1] = Dir[1] * T_SUM[1] / Motor_Kt[1];

	if (ctm_cnt == FREQ) ctm_cnt = 0;

	ctm_cnt++;
}
*/

void CTM::Init()
{
	for(int i=0; i<NO_OF_WHEEL; i++) {
		rpm[i] = 0.;
		rps[i] = 0.;
		rps_lpf[i] = 0.;
		rad[i] = 0.;
		rev[i] = 0.;

		rps_goal[i] = 0.;
		rad_goal[i] = 0.;
	}

	for(int i=0; i<DOF; i++) {
		pos_cartesian[i] = 0.;
		pos_cartesian_goal[i] = 0.;
	}
}

float CTM::mat_Integral(float in, float out_last)
{
	float out;
	out = out_last + in*DT;

	return out;
}

float CTM::mat_LowpassFilter(float in, float out_last, float hz)
{
	float out_lpf;
	out_lpf = (out_last + DT*(float)2.0f*(float)M_PI*hz*in)/((float)1.0 + DT*(float)2.0f*(float)M_PI*hz);

	return out_lpf;
}

float CTM::velocity_profile_Filter(float s_goal, float s_now, float v_goal, float v_max, float a_max)
{
	float v_ref = 0;

	if (s_goal >= s_now) {
		v_ref = min_float((v_goal + a_max * DT), v_max, sqrt(2.*a_max*fabsf(s_goal - s_now)));
	}
	else {
		v_ref = max_float((v_goal - a_max * DT), -v_max, -sqrt(2.*a_max*fabsf(s_goal - s_now)));
	}

	return v_ref;
}

float CTM::max_float(float d1, float d2, float d3)
{
	float dlarger1, dlarger2;

	if (d1 > d2) dlarger1 = d1;
	else		 dlarger1 = d2;

	if (d2 > d3) dlarger2 = d2;
	else		 dlarger2 = d3;

	if (dlarger1 > dlarger2)	return dlarger1;
	else						return dlarger2;
}

float CTM::min_float(float d1, float d2, float d3)
{
	float dsmaller1, dsmaller2;

	if (d1 < d2) dsmaller1 = d1;
	else         dsmaller1 = d2;

	if (d2 < d3) dsmaller2 = d2;
    else         dsmaller2 = d3;

	if (dsmaller1 < dsmaller2)  return dsmaller1;
	else						return dsmaller2;
}
