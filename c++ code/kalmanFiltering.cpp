#include <MatrixMath.h>

void KalmanFilterUpdate(float* control_cov, float* measure_cov, float* last_pose, float* last_pose_cov, float* measurement, float* control_input)
{
	/*************************************************************************************
	==== Read measurement data ====
	*************************************************************************************/

	// float last_pose[3] = pose;

	/*************************************************************************************
	==== Read Control Input ====
	*************************************************************************************/
	// control_input = ??;

	/*************************************************************************************
	Prediction Step
	*************************************************************************************/
	float motion_left = control_input[0]*2*M_PI*6/2*0.0254;
	float motion_right = control_input[1]*2*M_PI*6/2*0.0254;

	float relative_displacement = (motion_right - motion_left);
	float squareRootTerm = pow(track_width,2) - pow((motion_right - motion_left),2);
	float B[3][2] = {{cos(last_pose[2])/2, cos(last_pose[2])/2}, {sin(last_pose[2])/2, sin(last_pose[2])/2}, {-1/sq(squareRootTerm), 1/sq(squareRootTerm)}};

	float pose_pre[3] = {last_pose[0] + (motion_left + motion_right)*cos(last_pose[2])/2, last_pose[1] + (motion_left + motion_right)*sin(last_pose[2])/2, asin(relative_displacement)};

	float temp[3][2];

	float B_transpose[2][3];
	Matrix.Transpose((float*)B, 2, 3, (float*)B_transpose);

	float pose_cov_from_control[3][3];

	Matrix.Multiply((float*)B, (float*)control_cov, 3, 2, 2, (float*)temp);
	Matrix.Multiply((float*)temp, (float*)B_transpose, 3, 2, 3, (float*)pose_cov_from_control);


	float pose_cov_pre[3][3];
	Matrix.Add((float*)last_pose_cov, (float*)pose_cov_from_control, 3, 3, (float*)pose_cov_pre);

	/*************************************************************************************
	Correction Step
	*************************************************************************************/
	float distance_left = measurement[0]/100;
	float distance_right = measurement[1]/100;
	float yaw_angle = measurement[4];
	yaw_angle = yaw_angle % 360;
	if(yaw_angle >=180)
		yaw_angle = (yaw_angle - 360)*M_PI/180;
	else
		yaw_angle = (yaw_angle)*M_PI/180;	

	float Z[3] = {pose_pre[1] + distance_left*cos(pose_pre[2]) + track_width/2*cos(pose_pre[2]), pose_pre[1] - distance_right*cos(pose_pre[2]) - track_width/2*cos(pose_pre[2]), pose_pre[2] - yaw_angle};
	float Z_obs[3] = {0.8, 0, 0};

	float C[3][3] = {{0, 1, -(distance_left + track_width/2)*sin(pose_pre[2])}, {0, 1, (distance_right + track_width/2)*sin(pose_pre[2])}, {0, 0, }};
	float D[3][5] = {{cos(pose_pre[2]), 0, 0, 0, 0,}, {0, -cos(pose_pre[2]), 0, 0, 0}, {0, 0, 0, 0, -1}};

	Matrix.Multiply((float*)temp, (float*)B_transpose, 3, 2, 3, (float*)pose_cov_from_control);

	/*************************************************************************************
	Kalman Update
	K_matrix = P_pre * C' / (C * P_pre * C' + D*measure_cov*D');
	*************************************************************************************/

	float K_matrix[3][3];
	KalmanFilterConstant(3, 3, 5, C, D, (float*)pose_cov_pre, (float*)measure_cov, (float*)K_matrix)

	/*************************************************************************************
	Update Pose
	x = x_pre + K*(Z_obs - Z);
	*************************************************************************************/
	float pose_update[3];
	float new_pose[3];
	float error[3];

	Matrix.Subtract((float*)Z_obs, (float*)Z, 3, 1, (float*)error);
	Matrix.Multiply((float*)K_matrix, (float*)error, 3, 3, 1, (float*)pose_update);
	Matrix.Add((float*)pose_pre, (float*)pose_update, 3, 1, (float*)new_pose);
	Matrix.Copy((float*)new_pose, 3, 1, (float*)last_pose);

	/*************************************************************************************
	Update Pose_Covariance
	P = (eye(3) - K*C)*P_pre;
	*************************************************************************************/
	float identity[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

	float K_multiply_C[3][3];
	Matrix.Multiply((float*)K_matrix, (float*)C, 3, 3, 3, (float*)K_multiply_C);

	float multiplier[3][3];
	Matrix.Subtract((float*)identity, (float*)K_multiply_C, 3, 3, (float*)multiplier);

	float new_pose_cov[3][3];
	Matrix.Multiply((float*)multiplier, (float*)pose_cov_pre, 3, 3, 3, (float*)new_pose_cov);
	Matrix.Copy((float*)new_pose_cov, 3, 3, (float*)last_pose_cov);

}

void KalmanFilterInitialization(float* control_cov, float* measure_cov, float* last_pose, float* last_pose_cov)
{
	/*************************************************************************************
	==== Define sigma ===
	*************************************************************************************/
	float sig_ultrasonic = 0.08;
	float sig_imu = 2.5*M_PI/180;
	float sig_enc = 0.005;

	/*************************************************************************************
	==== Generate sigma^2 from sigma ===
	*************************************************************************************/
	sig_ultrasonic = pow(sig_ultrasonic, 2);
	sig_imu = pow(sig_imu, 2);
	sig_enc = pow(sig_enc, 2);

	/*************************************************************************************
	==== dimension of vehicle ===
	*************************************************************************************/
	float wheel_base = 10*0.0254;
	float track_width = 10.5*0.0254; 

	/*************************************************************************************
	==== Initialization ====
	*************************************************************************************/
	control_cov[2][2] = {{sig_enc, 0}, {0, sig_enc}};
	measure_cov[5][5] = {{sig_ultrasonic, 0, 0, 0, 0}, {0, sig_ultrasonic, 0, 0, 0}, {0, 0, sig_ultrasonic, 0, 0}, {0, 0, 0, sig_ultrasonic, 0}, {0, 0, 0, 0, sig_imu}};

	last_pose[3] = {0 , 0.4 , 0};
	last_pose_cov = {{pow(0.0, 2), 0, 0}, {0, pow(0.02,2), 0}, {0, 0, pow(0.1,2)}};
}

void KalmanFilterConstant(int a, int b, int c, float* C, float* D, float* pose_cov_pre, float* measure_cov, float* K_matrix)
{
	// C is a X b
	// D is a X c
	// pose_cov_pre is b X b
	// measure_cov is c X c

	// Using K_matrix = P_pre * C' / (C * P_pre * C' + D*measure_cov*D');
	// or, K_matrix = P_pre*C' * inverseTermEKF
	// where inverseTermEKF = inverse(firstTerm + secondTerm);
	// firstTerm = C * P_pre * C';
	// secondTerm = D*measure_cov*D';

	float C_transpose[b][a];
	Matrix.Transpose((float*)C, a, b, (float*)C_transpose);

	float D_transpose[c][a];
	Matrix.Transpose((float*)D, a, c, (float*)D_transpose);

	float C_multiply_P_cov_pre[a][b];
	float firstTerm[a][a];

	Matrix.Multiply((float*)C, (float*)pose_cov_pre, a, b, b, (float*)C_multiply_P_cov_pre); // a X b
	Matrix.Multiply((float*)C_multiply_P_cov_pre, (float*)C_transpose, a, b, a, (float*)firstTerm); // a X a

	float D_multiply_measure_cov[a][c];
	float secondTerm[a][a];

	Matrix.Multiply((float*)D, (float*)measure_cov, a, c, c, (float*)D_multiply_measure_cov); // a X c
	Matrix.Multiply((float*)D_multiply_measure_cov, (float*)D_transpose, a, c, a, (float*)firstTerm); // a X a

	float inverseTermEKF[a][a];
	Matrix.Add((float*)firstTerm, (float*)secondTerm, a, a, (float*)inverseTermEKF);
	Matrix.Invert((float*)inverseTermEKF, a);

	float K_matrix[b][a];
	float P_cov_pre_multiply_C_transpose[b][a];
	Matrix.Multiply((float*)pose_cov_pre, (float*)C_transpose, b, b, a, (float*)P_cov_pre_multiply_C_transpose); // b X a
	Matrix.Multiply((float*)P_cov_pre_multiply_C_transpose, (float*)inverseTermEKF, b, a, a, (float*)K_matrix); // b X a
}
