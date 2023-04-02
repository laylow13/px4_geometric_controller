#include "controller.hpp"
#include <cstdio>
using std::cout;
using std::endl;

double Controller::get_yaw_from_quaternion(const Eigen::Quaterniond &q)
{
	Eigen::Vector3d yaw = q.matrix().eulerAngles(0, 1, 2);
	// std::printf("euler is %lf %lf %lf", yaw(0), yaw(1), yaw(2));
	return yaw(2);
}

Eigen::Matrix3d Controller::rotz(double t)
{
	Eigen::Matrix3d R;
	R(0, 0) = std::cos(t);
	R(0, 1) = -std::sin(t);
	R(0, 2) = 0.0;
	R(1, 0) = std::sin(t);
	R(1, 1) = std::cos(t);
	R(1, 2) = 0.0;
	R(2, 0) = 0.0;
	R(2, 1) = 0.0;
	R(2, 2) = 1.0;

	return R;
}

Eigen::Quaterniond Controller::computeDesiredAttitude(const Eigen::Vector3d &desired_acceleration, const double reference_heading, const Eigen::Quaterniond &attitude_estimate) const
{
	// desired_acceleration means the desired thrust and is perpendicular to the body frame.

	const Eigen::Quaterniond q_heading = Eigen::Quaterniond(Eigen::AngleAxisd(reference_heading, Eigen::Vector3d::UnitZ()));
	// Compute desired orientation
	const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
	const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();
	Eigen::Vector3d z_B;
	if (almostZero(desired_acceleration.norm()))
	{
		// In case of free fall we keep the thrust direction to be the estimated one
		// This only works assuming that we are in this condition for a very short
		// time (otherwise attitude drifts)
		z_B = attitude_estimate * Eigen::Vector3d::UnitZ();
	}
	else
	{
		z_B = desired_acceleration.normalized();
	}

	const Eigen::Vector3d x_B_prototype = y_C.cross(z_B);
	const Eigen::Vector3d x_B = computeRobustBodyXAxis(x_B_prototype, x_C, y_C, attitude_estimate);
	const Eigen::Vector3d y_B = (z_B.cross(x_B)).normalized();
	// From the computed desired body axes we can now compose a desired attitude
	const Eigen::Matrix3d R_W_B((Eigen::Matrix3d() << x_B, y_B, z_B).finished());
	// ROS_INFO_STREAM("R_W_B: "<<R_W_B);
	const Eigen::Quaterniond desired_attitude(R_W_B);

	return desired_attitude;
}

bool Controller::almostZero(const double value) const
{
	return fabs(value) < 0.001;
}

bool Controller::almostZeroThrust(const double thrust_value) const
{
	return fabs(thrust_value) < 0.01;
}

Eigen::Vector3d Controller::computeRobustBodyXAxis(const Eigen::Vector3d &x_B_prototype, const Eigen::Vector3d &x_C, const Eigen::Vector3d &y_C, const Eigen::Quaterniond &attitude_estimate) const
{
	Eigen::Vector3d x_B = x_B_prototype;

	if (almostZero(x_B.norm()))
	{
		// if cross(y_C, z_B) == 0, they are collinear =>
		// every x_B lies automatically in the x_C - z_C plane

		// Project estimated body x-axis into the x_C - z_C plane
		const Eigen::Vector3d x_B_estimated =
			attitude_estimate * Eigen::Vector3d::UnitX();
		const Eigen::Vector3d x_B_projected =
			x_B_estimated - (x_B_estimated.dot(y_C)) * y_C;
		if (almostZero(x_B_projected.norm()))
		{
			// Not too much intelligent stuff we can do in this case but it should
			// basically never occur
			x_B = x_C;
		}
		else
		{
			x_B = x_B_projected.normalized();
		}
	}
	else
	{
		x_B.normalize();
	}

	return x_B;
}

Eigen::Vector3d integration_limit(Eigen::Vector3d input, double limit)
{
	for (int i = 0; i < 3; i++)
	{
		if (input(i) > limit)
		{
			input(i) = limit;
		}
		else if (input(i) < -limit)
		{
			input(i) = -limit;
		}
	}
	return Eigen::Vector3d(input(0), input(1), input(2));
}

void Controller::run(UAV_motion_t &motion_input, UAV_motion_t &motion_fb, attitude_sp_t &u)
{

	Eigen::Vector3d err_pos = motion_input.linear.pos - motion_fb.linear.pos;
	Eigen::Vector3d err_vel = motion_input.linear.vel - motion_fb.linear.vel;
	Eigen::Vector3d err_acc = motion_input.linear.acc; //- motion_fb.angular.q * motion_fb.linear.acc;

	err_vel_i += integration_limit(err_pos, param.i_speed_max / param.ctrl_rate);
	err_vel_i = integration_limit(err_vel_i, param.i_limit_max);

	// std::printf("ERR_X ERR_Y ERR_Z is %lf %lf %lf ", err_pos(0), err_pos(1), err_pos(2));

	// reserved for observer to anti_disturbance
	// Eigen::Vector3d drag_accelerations = Eigen::Vector3d::Zero();
	// std::printf("expect vel is %lf %lf %lf ", motion_input.linear.vel(0), motion_input.linear.vel(1), motion_input.linear.vel(2));
	double yaw_curr = get_yaw_from_quaternion(motion_fb.angular.q);

	// double yaw_des = get_yaw_from_quaternion(motion_input.angular.q);
	if (abs(motion_input.linear.vel(1)) < 0.1 && abs(motion_input.linear.vel(0)) < 0.1)
	{
		if (!no_yaw_flag)
		{
			yaw_des = yaw_curr;
		}
		no_yaw_flag = 1;
	}
	else
	{
		no_yaw_flag = 0;
		yaw_des = atan2(motion_input.linear.vel(1), motion_input.linear.vel(0));
	}

	yaw_des = 0;
	// std::printf("desired yaw is %lf", yaw_des);
	Eigen::Matrix3d rot_yaw = rotz(yaw_curr);
	Eigen::Matrix3d rot_yaw_trans = rot_yaw.transpose();

	/*const std::vector<double> integration_enable_limits = {0.1, 0.1, 0.1};
	for (size_t k = 0; k < 3; ++k)
	{
		if (std::fabs(err_vel(k)) < 0.2)
		{
			int_e_v(k) += err_vel(k) * 1.0 / 50.0;
		}
	}
*/
	const std::vector<double> integration_output_limits = {0.4, 0.4, 0.4};

	Eigen::Vector3d err_pos_to_vel = rot_yaw * param.Kp * rot_yaw_trans * err_pos;
	Eigen::Vector3d err_vel_i_to_f = rot_yaw * param.Kvi * rot_yaw_trans * err_vel_i;
	Eigen::Vector3d err_vel_to_f = rot_yaw * param.Kv * rot_yaw_trans * err_vel + err_pos_to_vel;

	Eigen::Vector3d F_des = rot_yaw * param.Ka * rot_yaw_trans * err_acc * param.mass + err_vel_to_f * param.mass + err_vel_i_to_f * param.mass + Eigen::Vector3d(0, 0, param.mass * param.gra);

	Eigen::Matrix3d wRb_odom = motion_fb.angular.q.toRotationMatrix();
	Eigen::Vector3d z_b_curr = wRb_odom.col(2);

	double u_true = F_des.dot(z_b_curr);
	double full_thrust = param.mass * param.gra / param.hov_percent;
	// TO DO:消息类型发生改变，原代码需要更改
	u.thrust = u_true / full_thrust;
	if (u.thrust>0.9)
	{
		RCLCPP_WARN(rclcpp::get_logger("controller"),"Thrust too high:%f",u.thrust);
		u.thrust=0.9;
	}
	// ROS_INFO("thrust_raw = %lf",u_true);
	/*
		if (u.thrust >= 0.9)
			ROS_WARN("FULL THRUST");
			u.thrust=0.9;
		if (u.thrust<=0)
		{
			u.thrust = 0;
		}
		//ROS_INFO("thrust = %lf",u_true);
		//ROS_INFO("F_des = %lf",uuu);
		//u.thrust = u.thrust >= 0.9 ? 0.9 : u.thrust;

	*/
	const Eigen::Quaterniond desired_attitude = computeDesiredAttitude(F_des / param.mass, yaw_des, motion_input.angular.q);

	u.q[1] = desired_attitude.x();
	u.q[2] = desired_attitude.y();
	u.q[3] = desired_attitude.z();
	u.q[0] = desired_attitude.w();
	
}
