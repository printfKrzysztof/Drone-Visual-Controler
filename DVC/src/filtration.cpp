/**
 * @file filtration.cpp
 * @author Krzysztof B (github)
 * @brief
 * @version 0.1
 * @date 2022-10-04
 *
 * @cite IQ_SIM
 *
 */

#include <main.hpp>
#include <cmath>
#include "config.h"
#include "kalman.hpp"
#include "gnc_functions.hpp"
#include <vector>
#include <iostream>

/*
 * We want to know 7 states:
 * Position in x    x
 * Position in y    y
 * Position in z    z
 * Velocity in x    Vx
 * Velocity in y    Vy
 * Velocity in z    Vz
 * Size of drone    r
 *
 * X=[x, y, z, Vx, Vy, Vz, r]
 *
 * We can measure 3 values
 *
 * Angle between us and robots xy plane     alphaxy
 * Angle between us and robots z plane      alphaz
 * Angular size of a drone (width)          deltaalpha
 *
 * Y = [axy, az, da]
 *
 * Estimate function:
 * Y = h(X, T)
 *
 * 1. f1: axy =
 */

geometry_msgs::Point a3dp; // point gettind data from drone

int n = 7;						// Number of states
int m = 3;						// Number of measurements
double dt = 1.0 / REFRESH_RATE; // Time step

Eigen::MatrixXd F(n, n); // System dynamics matrix
Eigen::MatrixXd C(m, n); // Output matrix
Eigen::MatrixXd Q(n, n); // Process noise covariance
Eigen::MatrixXd R(m, m); // Measurement noise covariance
Eigen::MatrixXd P(n, n); // Estimate error covariance

Eigen::VectorXd x_hat(n); // state vector
Eigen::VectorXd y_hat(m); // meassurements vector

KalmanFilter ekf(0, F, C, Q, R, P);
void InitMatrixes();
Eigen::MatrixXd CalculateCfromX(Eigen::VectorXd X);
Eigen::VectorXd GuessYfromX(Eigen::VectorXd X);

extern uint8_t flags;
void yolo_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
	// ROS_INFO("CALLBACK");

	for (const auto &bounding_box_auto : msg->bounding_boxes)
	{
		int centre_x = (int)(bounding_box_auto.xmax + bounding_box_auto.xmin) / 2;
		int centre_y = (int)(bounding_box_auto.ymax + bounding_box_auto.ymin) / 2;
		// 271 -> 0,5 pi rad (90 deg)

		int size = abs(bounding_box_auto.xmax - bounding_box_auto.xmin);
		y_hat(1) = centre_x;
		y_hat(2) = centre_y;
		y_hat(3) = size;
	}
	if (ekf.isInit())
	{
		ROS_INFO("UPDATE");
		C = CalculateCfromX(ekf.state());
		ekf.update(y_hat, GuessYfromX(ekf.state()), dt, C);
	}
	else
	{
		ROS_INFO("INIT");
		// Starting values
		x_hat(1) = 3;
		x_hat(2) = 0.04;
		x_hat(3) = 0.01;
		x_hat(4) = 0;
		x_hat(5) = 0;
		x_hat(6) = 0;
		x_hat(7) = 0.4;
		C = CalculateCfromX(x_hat);
		ekf.init(x_hat, F, C, Q, R, P);
	}
}

int main(int argc, char **argv)
{
	dvc_msgs::StatesVector sv;
	ros::init(argc, argv, "filtration");
	ros::NodeHandle filtration("~");

	init_subscriber_only(filtration);

	std::string ros_ns;
	if (!filtration.hasParam("namespace"))
	{
		ros_ns = "";
	}
	else
	{
		filtration.getParam("namespace", ros_ns);
	}
	InitMatrixes();
	ros::Publisher pub = filtration.advertise<dvc_msgs::StatesVector>((ros_ns + "/filtration/StatesVector").c_str(), 1);
	ros::Subscriber sub = filtration.subscribe("/darknet_ros/bounding_boxes", 1, yolo_callback);
	ros::Rate rate(REFRESH_RATE);
	while (ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
		// ROS_INFO("WE ARE IN MAIN: %f", dt);
		if (ekf.isInit())
		{
			ROS_INFO("PREDICT");
			ekf.predict();
			dt++;
			x_hat = ekf.state();

			sv.x = x_hat(1);
			sv.y = x_hat(2);
			sv.z = x_hat(3);
			sv.Vx = x_hat(4);
			sv.Vy = x_hat(5);
			sv.Vz = x_hat(6);
			sv.r = x_hat(7);

			pub.publish(sv);
		}
	}
	return 0;
}

void InitMatrixes()
{
	F.fill(0);
	F(4, 1) = 1;
	F(5, 2) = 1;
	F(6, 3) = 1;
	F(7, 7) = 1;
	Eigen::MatrixXd I(n, n); // Identity
	F = I.Identity(n, n) * F * dt;

	C.fill(0);
	// C() // ADD INITIALIZED VALUES
	Q.fill(0);
	Q(4, 4) = (DELTA_V ^ 2) / ((REFRESH_RATE ^ 2));
	Q(5, 5) = (DELTA_V ^ 2) / ((REFRESH_RATE ^ 2));
	Q(6, 6) = (DELTA_V ^ 2) / ((REFRESH_RATE ^ 2));
	Q(7, 7) = 1 / 100000;

	R.fill(0);
	R(1, 1) = (THETA_1 ^ 2);
	R(2, 2) = (THETA_2 ^ 2);
	R(3, 3) = (THETA_3 ^ 2);

	P.fill(0);
	P(1, 1) = 25;
	P(2, 2) = 25;
	P(3, 3) = 25;
	P(4, 4) = 100;
	P(5, 5) = 100;
	P(6, 6) = 100;
	P(7, 7) = 1 / 20;
}

Eigen::MatrixXd CalculateCfromX(Eigen::VectorXd X)
{
	Eigen::MatrixXd C_n(m, n);
	C_n.fill(0);

	a3dp = get_current_location();
	float x_d = a3dp.x;
	float y_d = a3dp.y;
	float z_d = a3dp.z;
	float x = X(1);
	float y = X(2);
	float z = X(3);
	float r = X(7);

	C(1, 1) = (RAD_TO_X * (y - y_d)) / (pow(x - x_d, 2) + pow(y - y_d, 2));
	C(1, 2) = -(RAD_TO_X * (x - x_d)) / (pow(x - x_d, 2) + pow(y - y_d, 2));
	C(2, 1) = (RAD_TO_Y * (2 * x - 2 * x_d) * sqrt(pow(x - x_d, 2) + pow(y - y_d, 2)) * (z - z_d)) / ((2 * pow(x - x_d, 2) + 2 * pow(y - y_d, 2)) * (pow(x - x_d, 2) + pow(y - y_d, 2) + pow(z - z_d, 2)));
	C(2, 2) = (RAD_TO_Y * (2 * y - 2 * y_d) * sqrt(pow(x - x_d, 2) + pow(y - y_d, 2)) * (z - z_d)) / ((2 * pow(x - x_d, 2) + 2 * pow(y - y_d, 2)) * (pow(x - x_d, 2) + pow(y - y_d, 2) + pow(z - z_d, 2)));
	C(2, 3) = -(RAD_TO_Y * sqrt(pow(x - x_d, 2) + pow(y - y_d, 2))) / (pow(x - x_d, 2) + pow(y - y_d, 2) + pow(z - z_d, 2));
	C(3, 1) = -(RAD_TO_X * r * (2 * x - 2 * x_d)) / pow(2 * (pow(x - x_d, 2) + pow(y - y_d, 2)), (3 / 2));
	C(3, 2) = -(RAD_TO_X * r * (2 * y - 2 * y_d)) / pow(2 * (pow(x - x_d, 2) + pow(y - y_d, 2)), (3 / 2));
	C(3, 7) = RAD_TO_X / pow((pow(x - x_d, 2) + pow(y - y_d, 2)), (1 / 2));

	ROS_INFO("C[%f, %f, %f, %f]", C(1, 1), C(2, 1), C(1, 2), C(1, 3));

	return C_n;
}

Eigen::VectorXd GuessYfromX(Eigen::VectorXd X)
{
	Eigen::VectorXd Y(m);

	a3dp = get_current_location();
	float phi_d = get_current_heading();
	float x_d = a3dp.x;
	float y_d = a3dp.y;
	float z_d = a3dp.z;
	float x = X(1);
	float y = X(2);
	float z = X(3);
	float r = X(7);

	Y(1) = 0;
	Y(2) = 0;
	Y(3) = 0;
	
	return Y;
}