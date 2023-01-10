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

KalmanFilter ekf;
geometry_msgs::Point a3dp;

int n = 7;						// Number of states
int m = 3;						// Number of measurements
double dt = 1.0 / REFRESH_RATE; // Time step

Eigen::MatrixXd A(n, n); // System dynamics matrix
Eigen::MatrixXd C(m, n); // Output matrix
Eigen::MatrixXd Q(n, n); // Process noise covariance
Eigen::MatrixXd R(m, m); // Measurement noise covariance
Eigen::MatrixXd P(n, n); // Estimate error covariance

Eigen::VectorXd x(n, 1);
Eigen::VectorXd z(1, m);
void InitMatrixes();
Eigen::MatrixXd CalculateCfromX(Eigen::VectorXd X);
//  {
//  	Eigen::MatrixXd C(m, n);
//  	// using vector X calculate all C values and return it;
//  	return C;
//  }
extern uint8_t flags;
void yolo_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
	for (const auto &bounding_box_auto : msg->bounding_boxes)
	{
		int centre_x = (int)(bounding_box_auto.xmax + bounding_box_auto.xmin) / 2;
		int centre_y = (int)(bounding_box_auto.ymax + bounding_box_auto.ymin) / 2;
		// 271 -> 0,5 pi rad (90 deg)

		int size = abs(bounding_box_auto.xmax - bounding_box_auto.xmin);
	}
	if (ekf.isInit())
	{
		ekf.update(z, dt, A, C);
	}
	else
	{
		ekf.init();
	}
}

int main(int argc, char **argv)
{
	dvc_msgs::StatesVector sv;
	ros::init(argc, argv, "filtration");
	ros::NodeHandle filtration("~");
	std::string ros_ns;
	if (!filtration.hasParam("namespace"))
	{
		ros_ns = "";
	}
	else
	{
		filtration.getParam("namespace", ros_ns);
	}
	ros::Publisher pub = filtration.advertise<dvc_msgs::StatesVector>((ros_ns + "/filtration/search_results").c_str(), 1);
	ros::Subscriber sub = filtration.subscribe("/darknet_ros/bounding_boxes", 1, &yolo_callback);
	ros::Rate rate(4.0);
	InitMatrixes();
	while (ros::ok())
	{
		if (ekf.isInit())
		{
			ekf.predict();
			pub.publish(sv);
		}
	}
}

void InitMatrixes()
{
	A.fill(0);
	A(4, 1) = 1;
	A(5, 2) = 1;
	A(6, 3) = 1;
	A(7, 7) = 1;

	C.fill(0);

	Q.fill(0);
	Q(4, 4) = (DELTA_V ^ 2) / (2 * (REFRESH_RATE ^ 2));
	Q(5, 5) = (DELTA_V ^ 2) / (2 * (REFRESH_RATE ^ 2));
	Q(6, 6) = (DELTA_V ^ 2) / (2 * (REFRESH_RATE ^ 2));
	Q(7, 7) = 1 / 100000;

	R.fill(0);
	R(1, 1) = (THETA_1 ^ 2) / 2;
	R(2, 2) = (THETA_2 ^ 2) / 2;
	R(3, 3) = (THETA_3 ^ 2) / 2;

	P.fill(0);
	P(1, 1) = 12.5;
	P(2, 2) = 12.5;
	P(3, 3) = 12.5;
	P(4, 4) = 50;
	P(5, 5) = 50;
	P(6, 6) = 50;
	P(7, 7) = 1 / 20;
}

Eigen::MatrixXd CalculateCfromX(Eigen::VectorXd X)
{
	Eigen::MatrixXd C_n(m, n);
	C_n.fill(0);
	a3dp = get_current_location(); // drones
	float x_d = a3dp.x;
	float y_d = a3dp.y;
	float z_d = a3dp.z;
	float x_frommatrix = X(1, 1);
	float y_frommatrix = X(1, 2);
	float z_frommatrix = X(1, 3);
	float r = X(1, 7);
	C(1, 1) = ((2 * x_frommatrix - 2 * x_d) * (y_frommatrix - y_d)) / (2 * pow((pow((x_frommatrix - x_d), 2) + pow((y_frommatrix - y_d), 2)), (3 / 2)) * pow((1 - pow((y_frommatrix - y_d), 2) / (pow((x_frommatrix - x_d), 2.0) + pow((y_frommatrix - y_d), 2))), (1 / 2)));
	C(2, 1) = ((y_frommatrix - y_d) * (2 * y_frommatrix - 2 * y_d) / (2 * pow(pow(x_frommatrix - x_d, 2) + pow(y_frommatrix - y_d, 2), 1.5)) - pow(pow(x_frommatrix - x_d, 2) + pow(y_frommatrix - y_d, 2), -0.5)) / pow(1 - pow(y_frommatrix - y_d, 2) / (pow(x_frommatrix - x_d, 2) + pow(y_frommatrix - y_d, 2)), 0.5);

	C(1, 2) = (z_frommatrix - z_d) * (2 * x_frommatrix - 2 * x_d) / (2 * ((pow(z_frommatrix - z_d, 2) / (pow(x_frommatrix - x_d, 2) + pow(y_frommatrix - y_d, 2)) + 1) * pow(pow(x_frommatrix - x_d, 2) + pow(y_frommatrix - y_d, 2), 1.5)));
	C(2, 2) = (z_frommatrix - z_d) * (2 * y_frommatrix - 2 * y_d) / (2 * ((pow(z_frommatrix - z_d, 2) / (pow(x_frommatrix - x_d, 2) + pow(y_frommatrix - y_d, 2)) + 1) * pow(pow(x_frommatrix - x_d, 2) + pow(y_frommatrix - y_d, 2), (1.5))));
	C(3, 2) = -1 / ((pow(z_frommatrix - z_d, 2) / (pow(x_frommatrix - x_d, 2) + pow(y_frommatrix - y_d, 2)) + 1) * sqrt(pow(x_frommatrix - x_d, 2) + pow(y_frommatrix - y_d, 2)));
	C(1, 3) = (-((((y_frommatrix) - (y_d)) / ((pow((y_frommatrix) - (y_d), 2) / pow((r) + (x_frommatrix) - (x_d), 2) + 1) * pow((r) + (x_frommatrix) - (x_d), 2)) + ((y_d) - (y_frommatrix)) / ((pow((y_frommatrix) - (y_d), 2) / pow((r) - (x_frommatrix) + (x_d), 2) + 1) * pow((r) - (x_frommatrix) + (x_d), 2))) * (atan((y_frommatrix - y_d) / (r + x_frommatrix - x_d)) + atan((y_frommatrix - y_d) / (r - x_frommatrix + x_d)))) - ((y_frommatrix - y_d) / ((pow(y_frommatrix - y_d, 2) / pow(r + x_frommatrix - x_d, 2) + 1) * pow(r + x_frommatrix - x_d, 2)) + (y_d - y_frommatrix) / ((pow(y_frommatrix - y_d, 2) / pow(r - x_frommatrix + x_d, 2) + 1) * pow(r - x_frommatrix + x_d, 2))) * ((atan((y_frommatrix - y_d) / (r + x_frommatrix - x_d))) + (atan((y_frommatrix - y_d) / (r - x_frommatrix + x_d))))) / (2 * sqrt((atan((y_frommatrix - y_d) / (r + x_frommatrix - x_d)) + atan((y_frommatrix - y_d) / (r - x_frommatrix + x_d))) * ((atan((y_frommatrix - y_d) / (r + x_frommatrix - x_d))) + (atan((y_frommatrix - y_d) / (r - x_frommatrix + x_d))))));
	C(2, 3) = ((1 / ((pow((y_frommatrix) - (y_d), 2) / pow((r) + (x_frommatrix) - (x_d), 2) + 1) * ((r) + (x_frommatrix) - (x_d))) + 1 / (((r) - (x_frommatrix) + (x_d)) * (pow((y_frommatrix) - (y_d), 2) / pow((r) - (x_frommatrix) + (x_d), 2) + 1))) * (atan((y_frommatrix - y_d) / (r + x_frommatrix - x_d)) + atan((y_frommatrix - y_d) / (r - x_frommatrix + x_d))) + (1 / ((pow(y_frommatrix - y_d, 2) / pow(r + x_frommatrix - x_d, 2) + 1) * (r + x_frommatrix - x_d)) + 1 / ((r - x_frommatrix + x_d) * (pow(y_frommatrix - y_d, 2) / pow(r - x_frommatrix + x_d, 2) + 1))) * ((atan((y_frommatrix - y_d) / (r + x_frommatrix - x_d))) + (atan((y_frommatrix - y_d) / (r - x_frommatrix + x_d))))) / (2 * sqrt((atan((y_frommatrix - y_d) / (r + x_frommatrix - x_d)) + atan((y_frommatrix - y_d) / (r - x_frommatrix + x_d))) * ((atan((y_frommatrix - y_d) / (r + x_frommatrix - x_d))) + (atan((y_frommatrix - y_d) / (r - x_frommatrix + x_d))))));
	C(7, 3) = (-((((y_frommatrix) - (y_d)) / ((pow((y_frommatrix) - (y_d), 2) / pow((r) + (x_frommatrix) - (x_d), 2) + 1) * pow((r) + (x_frommatrix) - (x_d), 2)) + ((y_frommatrix) - (y_d)) / ((pow((y_frommatrix) - (y_d), 2) / pow((r) - (x_frommatrix) + (x_d), 2) + 1) * pow((r) - (x_frommatrix) + (x_d), 2))) * (atan((y_frommatrix - y_d) / (r + x_frommatrix - x_d)) + atan((y_frommatrix - y_d) / (r - x_frommatrix + x_d)))) - ((y_frommatrix - y_d) / ((pow(y_frommatrix - y_d, 2) / pow(r + x_frommatrix - x_d, 2) + 1) * pow(r + x_frommatrix - x_d, 2)) + (y_frommatrix - y_d) / ((pow(y_frommatrix - y_d, 2) / pow(r - x_frommatrix + x_d, 2) + 1) * pow(r - x_frommatrix + x_d, 2))) * ((atan((y_frommatrix - y_d) / (r + x_frommatrix - x_d))) + (atan((y_frommatrix - y_d) / (r - x_frommatrix + x_d))))) / (2 * sqrt((atan((y_frommatrix - y_d) / (r + x_frommatrix - x_d)) + atan((y_frommatrix - y_d) / (r - x_frommatrix + x_d))) * ((atan((y_frommatrix - y_d) / (r + x_frommatrix - x_d))) + (atan((y_frommatrix - y_d) / (r - x_frommatrix + x_d)))))); // using vector X calculate all C values and return it;
	return C_n;
}