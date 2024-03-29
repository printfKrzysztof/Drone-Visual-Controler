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
#include <fstream>
#include <time.h>
#include <chrono>
#include <sys/time.h>
#include <ctime>

using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

std::fstream plik;
/*
 * We want to know 7 states:`
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

int n = 6;						   // Number of states
int m = 3;						   // Number of measurements
double dt = 1.0000 / REFRESH_RATE; // Time step
int new_time = 0;
int old_time = 0;

Eigen::MatrixXd A(n, n); // System dynamics matrix
Eigen::MatrixXd C(m, n); // Output matrix
Eigen::MatrixXd Q(n, n); // Process noise covariance
Eigen::MatrixXd R(m, m); // Measurement noise covariance
Eigen::MatrixXd P(n, n); // Estimate error covariance

Eigen::VectorXd x_hat(n); // state vector
Eigen::VectorXd y_hat(m); // meassurements vector

KalmanFilter ekf(0, A, C, Q, R, P);
void InitMatrixes();
void UpdateTime();
Eigen::MatrixXd CalculateCfromX(Eigen::VectorXd &X);
Eigen::VectorXd GuessYfromX(Eigen::VectorXd &X);

extern uint8_t flags;
void yolo_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
	// ROS_INFO("CALLBACK");

	for (const auto &bounding_box_auto : msg->bounding_boxes)
	{
		int centre_x = (int)(bounding_box_auto.xmax + bounding_box_auto.xmin) / 2;
		int centre_y = (int)(bounding_box_auto.ymax + bounding_box_auto.ymin) / 2;
		int size = abs(bounding_box_auto.xmax - bounding_box_auto.xmin);

		y_hat(1 - 1) = centre_x - MIDDLE_X;
		y_hat(2 - 1) = MIDDLE_Y - centre_y;
		y_hat(3 - 1) = size;
	}
	if (ekf.isInit())
	{
		// ROS_INFO("UPDATE");

		x_hat = ekf.state();
		C = CalculateCfromX(x_hat);
		plik << x_hat(0) << "," << x_hat(1) << "," << x_hat(2) << "," << dt << ";\n";
		ekf.update(y_hat, GuessYfromX(x_hat), C);
		UpdateTime();
		ekf.predict(dt);
		// std::cout << "Values we recived in pixels: " << y_hat << "\n";
		// std::cout << "Values we expected to see:   " << GuessYfromX(x_hat) << "\n";
		//  ROS_INFO("PREDICT");
	}
	else
	{
		ROS_INFO("INIT");
		// Starting values
		x_hat(1 - 1) = 3;
		x_hat(2 - 1) = 0.04;
		x_hat(3 - 1) = 0.01;
		x_hat(4 - 1) = 0;
		x_hat(5 - 1) = 0;
		x_hat(6 - 1) = 0;
		// x_hat(7 - 1) = 0.4;
		C = CalculateCfromX(x_hat);
		ekf.init(x_hat, A, C, Q, R, P);

		/*
		std::cout << F << std::endl
				  << C << std::endl;
		<< Q << std::endl
		<< R << std::endl
		<< P << std::endl
		<< x_hat << std::endl
		<< y_hat << std::endl;
		*/
		UpdateTime();
		dt = 0;
		plik << x_hat(0) << "," << x_hat(1) << "," << x_hat(2) << "," << dt << ";\n";
		new_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
		ekf.predict(dt);
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
	plik.open("/home/printfkrzysztof/pose.txt", std::fstream::in | std::fstream::out | std::fstream::app);
	if (plik.good() == true)
	{
		std::cout << "Poprawnie otwarty plik";
	}
	ros::Publisher pub = filtration.advertise<dvc_msgs::StatesVector>((ros_ns + "/filtration/StatesVector").c_str(), 1);
	ros::Subscriber sub = filtration.subscribe("/darknet_ros/bounding_boxes", 1, yolo_callback);
	ros::Rate rate(REFRESH_RATE);
	while (ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
		if (ekf.isInit())
		{

			x_hat = ekf.state();

			sv.x = x_hat(1 - 1);
			sv.y = x_hat(2 - 1);
			sv.z = x_hat(3 - 1);
			sv.Vx = x_hat(4 - 1);
			sv.Vy = x_hat(5 - 1);
			sv.Vz = x_hat(6 - 1);
			// sv.r = x_hat(7 - 1);

			pub.publish(sv);
		}
	}
	plik.close();
	return 0;
}

void InitMatrixes()
{
	A.fill(0);
	A(0, 3) = 1;
	A(1, 4) = 1;
	A(2, 5) = 1;

	C.fill(0);
	// C() // ADD INITIALIZED VALUES
	Q.fill(0);
	Q(3, 3) = (double)(DELTA_V ^ 2) / ((REFRESH_RATE ^ 2));
	Q(4, 4) = (double)(DELTA_V ^ 2) / ((REFRESH_RATE ^ 2));
	Q(5, 5) = (double)(DELTA_V ^ 2) / ((REFRESH_RATE ^ 2));
	// Q(6, 6) = (double)1 / 100000;

	R.fill(0);
	R(0, 0) = (THETA_1 ^ 2);
	R(1, 1) = (THETA_2 ^ 2);
	R(2, 2) = (THETA_3 ^ 2);

	P.fill(0);
	P(1 - 1, 1 - 1) = 2;
	P(2 - 1, 2 - 1) = 2;
	P(3 - 1, 3 - 1) = 2;
	P(4 - 1, 4 - 1) = 10;
	P(5 - 1, 5 - 1) = 10;
	P(6 - 1, 6 - 1) = 10;
	// P(7 - 1, 7 - 1) = (double)1 / 20;
}

Eigen::MatrixXd CalculateCfromX(Eigen::VectorXd &X)
{
	Eigen::MatrixXd C_n(m, n);
	C_n.fill(0);

	a3dp = get_current_location();
	double x_d = a3dp.x;
	double y_d = a3dp.y;
	double z_d = a3dp.z;
	double x = X(1 - 1);
	double y = X(2 - 1);
	double z = X(3 - 1);
	// double r = X(7 - 1);
	if (z < 0)
		z = 0;
	/*
if (r < 0.1)
	r = 0.1;
if (r > 1)
	r = 1;
*/
	double r = 0.4;
#ifdef DEBUG
	std::cout << "Calculating C from: " << x_d << y_d << z_d << x << y << z << r;
#endif // DEBUG

	C_n(1 - 1, 1 - 1) = (double)-(RAD_TO_X * (y - y_d)) / ((pow(x - x_d, 2) + pow(y - y_d, 2)) + 0.001);
	C_n(1 - 1, 2 - 1) = (double)(RAD_TO_X * (x - x_d)) / ((pow(x - x_d, 2) + pow(y - y_d, 2)) + 0.001);

	C_n(2 - 1, 1 - 1) = (double)-(RAD_TO_Y * (2 * x - 2 * x_d) / (-2 * ((pow(pow(x - x_d, 2) + pow(y - y_d, 2), 0.25) + pow(z - z_d, 2)) * ((z - z_d) * pow(pow(x - x_d, 2) + pow(y - y_d, 2), 0.5)))));
	C_n(2 - 1, 2 - 1) = (double)-(RAD_TO_Y * (2 * y - 2 * y_d) / (-2 * ((pow(pow(x - x_d, 2) + pow(y - y_d, 2), 0.25) + pow(z - z_d, 2)) * ((z - z_d) * pow(pow(x - x_d, 2) + pow(y - y_d, 2), 0.5)))));
	C_n(2 - 1, 3 - 1) = (double)(RAD_TO_Y * (sqrt(pow(x - x_d, 2) + pow(y - y_d, 2)))) / ((pow(sqrt(pow(x - x_d, 2) + pow(y - y_d, 2)), 2) + pow(z - z_d, 2)));

	C_n(3 - 1, 1 - 1) = (double)-(RAD_TO_X * r * (2 * x - 2 * x_d)) / (pow(2 * (pow(x - x_d, 2) + pow(y - y_d, 2)), (3 / 2)) + 0.001);
	C_n(3 - 1, 2 - 1) = (double)-(RAD_TO_X * r * (2 * y - 2 * y_d)) / (pow(2 * (pow(x - x_d, 2) + pow(y - y_d, 2)), (3 / 2)) + 0.001);
	// C_n(3 - 1, 7 - 1) = (double)(RAD_TO_X / sqrt((pow(x - x_d, 2) + pow(y - y_d, 2))));

	if (C_n(2 - 1, 1 - 1) == INFINITY || C_n(2 - 1, 1 - 1) == -INFINITY)
	{
		C_n(2 - 1, 1 - 1) = 1;
		C_n(2 - 1, 2 - 1) = 1;
	}
#ifdef DEBUG
	std::cout << C_n << std::endl
			  << "That is calculated C";
#endif // DEBUG

	return C_n;
}

Eigen::VectorXd GuessYfromX(Eigen::VectorXd &X)
{
	Eigen::VectorXd Y(m);

	a3dp = get_current_location();
	float phi_d = get_current_heading();
	float x_d = a3dp.x;
	float y_d = a3dp.y;
	float z_d = a3dp.z;
	float x = X(1 - 1);
	float y = X(2 - 1);
	float z = X(3 - 1);
	// float r = X(7 - 1);
	double r = 0.4;
	if (z < 0)
		z = 0.1;
		/*
	if (r < 0.1)
		r = 0.1;
	if (r > 1)
		r = 1;
*/
#ifdef DEBUG
	std::cout << "Calculating Y from: " << x_d << y_d << z_d << x << y << z << r << phi_d;
#endif // DEBUG

	Y(1 - 1) = (double)RAD_TO_X * (atan2(x - x_d, y - y_d) - DEG_TO_RAD * phi_d);
	Y(2 - 1) = (double)RAD_TO_Y * atan2(z_d - z, sqrt(pow(x - x_d, 2) + pow(y - y_d, 2)));
	Y(3 - 1) = (double)(RAD_TO_X * r) / (sqrt(pow(x - x_d, 2) + pow(y - y_d, 2)) + 0.001);

#ifdef DEBUG
	std::cout << Y << std::endl
			  << "That was calculated Y";
#endif // DEBUG

	return Y;
}

void UpdateTime()
{
	old_time = new_time;
	new_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
	dt = (double)(new_time - old_time) / 1000;
	if (dt == 0)
		dt = 0.00001;
}