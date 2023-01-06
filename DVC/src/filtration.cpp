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
#include "config.h"
#include "kalman_filter.h"

// What Kalman needs
KalmanFilter ekf;

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

int n = 7; // Number of states
int m = 3; // Number of measurements

double dt = 1.0 / 30; // Time step

// state vector (poczatkowy wektor stanu)
Eigen::VectorXd x_in(n, 1);

// state transition matrix (poczatkowa macierz stanu)
Eigen::MatrixXd F_in(n, n);

// measurement matrix  (macierz C)
Eigen::MatrixXd C_in(m, n);

/* --- COVARIANCE MATRIXES --- */
// process covariance matrix (poczatkowa kowariancja szumu procesu)
Eigen::MatrixXd Q_in(n, n);

// measurement covariance matrix (poczatkowa kowariancja szumu pomiaru)
Eigen::MatrixXd R_in(m, m);

// state covariance matrix (poczatkowa kowariancja stanu)
Eigen::MatrixXd P_in(n, n);

Eigen::MatrixXd CalculateCfromX(Eigen::VectorXd X(n, 1))
{
	Eigen::MatrixXd C(m, n);
	// using vector X calculate all C values and return it;
	return C;
}
extern uint8_t flags;

class DataResender
{

private:
	dvc_msgs::SearchResult SingleObject;
	ros::Publisher pub;
	ros::Subscriber yolo_sub;
	dvc_msgs::SearchResult Last[10] = {};

public:
	/**
	 * @brief Construct a new Data Resender object
	 *
	 * @param nh NodeHandler
	 */
	DataResender(ros::NodeHandle *nh)
	{
		std::string ros_ns;
		if (!nh->hasParam("namespace"))
		{
			ros_ns = "";
		}
		else
		{
			nh->getParam("namespace", ros_ns);
		}
		pub = nh->advertise<dvc_msgs::SearchResults>((ros_ns + "/filtration/search_results").c_str(), 1);
		yolo_sub = nh->subscribe("/darknet_ros/bounding_boxes", 1, &DataResender::yolo_callback, this);
		ROS_INFO((ros_ns + " IS A NEW TOPIC").c_str());
	}

	/**
	 * @brief Callback for found objects by darknet_ros
	 *
	 * @param msg Our messege
	 */
	void yolo_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
	{
		dvc_msgs::SearchResults AllObjects;
		Eigen::VectorXd z(1, m);
		for (const auto &bounding_box_auto : msg->bounding_boxes)
		{
			int centre_x = (int)(bounding_box_auto.xmax + bounding_box_auto.xmin) / 2;
			int centre_y = (int)(bounding_box_auto.ymax + bounding_box_auto.ymin) / 2;
			// 271 -> 0,5 pi rad (90 deg)
			SingleObject.angle = (double)((centre_x - MIDDLE_X) * X_TO_DEG);

			int size = abs(bounding_box_auto.xmax - bounding_box_auto.xmin)

						   AllObjects.search_results.push_back(SingleObject);
		}
		if (ekf.init == false)
		{
			ekf.Init(x_in, P_in, F_in, C_in, R_in, Q_in)
		}
		else
			ekf.UpdateEKF(z, C);
		pub.publish(AllObjects);
	}

	void flags_callback(const std_msgs::UInt8::ConstPtr &msg)
	{
		flags = msg->data;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "filtration");
	ros::NodeHandle filtration("~");
	DataResender yt = DataResender(&filtration);
	ros::Rate rate(4.0);
	while (ros::ok())
	{
		if(ekf.init == true)
		{
			ekf.Predict();
		}
	}
}