#include <gnc_functions.hpp>
#include <math.h>
#include <mutex>
#include <queue>
#include <memory>
#include <thread>
#include <chrono>
#include <condition_variable>
#include <stdlib.h>
#include <time.h>

using namespace std::chrono_literals;

//include API 


template<typename T>
class ThreadSafeQueue
{
private:
	mutable std::mutex mut;
	std::queue<std::shared_ptr<T>> dataQueue;
	std::condition_variable dataCond;

public:
	ThreadSafeQueue(){}

	std::shared_ptr<T> waitAndPop()
	{
		std::unique_lock<std::mutex> lk(mut);
		dataCond.wait(lk,[this]{return !dataQueue.empty();});
		std::shared_ptr<T> res = dataQueue.front();
		dataQueue.pop();
		return res;
	}

	void push(T newValue)
	{
		std::shared_ptr<T> data(std::make_shared<T>(std::move(newValue)));
		std::lock_guard<std::mutex> lk(mut);
		dataQueue.push(data);
		dataCond.notify_one();
	}
};

void addPoints(ThreadSafeQueue<gnc_api_waypoint>& queue)
{
	srand(time(NULL));

	while(ros::ok())
	{
		gnc_api_waypoint nextWayPoint;
		nextWayPoint.x = rand() % 100 - 50;
		nextWayPoint.y = rand() % 100 - 50;
		nextWayPoint.z = rand() % 19 + 1;
		nextWayPoint.psi = 0;
		queue.push(nextWayPoint);
		std::this_thread::sleep_for(1s);
	}
}

int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle gnc_node("~");
	
	//initialize control publisher/subscribers
	init_publisher_subscriber(gnc_node);

  	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	wait4start();

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(3);

	
	ThreadSafeQueue<gnc_api_waypoint> queue;

	std::thread t1 (&addPoints,std::ref(queue));


	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
		if(check_waypoint_reached(.3) == 1)
		{
			auto point = queue.waitAndPop();
			gnc_api_waypoint wayPoint = *point.get();


			set_destination(wayPoint.x,wayPoint.y,wayPoint.z, wayPoint.psi);
			
		}	
		
	}
	land();
	t1.join();
	return 0;
}