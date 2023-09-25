/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/lidar_visual/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace enc = sensor_msgs::image_encodings;

namespace lidar_visual {

bool isRecv = false;

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"lidar_visual");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.   
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    nh.param<std::string>("minData_topic", m_minTopic, "/data");
    nh.param<std::string>("scan_topic", m_laserTopic, "/data");

    m_laserSub = n.subscribe(m_laserTopic, 100, &QNode::laserCallback, this);
    m_minPub = n.advertise<std_msgs::Float64MultiArray>(m_minTopic, 100);

    start();
    return true;
}


void QNode::run() {
    ros::Rate loop_rate(33);

    while ( ros::ok() ) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::laserCallback(const sensor_msgs::LaserScan& data)
{
    m_laserData = data;
    Q_EMIT laser();
}

}  // namespace lidar_visual
