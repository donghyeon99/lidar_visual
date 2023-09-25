/**
 * @file /include/lidar_visual/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef lidar_visual_QNODE_HPP_
#define lidar_visual_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <iostream>

#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace lidar_visual {

/*****************************************************************************
** Class
*****************************************************************************/
class QNode : public QThread {
    Q_OBJECT
public:
    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    void run();
    std::string m_minTopic;
    std::string m_laserTopic;

    ros::Subscriber m_laserSub;
    ros::Publisher m_minPub;

    sensor_msgs::LaserScan m_laserData;

    void laserCallback(const sensor_msgs::LaserScan& data);

    cv::Mat *img_qnode;

Q_SIGNALS:
    void rosShutdown();
    void laser();
private:
    int init_argc;
    char** init_argv;
};

}  // namespace lidar_visual

#endif /* lidar_visual_QNODE_HPP_ */
