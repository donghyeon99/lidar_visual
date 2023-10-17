/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/lidar_visual/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/


namespace lidar_visual {
extern bool isRecv;

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    qnode.init();

    QObject::connect(&qnode, SIGNAL(laser()), this, SLOT(laserCallback()));
}

MainWindow::~MainWindow() {}

void MainWindow::laserCallback()
{
  vector<BeamInfo> minFilter;
  vector<BeamInfo> m_dist;
  sensor_msgs::LaserScan lidarData = qnode.m_laserData;
  
  double m_rhoZero = 3;

  int m_kernelSize = 5;

    // deg: -180 ~ 180
    int count = lidarData.scan_time / lidarData.time_increment;

    for(int i = 0; i < lidarData.ranges.size(); i++)
    {
        float degree = RAD2DEG(lidarData.angle_min + lidarData.angle_increment * i);

        BeamInfo data;
        data.deg = degree;
        data.dist = lidarData.ranges[i] >= lidarData.range_max ? lidarData.range_max : lidarData.ranges[i];

        m_dist.push_back(data);
    }
    //  cout<<"scan_time :"        <<lidarData.scan_time<<endl;
    //  cout<<"time_increment :"   <<lidarData.time_increment<<endl;
    //  cout<<"angle_increment :"  <<lidarData.angle_increment<<endl;
    //  cout<<"angle_min :"        <<lidarData.angle_min<<endl;
    //  cout<<"angle_max :"        <<lidarData.angle_max<<endl;
    //  cout<<"range size :"       <<lidarData.ranges.size()<<endl;
    //  cout<<m_dist.size()<<endl;

    // minFilter
    vector<BeamInfo> beamdata;
    bool active = false;
    BeamInfo minValue;
    for(int i = 0; i < m_dist.size()-m_kernelSize; i++)
    {
        int activeCnt = 0;
        int inactiveCnt = 0;
        minValue.dist  = 99.0;

        for(int k = 0; k < m_kernelSize; k++) // kernel 만들고 최소 값 구하기
        {
            //                      cout<<"kernel["<<k<<"]: "<<kernel.dist<<endl;
            BeamInfo kernel = m_dist[i+k];
            kernel = m_dist[i+k];
            if(kernel.dist > m_rhoZero)
            {
                inactiveCnt++;
                continue;
            }
            else
                activeCnt++;


            if(kernel.dist < minValue.dist)
                minValue = kernel;
        }

        if(active == true)
        {
            if(inactiveCnt >= m_kernelSize)
            {
                active = false;
            }
            else
            {
                if(minValue.dist < beamdata[beamdata.size()-1].dist)
                {
                    beamdata[beamdata.size()-1] = minValue;
                }
            }
        }
        else
        {
            if(activeCnt >= m_kernelSize)
            {
                active = true;
                BeamInfo newData;
                newData.deg = 0;
                newData.dist = 16;
                beamdata.push_back(newData);
            }
        }
    }

    minFilter.clear();

    for(int i = 0; i < beamdata.size(); i++)
    {
        double rho = beamdata[i].dist;
        if(rho < m_rhoZero && rho != 0.0)
        {
            minFilter.push_back(beamdata[i]);
        }
    }

    std_msgs::Float64MultiArray array_msg;
    array_msg.data.resize(minFilter.size()*2);
    int j = 0;
    for(int i = 0; i < minFilter.size(); i++)
    {
        BeamInfo data = minFilter[i];
        array_msg.data[j] = data.deg;
        array_msg.data[j+1] = data.dist;
        j +=2;
    }

    qnode.m_minPub.publish(array_msg);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}  // namespace lidar_visual

