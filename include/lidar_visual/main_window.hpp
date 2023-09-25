/**
 * @file /include/lidar_visual/main_window.hpp
 *
 * @brief Qt based gui for lidar_visual.
 *
 * @date November 2010
 **/
#ifndef lidar_visual_MAIN_WINDOW_H
#define lidar_visual_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <math.h>
#include <vector>
#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)*M_PI/180.)


/*****************************************************************************
** Namespace
*****************************************************************************/
using namespace std;
using namespace cv;
namespace lidar_visual {

typedef struct
{
    float deg = 0.0;
    float dist = 99;
}BeamInfo;


/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();


	void closeEvent(QCloseEvent *event); // Overloaded function

        bool cmp(const BeamInfo &p1, const BeamInfo &p2){
            if(p1.deg < p2.deg){
                return true;
            }
            else if(p1.deg == p2.deg){
                return p1.deg < p2.deg;
            }
            else{
                return false;
            }
        }


public Q_SLOTS:
		void laserCallback();
private:
	Ui::MainWindowDesign ui;
	QNode qnode;

    int mWidth = 640;
    int mHeight = 480;

};

}  // namespace lidar_visual

#endif // lidar_visual_MAIN_WINDOW_H
