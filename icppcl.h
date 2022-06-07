#ifndef ICPPCL_H
#define ICPPCL_H

#include <QMainWindow>
#include <boost/algorithm/string/split.hpp>
#include <boost/thread/thread.hpp>


// std
#include <vector>
#include <string>
#include <algorithm>

// Qt
#include <QtWidgets/QMainWindow>
#include <QString>
#include <QDebug>
#include <QLabel>
#include <QMessageBox>
#include <QAction>
#include <QMenu>
#include <QMenuBar>
#include <QToolBar>
#include <QStatusBar>
#include <QFileDialog>
#include <QColorDialog>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkRenderWindow.h>
#include <QTextEdit>
#include <QTime>
#include <QMouseEvent>
#include <QDesktopServices>
#include <QUrl>
#include <QTreeWidgetItem>
#include "ui_icppcl.h"

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
//#include <pcl/features/normal_3d.h>


// sampling
#include<pcl/keypoints/uniform_sampling.h>

// Features
#include <pcl/features/pfh.h>
#include <pcl/visualization/pcl_plotter.h>



//VTK
#include <vtkActor.h>
#include <vtkBoxWidget.h>
#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkConeSource.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkTransform.h>
#include <vtk-9.1/vtkAutoInit.h>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>

VTK_MODULE_INIT(vtkRenderingOpenGL2); // VTK was built with vtkRenderingOpenGL2
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);

//opencv

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;



QT_BEGIN_NAMESPACE
namespace Ui { class ICPPCL; }
QT_END_NAMESPACE

class ICPPCL : public QMainWindow
{
    Q_OBJECT
protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    PointCloudT::Ptr cloud_now;
    std::vector<PointCloudT::Ptr> cloud_show;
    std::map<int, pcl::PointCloud<pcl::Normal>::Ptr> nor_show;

    void open();

    std::string model_dirname = "";
    long total_points = 0; //Total amount of points in the viewer


public:
    ICPPCL(QWidget *parent = nullptr);
    std::string getFileName(std::string file_name);
    void print4x4MatrixT(const Eigen::Matrix4d & matrix);
    void keyboardEventOccured(const pcl::visualization::KeyboardEvent& event, void* non);

    void setCloudRGB(PointCloudT::Ptr &cloud, unsigned int r, unsigned int g, unsigned int b);
    void setCloudAlpha(PointCloudT::Ptr &cloud, unsigned int a);
    void setConsole(QString operation, QString detail);



    void updatePointcloud();
    void updatePropertyTable();
    void updateConsoleTable();
    void updateDataTree();
    void updateNormals();

    ~ICPPCL();

public slots:
    //Display menu slots
    void pointcolorChanged();
    void pointcolorRandom();
    void pointHide();
    void pointShow();

    void itemSelected(QTreeWidgetItem* item, int count);
    void itemPopmenu(const QPoint&);

    //Sampling Algorithm
    void uniformSampling();

    //Features
    void normalVector();
    void pfh();



private:
    Ui::ICPPCL *ui;
    bool next = false;
};
#endif // ICPPCL_H
