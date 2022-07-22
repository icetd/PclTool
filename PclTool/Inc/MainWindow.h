#pragma execution_character_set("utf-8")
#ifndef _MAINWINDOW_H
#define _MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include "ui_MainWindow.h"

#include <QDebug>
#include <QColorDialog>
#include <QFileDialog>
#include <QTime>
#include <QDir>
#include <QFile>
#include <QtMath>
#include <QDirIterator>
#include <QMessageBox>

#include "PclFunction.h"
#include "ViewSelectColor.h"
#include "ViewRendering.h"
#include "VIewPointSize.h"

#include "FilterVoxel.h"
#include "FilterAvoxel.h"
#include "FilterUniform.h"
#include "FilterRandom.h"
#include "FilterGridmin.h"
#include "FilterStatistical.h"
#include "FilterRadius.h"


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    

    void initView();
    void refreshViewe();
    void showMessage();
    void getCloudScope(PointCloudT cloud_in);

protected: 
    void pp_callback(const pcl::visualization::PointPickingEvent& event, void* args);
    void areapp_callback(const pcl::visualization::AreaPickingEvent& event, void* args);
    void mouseEventOccurred(const pcl::visualization::MouseEvent& event, void* args);
    void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* args);

    
private slots:

    /*Camera View && Fast Filter*/
    
    void btn_Up_clicked();
    void btn_Down_clicked();
    void btn_Forward_clicked();
    void btn_Back_clicked();
    void btn_Left_clicked();
    void btn_Right_clicked();

    void btn_Filter_I_clicked();

    /*File Menu*/
    void Open_clicked();
    void Save_clicked();
    void BackUp_clicked();
    void Quit_clicked();

    /*View Menu*/
    void View_background_pressed();
    
    void View_render_pressed();
    void Rendering_setting(QString data);
    
    void View_color_pressed();

    void View_size_pressed();
    void PointSize_setting(QString data);

    /*Filter Menu*/
    void Filter_voxel_pressed();
    void Filter_voxel_run(QString data);


    void Filter_avoxel_pressed();
    void Filter_avoxel_run(QString data);

    void Filter_uniform_pressed();
    void Filter_uniform_run(QString data);

    void Filter_random_pressed();
    void Filter_random_run(QString data);
 
    void Filter_gridmin_pressed();
    void Filter_gridmin_run(QString data);

    void Filter_statistical_pressed();
    void Filter_statistical_run(QString data_1, QString data_2);


    void Filter_radius_pressed();
    void Filter_radius_run(QString data_1, QString data_2);

private:


    Ui::MainWindowClass *ui;
 

    /** @brief The PCL visualizer object */
    pcl::visualization::PCLVisualizer::Ptr viewer_;
    
    /** @brief the max and min Point*/
    PointT pMax;
    PointT pMin;
   
    /** @brief The point cloud displayed */
    std::vector<std::string> cloud_name;
    PointCloudT::Ptr cloud_;
    PointCloudT::Ptr cloud_original;
    PointCloudT::Ptr cloud_reference;
    PointCloudT::Ptr cloud_area;
    PointCloudT::Ptr clicked_points_3d;
    PointCloudT::Ptr cloud_volume;
	
    /** @brief The triangles displayed */
    pcl::PolygonMesh::Ptr triangles;
    

    /** @keyboard & mouse data*/
    int mouse_currect_x;
    int mouse_currect_y;
    bool isPickingMode = false;
    bool isFirstPick = false;
    PointT curP ,lastP;
    int line_id = 0;


    /** @brief The volume base data */
    float reference_plane_h;
    aream areaMessage;

    /** @brief Set color */
    QColor background_color;
    ViewSelectColor *dialog_color;

    /** @brief The point cloud Render */
    ViewRendering *dialog_render;
    
    /** @brief The point cloud Size */
    int point_size = 1;
    ViewPointSize *dialog_size;


    /** @brief filter voxel */
    FilterVoxel *dialog_voxel;
    
    /** @brief filter aoxel */
    FilterAvoxel *dialog_avoxel;

    /** @brief filter uniform */
    FilterUniform *dialog_uniform;

    /** @brief filter Random */
    FilterRandom* dialog_random;

    /** @brief filter Gridmin */
    FilterGridmin* dialog_gridmin;

    /** @brief filter Statisical */
    FilterStatistical* dialog_statistical;

    /** @brief filter Radius */
    FilterRadius* dialog_radius;
};

#endif
