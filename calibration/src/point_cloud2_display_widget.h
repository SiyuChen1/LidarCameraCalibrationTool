/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef POINT_CLOUD2_DISPLAY_WIDGET_H
#define POINT_CLOUD2_DISPLAY_WIDGET_H

#include <assert.h>
//#include <memeory>

#include <ros/ros.h>
#include <ros/console.h> // 使用 ros message，nodehandle 等文件
#include <rviz/panel.h>   //plugin基类的头文件
// ros .h source files

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/default_plugin/point_cloud2_display.h"
#include "rviz/default_plugin/image_display.h"
//librviz libraries function

#include <QMainWindow>
#include <QWidget>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QComboBox>
#include <QCheckBox>
#include <QColorDialog>
#include <QLineEdit>
#include <QToolButton>
#include <QDebug>
#include <QEvent>
#include <QColor>
#include <QSlider>

// qt process and thread,which are used to simplify
#include <QProcess>
#include <QThread>

#include <QMetaType>
// this header file to use SIGNAL to emit non-qt defined datatype
// qt libraries

#include <QThread>
#include <QMutex>
#include <QMutexLocker>
#include <QUuid>
#include <QFile>
#include <QDir>

#include "CalibBoardDetector.h"
#include "point_cloud2_set_widget.h"
#include "xml_read_and_write.h"

#define XSTR(x) #x
#define STR(x) XSTR(x)
const std::string DEFAULT_PATH = STR(WORK_DEFAULT_PATH);
//get cmake current source path

using namespace XML;

namespace PointCloud2
{

class PointCloud2Display: public QWidget
{
  Q_OBJECT
public:
    PointCloud2Display(ros::NodeHandle &nh,QWidget* parent = 0);
    virtual ~PointCloud2Display();
    // update topic name
    void UpdateMarkerArrayTopicName();
    void UpdatePointCloud2TopicName();
    void InitCalibBoardDetectionNode();

private Q_SLOTS:
    // use QThread and QObject
    void startObjectDetection(bool checked);

    // use QProcess and XML file write
    void startRemoveGround(bool checked);

    // get shell type
    void getShellType(int index);

    void setFixedFrameName(const QString& fixed_frame_name_);

    void setPointCloud2TopicName(const int& topic_name_index_);

    void setMarkerArrayTopicName(const int& topic_name_index_);

    void setSelectable(const bool& selectable_);

    void setSize();

    void setAlpha(const int& alpha_para_);

    void setDecayTime(const QString& decay_time_para_);

    void setPositionTransform(const QString& position_transform_model_);

    void setColorTransform(const QString& color_transform_model_);

    void setMinColor(const QColor& min_color_);

    void setMaxColor(const QColor& max_color_);

    bool eventFilter(QObject* object, QEvent* event);

private:
    // rviz namespace definied class
    rviz::VisualizationManager* manager_;
    rviz::RenderPanel* render_panel_;
    rviz::Display* grid_;
    rviz::Display* tf_;
    rviz::Display* pointcloud2_;
    rviz::Display* marker_array_;

    // used to set property
    QString frame_name_;
    QString topic_name_;
    bool unreliable_state_;
    bool select_state_;
    QString style_type_;
    QString size_para_;
    int alpha_para_;
    QString decay_time_para_;
    QString position_transform_current_text_;
    QString color_transform_current_text_;

    //layout
    QVBoxLayout* set_layout_;
    QHBoxLayout* main_layout;

    // widget to make layout
    HumanInterfaceForPointCloud2DisplayWidget* set_widget_;
    CaliSetWidget* cali_set_widget_;
    MarkerArraySetWidget* marker_set_widget_;

    // add by Siyu Chen,use method 1 QThread and function QObject::movetoThread()
    QThread t_pointcloud2_detector_;

    // XML write 
    XMLWrite* xml;
    //method 2 use QProcess by Siyu Chen
    QProcess* start_remove_ground_;

    // shell type
    QString shell_type_;


Q_SIGNALS:

    void start_detect_node_(const std::string& topic_name_);

    void send_frame_id_(const std::string& frame_id_);

};

}


#endif 
