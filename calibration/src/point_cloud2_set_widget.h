#ifndef POINT_CLOUD2_SET_WIDGET_H
#define POINT_CLOUD2_SET_WIDGET_H

#include <assert.h>
// std c++ package 
// use assert()

#include <QMainWindow>
#include <QObject>
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

#include <QProcess>
#include <QThread>

#include <QMetaType>
// this header file to use SIGNAL to emit non-qt defined datatype
// qt libraries


namespace PointCloud2
{

class HumanInterfaceForPointCloud2DisplayWidget : public QWidget
{

    Q_OBJECT

public:
    HumanInterfaceForPointCloud2DisplayWidget(QWidget *parent = 0);
    ~HumanInterfaceForPointCloud2DisplayWidget();
    void initWidget();
public:
    //QLabel for tipp
    QLabel* fixed_frame_label_;
    QLabel* topic_label_;
    QLabel* unreliable_label_;
    QLabel* selectable_label_;
    QLabel* style_label_;
    QLabel* size_m_label_;
    QLabel* alpha_label_;
    QLabel* decay_time_label_;
    QLabel* position_transform_label_;
    QLabel* color_transform_label_;
    QLabel* min_color_label_;
    QLabel* max_color_label_;

    //input widget
    QLineEdit* fixed_frame_get_;
    QComboBox* topic_getName_;
    QCheckBox* unreliable_getP_;
    QCheckBox* select_getP_;
    QComboBox* style_get_;
    QLineEdit* size_get_;
    QSlider* alpha_get_;
    QLineEdit* decay_time_get_;
    QComboBox* position_transform_get_;
    QComboBox* color_transform_get_;
    QLabel* min_color_display_;
    QToolButton* min_color_pick_;
    QLabel* max_color_display_;
    QToolButton* max_color_pick_;

    //layout for this class
    QGridLayout *layout;
    QHBoxLayout* min_color_layout_;
    QHBoxLayout* max_color_layout_;

    //parameters of the state or current string
    
private:

    QString min_color_text_;
    QString max_color_text_;

    QColor min_color_;
    QColor max_color_;

private Q_SLOTS:
    void getMinColor();
    
    void getMaxColor();

Q_SIGNALS:
    void emitMinColor(QColor emit_min_color_);

    void emitMaxColor(QColor emit_max_color_);
    
};

class CaliSetWidget : public QWidget
{
    Q_OBJECT
public:
    CaliSetWidget(QWidget *parent = 0);
    ~CaliSetWidget();
    void initWidget();
public:
    QLabel* shell_sort_;
    QComboBox* shelltype_get_;
    QLabel* start_cali_node_;
    QLabel* start_remove_ground_;
    QCheckBox* remove_ground_state_;
    QCheckBox* set_state_;
    QGridLayout* main_layout_;
};


class MarkerArraySetWidget : public QWidget
{
    Q_OBJECT
public:
    MarkerArraySetWidget(QWidget *parent = 0);
    ~MarkerArraySetWidget();
private:
    void initWidget();
public:
    QLabel* topic_name_markerarray_;
    QComboBox* topic_name_get_;
    QGridLayout* main_layout_;
};

}
#endif