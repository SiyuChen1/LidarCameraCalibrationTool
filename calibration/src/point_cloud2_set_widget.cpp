#include "point_cloud2_set_widget.h"

namespace PointCloud2
{

CaliSetWidget::CaliSetWidget(QWidget *parent){
    // new layout
    main_layout_ = new QGridLayout(this);
    assert(main_layout_);
    shell_sort_ = new QLabel("Shell Type",this);
    assert(shell_sort_);
    shelltype_get_ = new QComboBox(this);
    assert(shelltype_get_);
    start_cali_node_ = new QLabel("Start Detection",this);
    assert(start_cali_node_);
    set_state_ = new QCheckBox(this);
    assert(set_state_);
    start_remove_ground_ = new QLabel("Remove Ground",this);
    assert(start_remove_ground_);
    remove_ground_state_ = new QCheckBox(this);
    assert(remove_ground_state_);

    main_layout_->addWidget(shell_sort_,0,0);
    main_layout_->addWidget(shelltype_get_,0,1);
    main_layout_->addWidget(start_cali_node_, 1 , 0 );
    main_layout_->addWidget(set_state_, 1 , 1 );
    main_layout_->addWidget(start_remove_ground_,2,0);
    main_layout_->addWidget(remove_ground_state_,2,1);

    initWidget();

    this->setLayout(main_layout_);
}

void CaliSetWidget::initWidget(){
    shelltype_get_->addItem("bash");
    shelltype_get_->addItem("zsh");
    shelltype_get_->addItem("sh");
}

CaliSetWidget::~CaliSetWidget(){
    delete set_state_;
    delete start_cali_node_;
    delete main_layout_;
}

MarkerArraySetWidget::MarkerArraySetWidget(QWidget* parent){
    // new layout and label
    topic_name_markerarray_ = new QLabel("Marker Topic",this);
    assert(topic_name_markerarray_);
    topic_name_get_ = new QComboBox(this);
    assert(topic_name_get_);
    main_layout_ = new QGridLayout(this);
    assert(main_layout_);

    // make up layout 
    main_layout_->addWidget(topic_name_markerarray_,0,0);
    main_layout_->addWidget(topic_name_get_,0,1);

    // init this widget
    this->initWidget();

    this->setLayout(main_layout_);
}

MarkerArraySetWidget::~MarkerArraySetWidget(){
    delete topic_name_get_;
    delete topic_name_markerarray_;
    delete main_layout_;
}

void MarkerArraySetWidget::initWidget(){
    topic_name_get_->addItem("----");
}

HumanInterfaceForPointCloud2DisplayWidget::HumanInterfaceForPointCloud2DisplayWidget(QWidget *parent)
    :min_color_(0,0,0),max_color_(255,255,255)
{
    //new Qlabel and inherited by this
    fixed_frame_label_ = new QLabel("Fixed Frame",this);
    topic_label_ = new QLabel("Topic",this);
    unreliable_label_ = new QLabel("Unreliable",this);
    selectable_label_ = new QLabel("Selectable",this);
    style_label_ = new QLabel("Style",this);
    size_m_label_ = new QLabel("Size (m)",this);
    alpha_label_ = new QLabel("Alpha",this);
    decay_time_label_ = new QLabel("Decay Time",this);
    position_transform_label_ = new QLabel("Position Transform",this);
    color_transform_label_ = new QLabel("Color Transform",this);
    min_color_label_ = new QLabel("Min Color",this);
    max_color_label_ = new QLabel("Max Color",this);

    //new input widget
    fixed_frame_get_ = new QLineEdit(this);
    topic_getName_ = new QComboBox(this);
    unreliable_getP_ = new QCheckBox(this);
    select_getP_ = new QCheckBox(this);
    style_get_ = new QComboBox(this);
    size_get_ = new QLineEdit(this);
    alpha_get_ = new QSlider(this);
    decay_time_get_ = new QLineEdit(this);
    position_transform_get_ = new QComboBox(this);
    color_transform_get_ = new QComboBox(this);
    min_color_display_ = new QLabel(this);
    max_color_display_ = new QLabel(this);

    //new toolbutton
    min_color_pick_ = new QToolButton(this);
    max_color_pick_ = new QToolButton(this);

    alpha_get_->setOrientation(Qt::Horizontal);  // 水平方向
    alpha_get_->setMinimum(0);  // 最小值
    alpha_get_->setMaximum(100);  // 最大值
    alpha_get_->setSingleStep(1);

    //set layout
    layout = new QGridLayout(this);

    // layout for color select
    min_color_layout_ = new QHBoxLayout;
    max_color_layout_ = new QHBoxLayout;
    min_color_layout_->addWidget(min_color_display_,3);
    min_color_layout_->addWidget(min_color_pick_,1);
    max_color_layout_->addWidget(max_color_display_,3);
    max_color_layout_->addWidget(max_color_pick_,1);

    // set up grid layout
    layout->addWidget(fixed_frame_label_,0,0);
    layout->addWidget(fixed_frame_get_,0,1);
    layout->addWidget(topic_label_,1,0);
    layout->addWidget(topic_getName_,1,1);
    layout->addWidget(unreliable_label_,2,0);
    layout->addWidget(unreliable_getP_,2,1);
    layout->addWidget(selectable_label_,3,0);
    layout->addWidget(select_getP_,3,1);
    layout->addWidget(style_label_,4,0);
    layout->addWidget(style_get_,4,1);
    layout->addWidget(size_m_label_,5,0);
    layout->addWidget(size_get_,5,1);
    layout->addWidget(alpha_label_,6,0);
    layout->addWidget(alpha_get_,6,1);
    layout->addWidget(decay_time_label_,7,0);
    layout->addWidget(decay_time_get_,7,1);
    layout->addWidget(position_transform_label_,8,0);
    layout->addWidget(position_transform_get_,8,1);
    layout->addWidget(color_transform_label_,9,0);
    layout->addWidget(color_transform_get_,9,1);
    layout->addWidget(min_color_label_,10,0);
    layout->addLayout(min_color_layout_,10,1);
    layout->addWidget(max_color_label_,11,0);
    layout->addLayout(max_color_layout_,11,1);

    this->setLayout(layout);

    initWidget();
    

    // connect signals and slots
    connect(this->min_color_pick_,SIGNAL(clicked(bool)),this,SLOT(getMinColor()));
    connect(this->max_color_pick_,SIGNAL(clicked(bool)),this,SLOT(getMaxColor()));

}

void HumanInterfaceForPointCloud2DisplayWidget::getMaxColor()
{
    QColor init_color(0,0,0);

    // get max_color from color dialog
    max_color_ = QColorDialog::getColor(init_color,this,"Get Max Color",QColorDialog::ShowAlphaChannel);
    max_color_text_ = "QColor R(" + QString::number(max_color_.red()) + "),G(" + QString::number(max_color_.green())
           +"),B("  + QString::number(max_color_.blue()) +")";
    
    // display max_color
    max_color_display_->setText(max_color_text_);

    Q_EMIT emitMaxColor(max_color_);
}

void HumanInterfaceForPointCloud2DisplayWidget::getMinColor()
{
    QColor init_color(0,0,0);

    // get min_color from color dialog
    min_color_ = QColorDialog::getColor(init_color,this,"Get Min Color",QColorDialog::ShowAlphaChannel);
    min_color_text_ = "QColor R(" + QString::number(min_color_.red()) + "),G(" + QString::number(min_color_.green())
           +"),B("  + QString::number(min_color_.blue()) +")";
    
    // display min_color 
    min_color_display_->setText(min_color_text_);

    Q_EMIT emitMinColor(min_color_);
}

void HumanInterfaceForPointCloud2DisplayWidget::initWidget()
{
    // init value for this widget
    fixed_frame_get_->setText("velodyne");

    unreliable_getP_->setCheckState(Qt::Unchecked);

    select_getP_->setCheckState(Qt::Checked);

    topic_getName_->addItem("----");

    style_get_->addItem("Points");
    style_get_->addItem("Squares");
    style_get_->addItem("Flat Squares");
    style_get_->addItem("Spheres");
    style_get_->addItem("Boxes");

    size_get_->setText("0.03");

    alpha_get_->setValue( 80 );

    decay_time_get_->setText("0");

    position_transform_get_->addItem("XYZ");

    color_transform_get_->addItem("Intensity");
}

HumanInterfaceForPointCloud2DisplayWidget::~HumanInterfaceForPointCloud2DisplayWidget(){
    delete fixed_frame_get_;
    delete fixed_frame_label_;
    delete topic_label_;
    delete topic_getName_;
    delete unreliable_getP_;
    delete unreliable_label_;
    delete selectable_label_;
    delete select_getP_;
    delete style_get_;
    delete style_label_;
    delete size_m_label_;
    delete size_get_;
    delete alpha_get_;
    delete alpha_label_;
    delete decay_time_get_;
    delete decay_time_label_;
    delete position_transform_get_;
    delete position_transform_label_;
    delete color_transform_get_;
    delete color_transform_label_;
    delete min_color_pick_;
    delete min_color_display_;
    delete min_color_label_;
    delete max_color_display_;
    delete max_color_label_;
    delete max_color_pick_;
    delete min_color_layout_;
    delete max_color_layout_;
    delete layout;
}

}