
#include "point_cloud2_display_widget.h"

namespace PointCloud2
{

// Constructor for PointCloud2Display.  This does most of the work of the class.
PointCloud2Display::PointCloud2Display(ros::NodeHandle &nh,QWidget* parent )
  :frame_name_("velodyne")
{
    // use CalibBoardDetectionNode
    // 1. QThread thread_ is variable of this class (define in class)
    // 2. then use ros::NodeHandle to constructor a class pointer
    // 3. move class pointer to thread 
    // 4. start thread 
    CalibBoardDetectionNode* cali_node_ = new CalibBoardDetectionNode(nh);
    assert(cali_node_);
    cali_node_->moveToThread(&t_pointcloud2_detector_);
    t_pointcloud2_detector_.start();

    // set icon of application
    QString icon_path_ = QString::fromStdString(DEFAULT_PATH) + "/icon/tool.png";
    QPixmap icon_pix_(icon_path_);
    setWindowIcon(QIcon(icon_pix_));

    // new xml write pointer
    xml = new XMLWrite;
    assert(xml);

    // new a pointer points to QProcess
    start_remove_ground_ = new QProcess;

    //Construct the human -computer interface layout
    set_widget_ = new HumanInterfaceForPointCloud2DisplayWidget(this);
    assert(set_widget_);
    cali_set_widget_ = new CaliSetWidget(this);
    assert(cali_set_widget_);
    marker_set_widget_ = new MarkerArraySetWidget(this);
    assert(marker_set_widget_);
    set_layout_ = new QVBoxLayout;
    assert(set_layout_);
    set_layout_->addWidget(set_widget_);
    set_layout_->addWidget(cali_set_widget_);
    set_layout_->addWidget(marker_set_widget_);

    // Construct and lay out render panel.
    render_panel_ = new rviz::RenderPanel();
    assert(render_panel_);
    main_layout = new QHBoxLayout;
    assert(main_layout);
    main_layout->addLayout(set_layout_,1);
    main_layout->addWidget(render_panel_,6);

    // Set the top-level layout for this PointCloud2Display widget.
    // setLayout( main_layout );
    this->setLayout(main_layout);
 
    // Next we initialize the main RViz classes.
    // The VisualizationManager is the container for Display objects,
    // holds the main Ogre scene, holds the ViewController, etc.  It is
    // very central and we will probably need one in every usage of
    // librviz.
    manager_ = new rviz::VisualizationManager( render_panel_ );
    assert(manager_);
    render_panel_->initialize( manager_->getSceneManager(), manager_ );
    manager_->initialize();
    manager_->startUpdate();

    // Create a Grid display.
    manager_->setFixedFrame("odom");
    grid_ = manager_->createDisplay( "rviz/Grid", "root", true );

    // create tf display widget
    tf_ = manager_->createDisplay("rviz/TF","test TF",true);

    // create pointcloud2display widget
    pointcloud2_ = manager_->createDisplay("rviz/PointCloud2","PointCloud2 Display Widget",true);

    // create markerarray display
    marker_array_ = manager_->createDisplay("rviz/MarkerArray","marker array",true);

    // make qt signal emit non-qt datatype 
    // for example std::string and std::string&
    qRegisterMetaType<std::string>("std::string");     
    qRegisterMetaType<std::string>("std::string&");     

    // send frame_id_
    connect(this,SIGNAL(send_frame_id_(std::string)),cali_node_,SLOT(set_frame_id(std::string)));

    //connect thread and object
    connect(this,SIGNAL(start_detect_node_(std::string)),cali_node_,SLOT(run(std::string)));

    // start calibration node
    connect(cali_set_widget_->set_state_,SIGNAL(clicked(bool)),this,SLOT(startObjectDetection(bool)));

    // start remove_ground .launch file
    connect(cali_set_widget_->remove_ground_state_,SIGNAL(clicked(bool)),this,SLOT(startRemoveGround(bool)));

    // Make signal/slot connections.
    connect( set_widget_->fixed_frame_get_,SIGNAL(textChanged(QString)),this,SLOT(setFixedFrameName(QString)));
    connect(set_widget_->topic_getName_,SIGNAL(activated(int)),this,SLOT(setPointCloud2TopicName(int)));
    connect(marker_set_widget_->topic_name_get_,SIGNAL(activated(int)),this,SLOT(setMarkerArrayTopicName(int)));
    connect(set_widget_->select_getP_,SIGNAL(clicked(bool)),this,SLOT(setSelectable(bool)));
    connect(set_widget_->size_get_,SIGNAL(editingFinished()),this,SLOT(setSize()));
    connect(set_widget_->alpha_get_,SIGNAL(valueChanged(int)),this,SLOT(setAlpha(int)));
    connect(set_widget_->decay_time_get_,SIGNAL(textChanged(QString)),this,SLOT(setDecayTime(QString)));
    connect(set_widget_->position_transform_get_,SIGNAL(activated(QString)),this,SLOT(setPositionTransform(QString)));
    connect(set_widget_->color_transform_get_,SIGNAL(activated(QString)),this,SLOT(setColorTransform(QString)));
    connect(set_widget_,SIGNAL(emitMinColor(QColor)),this,SLOT(setMinColor(QColor)));
    connect(set_widget_,SIGNAL(emitMaxColor(QColor)),this,SLOT(setMaxColor(QColor)));

    // get shell type
    connect(cali_set_widget_->shelltype_get_,SIGNAL(activated(int)),this,SLOT(getShellType(int)));

    // update topic list
    UpdatePointCloud2TopicName();
    UpdateMarkerArrayTopicName();

}

void PointCloud2Display::getShellType(int index){
    shell_type_ = cali_set_widget_->shelltype_get_->itemText(index);
}

void PointCloud2Display::InitCalibBoardDetectionNode(){
}

// Destructor.
PointCloud2Display::~PointCloud2Display()
{
    // the right way to handle thread
    t_pointcloud2_detector_.quit();
    t_pointcloud2_detector_.wait();
    delete xml; 

    delete start_remove_ground_;
    delete set_widget_;
    delete cali_set_widget_;
    delete marker_set_widget_;
    delete set_layout_;
    delete manager_;
    delete render_panel_;
    delete main_layout;
}

void PointCloud2Display::startRemoveGround(bool checked){
    if(checked){
        // write XML and update topic name
        QString remove_ground_launch_path_ = QString::fromStdString(DEFAULT_PATH) 
            + "/launch/remove_ground.launch";
        xml->write(remove_ground_launch_path_,topic_name_);

        // write sh file
        // QString sh_path_ = QString::fromStdString(DEFAULT_PATH) + "/sh/remove_ground.sh";

        // QFile file(sh_path_);
        // if(!file.open(QIODevice::WriteOnly))
        // {
        //     qDebug()<<file.errorString();
        // }
        // QString source_path_ = QString::fromStdString(DEFAULT_PATH);
        // QString path_base_ = source_path_.remove(source_path_.section("/",-2,-1));
        // QString com_ = "source " + path_base_ + "devel/setup." + shell_type_ + "\n";
        // com_ += "roslaunch calibration remove_ground.launch";
        // QTextStream str(&file);
        // str<<com_<<endl;
        // file.close();

        // start QProcess
        // QString process_control_ = "source " + sh_path_;
        // qDebug()<<process_control_;

        // add by Siyu Chen ,this method only need you
        // "sudo gedit ~/.zshrc" or "sudo gedit ~/.bashrc"
        // and add "source /devel/setup.bash"
        start_remove_ground_->start("roslaunch calibration remove_ground.launch");
    }
    else{
        if(start_remove_ground_->state() == QProcess::Running)
            // if process is still running ,then close this process
            start_remove_ground_->kill();   
    }
        
}

void PointCloud2Display::startObjectDetection(bool checked){
    if(checked){
      std::string topic_name_std = this->topic_name_.toStdString();
      Q_EMIT start_detect_node_(topic_name_std);
    }
    else
      Q_EMIT start_detect_node_("");
}

void PointCloud2Display::UpdatePointCloud2TopicName()
{   
    QStringList pointcloud2_topic_list;

    QString pointcloud2_topic_current = set_widget_->topic_getName_->currentText();
        
    if (pointcloud2_topic_current == "") {
        pointcloud2_topic_current = "----";
    }

    // Insert blank topic name to the top of the lists
    pointcloud2_topic_list << "----";

    // Get all available topic
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    // Analyse topics
    for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++) {
        const ros::master::TopicInfo &info = *it;
        const QString topic_name = QString::fromStdString(info.name);
        const QString topic_type = QString::fromStdString(info.datatype);

        // Check whether this topic is image
        if (topic_type.contains("sensor_msgs/PointCloud2") == true) {
            pointcloud2_topic_list << topic_name;
            continue;
        }
    }

        //remove all list items from combo box
        set_widget_->topic_getName_->clear();

        // set new items to combo box
        set_widget_->topic_getName_->addItems(pointcloud2_topic_list);
 
        set_widget_->topic_getName_->insertSeparator(1);

        // set last topic as current
        int pointcloud_topic_index = set_widget_->topic_getName_->findText(pointcloud2_topic_current);
     
        if (pointcloud_topic_index != -1) {
            set_widget_->topic_getName_->setCurrentIndex(pointcloud_topic_index);
        }

}

void PointCloud2Display::UpdateMarkerArrayTopicName()
{   
    QStringList markerarray_topic_list;

    QString markerarray_topic_current = marker_set_widget_->topic_name_get_->currentText();
        
    if (markerarray_topic_current == "") {
        markerarray_topic_current = "----";
    }

    // Insert blank topic name to the top of the lists
    markerarray_topic_list << "----";

    // Get all available topic
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    // Analyse topics
    for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++) {
        const ros::master::TopicInfo &info = *it;
        const QString topic_name = QString::fromStdString(info.name);
        const QString topic_type = QString::fromStdString(info.datatype);

        // Check whether this topic is image
        if (topic_type.contains("visualization_msgs/MarkerArray") == true) {
            markerarray_topic_list << topic_name;
            continue;
        }
    }

        //remove all list items from combo box
        marker_set_widget_->topic_name_get_->clear();

        // set new items to combo box
        marker_set_widget_->topic_name_get_->addItems(markerarray_topic_list);
 
        marker_set_widget_->topic_name_get_->insertSeparator(1);

        // set last topic as current
        int markerarray_topic_index = marker_set_widget_->topic_name_get_->findText(markerarray_topic_current);
     
        if (markerarray_topic_index != -1) {
            set_widget_->topic_getName_->setCurrentIndex(markerarray_topic_index);
        }

}

void PointCloud2Display::setFixedFrameName(const QString& fixed_frame_name_){
    this->frame_name_ = fixed_frame_name_;
    manager_->setFixedFrame(this->frame_name_);
    std::string frame_id_ = this->frame_name_.toStdString();
    Q_EMIT send_frame_id_(frame_id_);
}

void PointCloud2Display::setPointCloud2TopicName(const int& topic_name_index_){
    
    UpdatePointCloud2TopicName();
    QString topic_name_current_ = this->set_widget_->topic_getName_->itemText(topic_name_index_);
    this->topic_name_ = topic_name_current_;
    pointcloud2_->subProp("Topic")->setValue(this->topic_name_);
}

void PointCloud2Display::setMarkerArrayTopicName(const int& topic_name_index_){

    UpdateMarkerArrayTopicName();
    QString topic_name_current_ = this->marker_set_widget_->topic_name_get_->itemText(topic_name_index_);
    marker_array_->subProp("Marker Topic")->setValue(topic_name_current_);
}

//The event filter to catch clicking on combo box
bool PointCloud2Display::eventFilter(QObject* object, QEvent* event){
    if (event->type() == QEvent::MouseButtonPress) {
        // combo box will update its contents if this filter is applied
        UpdatePointCloud2TopicName();
    }
    return QObject::eventFilter(object, event);
}

void PointCloud2Display::setSelectable(const bool& selectable_){
    pointcloud2_->subProp("Selectable")->setValue(selectable_);
}

void PointCloud2Display::setSize(){
    QString size_para_ = set_widget_->size_get_->displayText();
    pointcloud2_->subProp("Size (m)")->setValue(size_para_);
}

void PointCloud2Display::setAlpha(const int& alpha_para_){
    pointcloud2_->subProp("Alpha")->setValue(alpha_para_ / 100.0f);
}

void PointCloud2Display::setDecayTime(const QString& decay_time_para_){
    pointcloud2_->subProp("Decay Time")->setValue(decay_time_para_);
}

void PointCloud2Display::setPositionTransform(const QString& position_transform_model_){
    pointcloud2_->subProp("Position Transformer")->setValue(position_transform_model_);
}

void PointCloud2Display::setColorTransform(const QString& color_transform_model_){
    pointcloud2_->subProp("Color Transformer")->setValue(color_transform_model_);
}

void PointCloud2Display::setMinColor(const QColor& min_color_){
    pointcloud2_->subProp("Min Color")->setValue(min_color_);
}

void PointCloud2Display::setMaxColor(const QColor& max_color_){
    pointcloud2_->subProp("Max Color")->setValue(max_color_);
}

}

