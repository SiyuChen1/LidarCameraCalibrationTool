#ifndef CALIBBOARDDETECTOR_H
#define CALIBBOARDDETECTOR_H

//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/surface/concave_hull.h>
#include <pcl/common/intersections.h>
#include <pcl/common/centroid.h>

//ros
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>


#include <QObject>
#include <QDebug>
// qt package

using namespace std;

namespace PointCloud2
{

class CalibBoardDetector
{
public:

    typedef pcl::PointXYZI Point;
    typedef pcl::PointCloud<Point> PointCloud;
    typedef std::vector<PointCloud> PointCloudArray;

    CalibBoardDetector();

    ~CalibBoardDetector();

    void detect(const PointCloud& cloud);

    PointCloudArray& get_vertex_array();

    PointCloudArray& get_cloud_clustered();

private:

    void _height_filter(PointCloud& cloud);

    void _down_sample(PointCloud& cloud);

    std::vector<PointCloud> _euclidean_cluster(const PointCloud& cloud);

    bool _ransac_plane_fitting(const PointCloud& cloud,
                               pcl::ModelCoefficients::Ptr coeff,
                               pcl::PointIndices::Ptr inliers);

    PointCloud _project_cloud(const PointCloud& cloud,
                              const pcl::ModelCoefficients::Ptr coeff,
                              const pcl::PointIndices::Ptr inliers);

    PointCloud _find_contour(const PointCloud& cloud);

    void _edge_lines_fitting(PointCloud& cloud,
                             PointCloudArray& line_vec,
                             std::vector<pcl::ModelCoefficients>& coeff_vec);

    PointCloud _find_intersection_points(const std::vector<pcl::ModelCoefficients>& coeff_array);

    void _sort_vertex(PointCloud& vertex_points);

    void _sort_vertex_array(PointCloudArray& vertex_array_);

    double _cal_bearing(const Point& pt);

    double _point_cross(const Point &a, const Point &b);

    static bool _point_bearing_cmp(const Point &a, const Point &b);

    static bool _cloud_bearing_cmp(const PointCloud& cloud_a, const PointCloud& cloud_b);

    static bool _point_height_cmp(const Point &a, const Point &b){
        return a.z < b.z;
    }

private:
    PointCloud m_cloud_src_;
    PointCloudArray m_cloud_cluster_array_;

    PointCloudArray m_vertex_array_;

};

class CalibBoardDetectionNode : public QObject
{

Q_OBJECT

public:
    CalibBoardDetectionNode(ros::NodeHandle& nh,QObject* parent = nullptr);

    ~CalibBoardDetectionNode();

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_in);

    void generate_marker_array(visualization_msgs::MarkerArray& marker_array,
                               const CalibBoardDetector::PointCloudArray& vertex_pt_array);

    pcl::PointCloud<pcl::PointXYZRGB> generate_colored_cloud(const CalibBoardDetector::PointCloudArray& cloud_array);

public Q_SLOTS:

    void run(const std::string& topic_name_);

    void set_frame_id(const std::string& frame_id_);

private:
    ros::NodeHandle& m_nh_;
    ros::Subscriber m_sub_cloud_;
    ros::Publisher m_pub_marker_;
    ros::Publisher m_pub_cloud_;

    CalibBoardDetector m_detector_;

    std::string frame_id_;

};

}

#endif //PROJECT_CALIBBOARDDETECTOR_H
