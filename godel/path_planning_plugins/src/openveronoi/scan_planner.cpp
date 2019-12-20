#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <godel_msgs/PathPlanning.h>
#include <mesh_importer/mesh_importer.h>
#include <path_planning_plugins/openveronoi_plugins.h>
#include <pluginlib/class_list_macros.h>
#include <profilometer/profilometer_scan.h>
#include <ros/node_handle.h>
#include <tf/transform_datatypes.h>



#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>



#include <ros/ros.h>
#include <boost/bimap.hpp>
#include <pcl/geometry/triangle_mesh.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/filters/project_inliers.h>
#include <mesh_importer/mesh_importer.h>
#include "godel_process_path_generation/get_boundary.h"



#include <ros/ros.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <geometry_msgs/PolygonStamped.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include "godel_process_path_generation/polygon_pts.hpp"
#include "godel_process_path_generation/polygon_utils.h"
#include "godel_process_path_generation/utils.h"

#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"

// Together these constants define a 5cm approach and departure path for the laser scans
const static int SCAN_APPROACH_STEP_COUNT = 5;
const static double SCAN_APPROACH_STEP_DISTANCE = 0.01; // 1cm

namespace path_planning_plugins
{
// Scan Planner

void openveronoi::ScanPlanner::init(pcl::PolygonMesh mesh)
{
  mesh_ = mesh;
}




double openveronoi::ScanPlanner::calculateDistance(double x2, double x1, double y2, double y1) { 
    // Calculating distance 
    return sqrt(pow(x2-x1, 2) + pow(y2-y1, 2)); 
}


float openveronoi::ScanPlanner::calculateAngleCorrection (std::unique_ptr<mesh_importer::MeshImporter>& mesh_importer_ptr, const pcl::PolygonMesh& input_mesh) {

  // Zet PointCloud2 om naar PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(input_mesh.cloud, *points);
  pcl::transformPointCloud(*points, *points, mesh_importer_ptr->plane_frame_.inverse());

  double mostLeftDownX = 0.0;
  double mostLeftDownY = 0.0;
  double mostLeftUpX = 0.0;
  double mostLeftUpY = 0.0;
  double mostRightDownX = 0.0;
  double mostRightDownY = 0.0;
  double mostRightUpX = 0.0;
  double mostRightUpY = 0.0;

  double averageX = 0.0;
  double averageY = 0.0;

  for (int i = 0; i < points->points.size(); ++i) {
    averageX += points->points[i].x;
    averageY += points->points[i].y;
  }

  averageX /= (double)points->points.size();
  averageY /= (double)points->points.size();


  for (int i = 0; i < points->points.size(); ++i) {
    double x = points->points[i].x - averageX;
    double y = points->points[i].y - averageY;

    double distance = calculateDistance(x,0,y,0);

    if ( x < 0 && y < 0 && distance > calculateDistance(mostLeftDownX,0,mostLeftDownY,0) ) {
      mostLeftDownX = x;
      mostLeftDownY = y;
    }
    if ( x > 0 && y < 0 && distance > calculateDistance(mostRightDownX,0,mostRightDownY,0) ) {
      mostRightDownX = x;
      mostRightDownY = y;
    }
    if ( x < 0 && y > 0 && distance > calculateDistance(mostLeftUpX,0,mostLeftUpY,0) ) {
      mostLeftUpX = x;
      mostLeftUpY = y;
    }
    if ( x > 0 && y > 0 && distance > calculateDistance(mostRightUpX,0,mostRightUpY,0) ) {
      mostRightUpX = x;
      mostRightUpY = y;
    }
  }

  float angle = atan2(mostLeftDownY - mostLeftUpY, mostLeftDownX - mostLeftUpX) * 180.0 / 3.14159265359 + 90.0;

  return 3.14159265359/180.0*angle;
}



bool openveronoi::ScanPlanner::generatePath(std::vector<geometry_msgs::PoseArray>& path)
{
  using godel_process_path::PolygonBoundaryCollection;
  using godel_process_path::PolygonBoundary;

  path.clear();

  std::unique_ptr<mesh_importer::MeshImporter> mesh_importer_ptr(new mesh_importer::MeshImporter(false));

  ros::NodeHandle nh;
  godel_msgs::PathPlanningParameters params;
  try
  {
    nh.getParam(DISCRETIZATION, params.discretization);
    nh.getParam(MARGIN, params.margin);
    nh.getParam(OVERLAP, params.overlap);
    nh.getParam(SAFE_TRAVERSE_HEIGHT, params.traverse_height);
    nh.getParam(SCAN_WIDTH, params.scan_width);
    nh.getParam(TOOL_RADIUS, params.tool_radius);
  }
  catch(const std::exception& e)
  {
    ROS_ERROR_STREAM("Unable to populate path planning parameters" << e.what());
    return false;
  }

  // 0 - Calculate boundaries for a surface
  if (mesh_importer_ptr->calculateSimpleBoundary(mesh_))
  {
    // 1 - Read & filter boundaries that are ill-formed or too small
    PolygonBoundaryCollection filtered_boundaries = filterPolygonBoundaries(mesh_importer_ptr->getBoundaries());

    // 2 - Read boundary pose
    geometry_msgs::Pose boundary_pose;
    mesh_importer_ptr->getPose(boundary_pose);

    // 3 - Skip if boundaries are empty
    if (filtered_boundaries.empty())
      return false;

    geometry_msgs::PoseArray scan_poses;

    // 4 - Generate scan polygon boundary
    PolygonBoundary scan_boundary = scan::generateProfilometerScanPath(filtered_boundaries.front(), params);

    // 5 - Get boundary pose eigen
    Eigen::Affine3d boundary_pose_eigen;
    tf::poseMsgToEigen(boundary_pose, boundary_pose_eigen);

    // 6 - Transform points to world frame and generate pose
    std::vector<geometry_msgs::Point> points;
    std::vector<geometry_msgs::Point> points2;

    float angleCorrection = calculateAngleCorrection(mesh_importer_ptr,mesh_);
    nh.setParam("angleCorrection",angleCorrection);

    for(const auto& pt : scan_boundary)
    {
      geometry_msgs::Point p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = 0.0;

      points.push_back(p);
    }

    float beginCoordinatesX = points[0].x*(5.0/6.0);
    float beginCoordinatesY = points[0].y;

    float endCoordinatesX = points[points.size()-1].x*(5.0/6.0);
    float endCoordinatesY = points[points.size()-1].y;

    float distance = 0.05;

    geometry_msgs::Point p11;
    p11.x = points[points.size()-1].x;
    p11.y = points[points.size()-1].y;
    p11.z = 0.05;
    points.push_back(p11);

    geometry_msgs::Point p12;
    p12.x = beginCoordinatesX;
    p12.y = beginCoordinatesY-distance;
    p12.z = 0.05;
    points2.push_back(p12);


    for ( float y = -distance; y <= +distance; y+=SCAN_APPROACH_STEP_DISTANCE) {
      geometry_msgs::Point p;
      p.x = beginCoordinatesX;
      p.y = beginCoordinatesY+y;
      p.z = 0.0;

      points2.push_back(p);
    }

    geometry_msgs::Point p21;
    p21.x = points2[points2.size()-1].x;
    p21.y = points2[points2.size()-1].y;
    p21.z = 0.05;
    points2.push_back(p21);

    geometry_msgs::Point p22;
    p22.x = endCoordinatesX;
    p22.y = endCoordinatesY-distance;
    p22.z = 0.05;
    points2.push_back(p22);


    for ( float y = -distance; y <= +distance; y+=SCAN_APPROACH_STEP_DISTANCE) {
      geometry_msgs::Point p;
      p.x = endCoordinatesX;
      p.y = endCoordinatesY+y;
      p.z = 0.0;

      points2.push_back(p);
    }

    geometry_msgs::Point p23;
    p23.x = points2[points2.size()-1].x;
    p23.y = points2[points2.size()-1].y;
    p23.z = 0.05;
    points2.push_back(p23);







    for ( int i = 0; i < points.size(); i++ ) {

      double radius  = sqrt(pow(points[i].x, 2) + pow(points[i].y, 2));
      float  angle   = atan2(points[i].y, points[i].x)+angleCorrection;

      points[i].x = radius * cos(angle);
      points[i].y = radius * sin(angle);

    }


    for ( int i = 0; i < points2.size(); i++ ) {

      double radius = sqrt(pow(points2[i].x, 2) + pow(points2[i].y, 2));
      float  angle  = atan2(points2[i].y, points2[i].x)+angleCorrection;

      points2[i].x = radius * cos(angle);
      points2[i].y = radius * sin(angle);

    }

    std::transform(points.begin(), points.end(), std::back_inserter(scan_poses.poses),
                   [boundary_pose_eigen] (const geometry_msgs::Point& point) {
      geometry_msgs::Pose pose;
      Eigen::Affine3d r = boundary_pose_eigen * Eigen::Translation3d(point.x, point.y, point.z);
      tf::poseEigenToMsg(r, pose);

      ros::NodeHandle nh;
      float angleCorrection = 0.0;
      nh.getParam("angleCorrection",angleCorrection);

      // Convert roll, pitch and yaw to Quaternion :
      float roll2 = 1.5707, pitch2 = 0, yaw2 = 0;

      ///* GET THE EULER ANGLES AND PUBLISH THEM
      tf::Quaternion q_original(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
      tf::Matrix3x3 m(q_original);

      double roll, pitch, yaw;
      tf::Matrix3x3(q_original).getRPY(roll, pitch, yaw);

      yaw2 = yaw+angleCorrection;


      pose.orientation =  tf::createQuaternionMsgFromRollPitchYaw(roll2,pitch2,yaw2);


      return pose;
    });


    std::transform(points2.begin(), points2.end(), std::back_inserter(scan_poses.poses),
                   [boundary_pose_eigen] (const geometry_msgs::Point& point) {
      geometry_msgs::Pose pose;
      Eigen::Affine3d r = boundary_pose_eigen * Eigen::Translation3d(point.x, point.y, point.z);
      tf::poseEigenToMsg(r, pose);

      ros::NodeHandle nh;
      float angleCorrection = 0.0;
      nh.getParam("angleCorrection",angleCorrection);

      // Convert roll, pitch and yaw to Quaternion :
      float roll2 = 1.5707, pitch2 = 0, yaw2 = 0;

      ///* GET THE EULER ANGLES AND PUBLISH THEM
      tf::Quaternion q_original(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
      tf::Matrix3x3 m(q_original);

      double roll, pitch, yaw;
      tf::Matrix3x3(q_original).getRPY(roll, pitch, yaw);

      yaw2 = yaw+angleCorrection-1.5707;

      pose.orientation =  tf::createQuaternionMsgFromRollPitchYaw(roll2,pitch2,yaw2);

      return pose;
    });

    // 8 - return result
    path.push_back(scan_poses);
    return true;
  }
  else
    ROS_WARN_STREAM("Could not calculate boundary for mesh");
  return false;
}
} // end namespace path_planning_plugins

PLUGINLIB_EXPORT_CLASS(path_planning_plugins::openveronoi::ScanPlanner, path_planning_plugins_base::PathPlanningBase)
