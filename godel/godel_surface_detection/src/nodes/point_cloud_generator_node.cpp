#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <boost/make_shared.hpp>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

// parameters names
const std::string CLOUD_DESCRIPTIONS = "point_cloud_descriptions";
const std::string FRAME_ID = "frame_id";

// topics
const std::string POINT_CLOUD_TOPIC = "generated_cloud";

// constants

class GeneratePointCloud
{
  typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
  struct Description
  {
    tf::Vector3 size;
    tf::Transform transform;
    double resolution;
  };

public:
  GeneratePointCloud() {}

  bool init()
  {
    XmlRpc::XmlRpcValue list;
    ros::NodeHandle ph("~");

    // loading frame id
    if (!ph.getParam(FRAME_ID, frame_id_))
    {
      ROS_ERROR_STREAM("failed to load frame id");
    }

    // parameter numeric_fiels
    std::map<std::string, double> numeric_fields;
    numeric_fields.insert(std::make_pair("x", 0));
    numeric_fields.insert(std::make_pair("y", 0));
    numeric_fields.insert(std::make_pair("z", 0));
    numeric_fields.insert(std::make_pair("rx", 0));
    numeric_fields.insert(std::make_pair("ry", 0));
    numeric_fields.insert(std::make_pair("rz", 0));
    numeric_fields.insert(std::make_pair("l", 0));
    numeric_fields.insert(std::make_pair("w", 0));
    numeric_fields.insert(std::make_pair("h", 0));
    numeric_fields.insert(std::make_pair("resolution", 0));

    // loading point cloud descriptions
    if (ph.getParam(CLOUD_DESCRIPTIONS, list))
    {
      if (list.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        for (int j = 0; j < list.size(); j++)
        {

          XmlRpc::XmlRpcValue& entry = list[j];

          // parse numeric fields
          std::map<std::string, double>::iterator i;
          for (i = numeric_fields.begin(); i != numeric_fields.end(); i++)
          {
            if (entry.hasMember(i->first))
            {
              double val = static_cast<double>(entry[i->first]);
              numeric_fields[i->first] = val;
            }
            else
            {
              ROS_ERROR_STREAM("Point Cloud description entry is missing field '" << i->first
                                                                                  << "'");
              return false;
            }
          }

          // populating structure
          Description d;
          d.size = tf::Vector3(numeric_fields["l"], numeric_fields["w"], numeric_fields["h"]);
          d.transform = tf::Transform(
              tf::Quaternion(numeric_fields["ry"], numeric_fields["rx"], numeric_fields["rz"]),
              tf::Vector3(numeric_fields["x"], numeric_fields["y"], numeric_fields["z"]));
          d.resolution = numeric_fields["resolution"];

          cloud_descriptions_.push_back(d);
        }
      }
    }

    return true;
  }

  void run()
  {
    ros::NodeHandle nh;
    ros::Publisher cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>(POINT_CLOUD_TOPIC, 1);

    if (init()) {

      full_cloud_.clear();
      full_cloud_.header.frame_id = frame_id_;

      // ORIGINAL GODEL CODE : genenerate_cloud();
	  
      ros::NodeHandle nh("/");
	    nh.setParam("make3DScreenshot",false); // Define the parameter for the first time

	    bool make3DScreenshot=false;
	  
      sensor_msgs::PointCloud2 msg;
      pcl::toROSMsg(full_cloud_, msg);

      ros::Duration loop_duration(0.4f);
      while (ros::ok()) {
		 
        nh.getParam("make3DScreenshot",make3DScreenshot);
      
        if ( make3DScreenshot ) {
          
          make_screenshot(); // Make a 3D screenshot A.K.A. read data 3D-camera

          pcl::toROSMsg(full_cloud_, msg); // Create new message with the new added pointcloud data
          
          nh.setParam("make3DScreenshot",false); // Reset parameter
        }

        msg.header.stamp = ros::Time::now() - loop_duration;
        cloud_publisher.publish(msg);
        loop_duration.sleep();

      }
    }
  }

protected:  
  void make_screenshot()
  {
    full_cloud_.clear(); // Prevent that multiple 3D screenshots/clouds merge with eachother

    sensor_msgs::PointCloud2ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
        "/camera/depth/points", ros::Duration(20.0f));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

    
    // Box-cutting project : Convert message to PointCloud2
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg,pcl_pc2);

    // Box-cutting project : Convert cloud_ptr (class PointCloud2) to PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    // Transform PointCloud to the frame of the 3D-camera :
    tf::Transform transformationMatrix = tf::Transform::getIdentity();

    tf::Transform transformation1 = tf::Transform::getIdentity();
    transformation1.setRotation(tf::Quaternion(tf::Vector3(0, 0, 1), -3.14159265359));

    tf::Transform transformation2 = tf::Transform::getIdentity();
    transformation2.setRotation(tf::Quaternion(tf::Vector3(0, 1, 0), 3.14159265359));

    tf::Transform transformation3 = tf::Transform::getIdentity();
    transformation3.setOrigin(tf::Vector3(0.0, 0.55, 0.8));

    tf::Transform transformation4 = tf::Transform::getIdentity();
    transformation4.setRotation(tf::Quaternion(tf::Vector3(0, 0, 1), -3.14159265359/2.0));

    transformationMatrix = transformation4 * transformation3 * transformation2 * transformation1;

    // Make the transforming actually happen
    Eigen::Affine3d eigen3d;
    tf::transformTFToEigen(transformationMatrix, eigen3d);
    pcl::transformPointCloud(*temp_cloud, *temp_cloud, Eigen::Affine3f(eigen3d));

    full_cloud_ += *temp_cloud;

    full_cloud_.header.frame_id = frame_id_;
  }


  void genenerate_cloud()
  {
    full_cloud_.clear();

    ///* OORSPRONKELIJK :
    for (unsigned int i = 0; i < cloud_descriptions_.size(); i++)
    {
      Cloud box;
      Description& desc = cloud_descriptions_[i];
      create_box(desc, box);

      // transforming box
      Eigen::Affine3d eigen3d;
      tf::transformTFToEigen(desc.transform, eigen3d);
      pcl::transformPointCloud(box, box, Eigen::Affine3f(eigen3d));

      // concatenating box
      full_cloud_ += box;
    }
    //*/


    /* DIT WERKT (MAAR EVENTJES UITGESCHAKELD)
	ALLEEN MOET DE POINTCLOUD NOG GOED GETRANSFORMEERD WORDEN NAAR DE ECHTE WERELD!
    sensor_msgs::PointCloud2ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
        "/camera/depth/points", ros::Duration(5.0f));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    full_cloud_ += *temp_cloud;
    */

    full_cloud_.header.frame_id = frame_id_;
  }

  void create_rectangular_patch(tf::Vector3 start, tf::Vector3 end, double res, Cloud& patch)
  {
    int count_x = (end.x() - start.x()) / res + 1;
    int count_y = (end.y() - start.y()) / res + 1;

    pcl::PointXYZ p;
    p.z = 0;
    patch.resize(count_x * count_y);
    int counter = 0;
    for (int i = 0; i < count_x; i++)
    {
      p.x = start.x() + res * i;
      for (int j = 0; j < count_y; j++)
      {
        p.y = start.y() + res * j;
        patch.points[counter] = p;
        counter++;
      }
    }
  }

  void create_box(const Description& desc, Cloud& box_points)
  {
    // face points
    Cloud top, bottom, front, rear, left, right, temp;
    tf::Vector3 start, end;

    // transforms
    tf::Transform t;
    Eigen::Affine3d eigen3d;

    // ================================ create top and bottom patches
    // ===============================
    start = tf::Vector3(-desc.size.x() / 2, -desc.size.y() / 2, 0);
    end = tf::Vector3(desc.size.x() / 2, desc.size.y() / 2, 0);
    create_rectangular_patch(start, end, desc.resolution, temp);

    // transform cloud to top
    t = tf::Transform(tf::Quaternion::getIdentity(), tf::Vector3(0, 0, 0.5f * desc.size.z()));
    tf::transformTFToEigen(t, eigen3d);
    pcl::transformPointCloud(temp, top, Eigen::Affine3f(eigen3d));

    // transform cloud to bottom
    t = tf::Transform(tf::Quaternion::getIdentity(), tf::Vector3(0, 0, 0.5f * -desc.size.z()));
    tf::transformTFToEigen(t, eigen3d);
    pcl::transformPointCloud(temp, bottom, Eigen::Affine3f(eigen3d));

    // concatenate
    box_points += top;
    box_points += bottom;

    // ================================ create front and rear patches
    // ===============================
    start = tf::Vector3(-desc.size.x() / 2, -desc.size.z() / 2, 0);
    end = tf::Vector3(desc.size.x() / 2, desc.size.z() / 2, 0);
    create_rectangular_patch(start, end, desc.resolution, temp);

    // transform cloud to front
    t = tf::Transform(tf::Quaternion(tf::Vector3(1, 0, 0), M_PI_2),
                      tf::Vector3(0, 0.5f * desc.size.y(), 0));
    tf::transformTFToEigen(t, eigen3d);
    pcl::transformPointCloud(temp, front, Eigen::Affine3f(eigen3d));

    // transform cloud to rear
    t = tf::Transform(tf::Quaternion(tf::Vector3(1, 0, 0), M_PI_2),
                      tf::Vector3(0, 0.5f * -desc.size.y(), 0));
    tf::transformTFToEigen(t, eigen3d);
    pcl::transformPointCloud(temp, rear, Eigen::Affine3f(eigen3d));

    box_points += front;
    box_points += rear;

    // ================================ create left and right patches
    // ===============================
    start = tf::Vector3(-desc.size.z() / 2, -desc.size.y() / 2, 0);
    end = tf::Vector3(desc.size.z() / 2, desc.size.y() / 2, 0);
    create_rectangular_patch(start, end, desc.resolution, temp);

    // transform cloud to left
    t = tf::Transform(tf::Quaternion(tf::Vector3(0, 1, 0), M_PI_2),
                      tf::Vector3(0.5f * desc.size.x(), 0, 0));
    tf::transformTFToEigen(t, eigen3d);
    pcl::transformPointCloud(temp, left, Eigen::Affine3f(eigen3d));

    // transform cloud to right
    t = tf::Transform(tf::Quaternion(tf::Vector3(0, 1, 0), M_PI_2),
                      tf::Vector3(0.5f * -desc.size.x(), 0, 0));
    tf::transformTFToEigen(t, eigen3d);
    pcl::transformPointCloud(temp, right, Eigen::Affine3f(eigen3d));

    box_points += left;
    box_points += right;
  }

protected:
  std::vector<Description> cloud_descriptions_;
  std::string frame_id_;
  Cloud full_cloud_;
  float resolution_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "generate_point_cloud_node");

  GeneratePointCloud g;
  g.run();
  return 0;
}
