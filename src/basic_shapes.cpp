#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include<Eigen/Dense>
//#include <rviz_visual_tools/rviz_visual_tools.h>

template<class Vector3>
std::pair<Vector3, Vector3> best_plane_from_points(const std::vector<Vector3> & c)
{
    // copy coordinates to  matrix in Eigen format
    size_t num_atoms = c.size();
    Eigen::Matrix<typename Vector3::Scalar, Eigen::Dynamic, Eigen::Dynamic > coord(3, num_atoms);
    for (size_t i = 0; i < num_atoms; ++i) coord.col(i) = c[i];

    // calculate centroid
    Vector3 centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

    // subtract centroid
    coord.row(0).array() -= centroid(0); coord.row(1).array() -= centroid(1); coord.row(2).array() -= centroid(2);

    // we only need the left-singular matrix here
    //  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
    auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    Vector3 plane_normal = svd.matrixU().rightCols(1);
    return std::make_pair(centroid, plane_normal);
}

int main( int argc, char** argv )
{
if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
   ros::console::notifyLoggerLevelsChanged();
}
  // rviz visual tools init
  //rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
  //visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base_frame","/rviz_visual_markers"));

  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  
  Eigen::Vector3f u1(0, 0, 1);
  Eigen::Vector3f u2(1, 0, 0);
  Eigen::Vector3f u3(0, 1, 0);
  Eigen::Vector3f pC, pNormal;
  std::pair<Eigen::Vector3f, Eigen::Vector3f> out;

  std::vector<Eigen::Vector3f> points;
  points.push_back(u1);
  points.push_back(u2);
  points.push_back(u3);
  
  out = best_plane_from_points(points);
  pC = out.first;
  pNormal = out.second;
//  Eigen::Quaternionf q(pNormal, u2);
      ROS_DEBUG_STREAM(u1 << std::endl << u2 << std::endl <<u3 << std::endl );
      ROS_DEBUG_STREAM(pC << std::endl << pNormal << std::endl) ;
  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    
 //   tf::Vector3 axis_vector;
  //  tf::vectorEigenToTF(pNormal, axis_vector);
   // tf::Vector3 up_vector(0.0, 0.0, 1.0);
//    tf::Vector3 right_vector = axis_vector.cross(up_vector);
 //   right_vector.normalized();
  //  tf::Quaternion tfQ;
   // tfQ.normalize();
    //geometry_msgs::Quaternion cylinder_orientation;
//    tf::quaternionTFToMsg(q, cylinder_orientation);
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = pC[0];
    marker.pose.position.y = pC[1];
    marker.pose.position.z = pC[2];
    //marker pose.orientation = tfQ;
    marker.pose.orientation.x = pC[0];
    marker.pose.orientation.y = pC[1];
    marker.pose.orientation.z = pC[2];
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 0.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);

    /* // Cycle between different shapes
    switch (shape)
    {
    case visualization_msgs::Marker::CUBE:
      shape = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      shape = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      shape = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      shape = visualization_msgs::Marker::CUBE;
      break;
    }
    */
    r.sleep();
  }
}
