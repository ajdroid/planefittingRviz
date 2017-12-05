#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>
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
  int num_points = 3;
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher plane_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Publisher points_pub = n.advertise<visualization_msgs::Marker>("point_marker_list", num_points);

  // Set our initial shape type to be a cube
  
  // Set points to use for plane fitting
  Eigen::Vector3d u1(0, 0, 1);
  Eigen::Vector3d u2(1, 0, 0);
  Eigen::Vector3d u3(0, 1, 0);
  Eigen::Vector3d pC, pNormal;
  std::pair<Eigen::Vector3d, Eigen::Vector3d> out;

  std::vector<Eigen::Vector3d> points;
  points.push_back(u1);
  points.push_back(u2);
  points.push_back(u3);
  
  // Fit plane to points
  out = best_plane_from_points(points);
  pC = out.first;
  pNormal = out.second;

  // Printing normals etc.
  ROS_DEBUG_STREAM(u1 << std::endl << u2 << std::endl <<u3 << std::endl );
  ROS_DEBUG_STREAM(pC << std::endl << pNormal << std::endl);

  while (ros::ok())
  {
    visualization_msgs::Marker plane_marker, sphere_list;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    plane_marker.header.frame_id = "/map";
    plane_marker.header.stamp = ros::Time::now();
    sphere_list.header.frame_id= "/map";
    sphere_list.header.stamp= ros::Time::now();


    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    plane_marker.ns = "basic_shapes";
    plane_marker.id = 0;
    sphere_list.ns= "basic_shapes";
    sphere_list.id = 1;


    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    plane_marker.type = visualization_msgs::Marker::CUBE;
    sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    plane_marker.action = visualization_msgs::Marker::ADD;
    sphere_list.action= visualization_msgs::Marker::ADD;
    sphere_list.pose.orientation.w= 1.0;


    // convert from axis vector to quaternion    
    tf::Vector3 axis_vector;
    tf::vectorEigenToTF(pNormal, axis_vector);
    tf::Vector3 up_vector(0.0, 0.0, 1.0);
    tf::Vector3 right_vector = axis_vector.cross(up_vector);
    right_vector.normalized();
    tf::Quaternion tfQ(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
    tfQ.normalize();
    geometry_msgs::Quaternion cube_orientation;
    tf::quaternionTFToMsg(tfQ, cube_orientation);

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    plane_marker.pose.position.x = pC[0];
    plane_marker.pose.position.y = pC[1];
    plane_marker.pose.position.z = pC[2];
    
    plane_marker.pose.orientation = cube_orientation;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    plane_marker.scale.x = 1.0;
    plane_marker.scale.y = 1.0;
    plane_marker.scale.z = 0.0;
    // POINTS markers use x and y scale for width/height respectively
    sphere_list.scale.x = 0.1;
    sphere_list.scale.y = 0.1;
    sphere_list.scale.z = 0.1;

    // Set the color -- opaque red plane, opaque white points
    plane_marker.color.r = 1.0f;
    plane_marker.color.g = 0.0f;
    plane_marker.color.b = 0.0f;
    plane_marker.color.a = 1.0;
    sphere_list.color.r = 1.0f;
    sphere_list.color.g = 1.0f;
    sphere_list.color.b = 1.0f;
    sphere_list.color.a = 1.0;

    // Set sphere centres
    geometry_msgs::Point pt;
    tf::pointEigenToMsg(u1, pt);
    sphere_list.points.push_back(pt);
    tf::pointEigenToMsg(u2, pt);
    sphere_list.points.push_back(pt);
    tf::pointEigenToMsg(u3, pt);
    sphere_list.points.push_back(pt);
  
    plane_marker.lifetime = ros::Duration();

    // Publish the marker
    while (plane_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    plane_pub.publish(plane_marker);
    points_pub.publish(sphere_list);

    r.sleep();
  }
}
