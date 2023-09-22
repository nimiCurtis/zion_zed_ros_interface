// #include <ros/ros.h>
// #include <tf/LinearMath/Quaternion.h>
// #include <tf/transform_broadcaster.h>
// #include <tf/transform_listener.h>

// #include <geometry_msgs/TransformStamped.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Pose.h>


// class CloudTransformer
// {
// public:
//   explicit CloudTransformer(ros::NodeHandle nh)
//     : nh_(nh)
//   {
//     // Define Publishers and Subscribers here
//     pcl_sub_ = nh_.subscribe("/zedm/zed_node/pose", 1, &CloudTransformer::poseCallback, this);

//   }

// private:
//   ros::NodeHandle nh_;
//   ros::Subscriber pcl_sub_;
//   ros::Publisher pcl_pub_;
//   tf::TransformListener listener_;

// void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg){
//     static tf::TransformBroadcaster br;
//     geometry_msgs::TransformStamped transformStamped;
//     tf::StampedTransform transform;
//     listener_.lookupTransform("map", "zedm_left_camera_frame", ros::Time(0), transform);
//   //   listener_.lookupTransform("zedm_left_camera_optical_frame", "zedm_left_camera_frame", ros::Time(0), transform);

//     tf::Matrix3x3 rotation_matrix(transform.getRotation());
//     double roll, pitch, yaw;
//     rotation_matrix.getRPY(roll, pitch, yaw); //(xaxis,zaxis,yaxis)

//     transformStamped.header.stamp = ros::Time::now();
//     transformStamped.header.frame_id = "zedm_left_camera_frame";
//     transformStamped.child_frame_id = "zedm_left_camera_frame_projected";
//     transformStamped.transform.translation.x = 0;
//     transformStamped.transform.translation.y = 0;
//     transformStamped.transform.translation.z = 0.0;
//     tf::Quaternion q;


//     q.setRPY(-roll, -pitch, 0);
//     transformStamped.transform.rotation.x = q.x();
//     transformStamped.transform.rotation.y = q.y();
//     transformStamped.transform.rotation.z = q.z();
//     transformStamped.transform.rotation.w = q.w();

//     br.sendTransform(transformStamped);
//   }
// };  // End of class CloudTransformer

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "point_cloud_tf");
//   ros::NodeHandle nh;

//   CloudTransformer tranform_cloud(nh);

//   // Spin until ROS is shutdown
//   while (ros::ok())
//     ros::spin();

//   return 0;
// }





































// // class ZionTransformBroad
// // {
// // public:
// //   explicit class ZionTransformBroad(ros::NodeHandle nh) : nh_(nh)
// //   {
// //     // Define Publishers and Subscribers here
// //     pcl_sub_ = nh_.subscribe("/zedm/zed_node/pose", 1, &ZionTransformBroad::poseCallback,this);
// //   }

// // private:
// //   ros::NodeHandle nh_;
// //   ros::Subscriber pcl_sub_;
// //   ros::Publisher pcl_pub_;
// //   tf2_ros::TransformListener listener_;

// //   void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg){
// //     static tf2_ros::TransformBroadcaster br;
// //     geometry_msgs::TransformStamped transformStamped;
    
// //     transformStamped.header.stamp = ros::Time::now();
// //     transformStamped.header.frame_id = "zedm_left_camera_frame";
// //     transformStamped.child_frame_id = "zedm_left_camera_frame_projected";
// //     transformStamped.transform.translation.x = 0;
// //     transformStamped.transform.translation.y = 0;
// //     transformStamped.transform.translation.z = 0.0;
// //     tf2::Quaternion q;


// //     q.setRPY(0, 0, 0);
// //     transformStamped.transform.rotation.x = q.x();
// //     transformStamped.transform.rotation.y = q.y();
// //     transformStamped.transform.rotation.z = q.z();
// //     transformStamped.transform.rotation.w = q.w();

// //     br.sendTransform(transformStamped);
// //   }



// // };  // End of class CloudTransformer

// // int main(int argc, char **argv)
// // {
// //   ros::init(argc, argv, "point_cloud_tf");
// //   ros::NodeHandle nh;

  
// //   ZionTransformBroad tranform_cloud(nh);

// //   // Spin until ROS is shutdown
// //   while (ros::ok())
// //     ros::spin();

// //   return 0;
// // }

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

class CloudTransformer
{
public:
  explicit CloudTransformer(ros::NodeHandle nh)
    : nh_(nh), tf_listener_(tf_buffer_)
  {
    pcl_sub_ = nh_.subscribe("/tf", 1, &CloudTransformer::tfCallback, this);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber pcl_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  void tfCallback(const tf2_msgs::TFMessage& msg)
  {
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    try
    {
      // transformStamped = tf_buffer_.lookupTransform("map", "zedm_left_camera_frame", ros::Time(0));
      transformStamped = tf_buffer_.lookupTransform("map", "zedm_base_link", ros::Time(0));

    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("Transform exception: %s", ex.what());
      return;
    }

    // Extract the rotation from existing transform
    tf2::Transform tf2_transform;
    tf2::fromMsg(transformStamped.transform, tf2_transform);
    tf2::Vector3 translation(tf2_transform.getOrigin());
    tf2::Matrix3x3 rotation_matrix(tf2_transform.getRotation());
    double roll, pitch, yaw;
    rotation_matrix.getRPY(roll, pitch, yaw);
    



    // Create a new transform to broadcast
    geometry_msgs::TransformStamped new_transformStamped;
    new_transformStamped.header.stamp = ros::Time::now();
    new_transformStamped.header.frame_id = "map";
    // new_transformStamped.child_frame_id = "zedm_left_camera_frame_projected";
    new_transformStamped.child_frame_id = "zedm_base_link_projected";

    // new_transformStamped.header.frame_id = "zedm_left_camera_frame";
    // new_transformStamped.child_frame_id = "zedm_left_camera_frame_projected";

    // new_transformStamped.transform.translation.x = 0;
    // new_transformStamped.transform.translation.y = 0;
    // new_transformStamped.transform.translation.z = 0;

    // Set rotation (negating roll and pitch)
    tf2::Quaternion new_rotation;
    // new_rotation.setRPY(-roll, -pitch, 0);
    new_rotation.setRPY(0, 0, yaw);

    new_transformStamped.transform.translation = tf2::toMsg(translation);
    new_transformStamped.transform.rotation = tf2::toMsg(new_rotation);

    br.sendTransform(new_transformStamped);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_tf");
  ros::NodeHandle nh;

  CloudTransformer transform_cloud(nh);

  while (ros::ok())
    ros::spin();

  return 0;
}

