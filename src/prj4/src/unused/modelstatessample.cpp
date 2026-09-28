#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

// Messages for getting model and link poses
geometry_msgs::Pose ball_model_pose, ball_link_pose;

// Service client for setting model poses
ros::ServiceClient set_model_state_client;

int getIndex(std::vector<std::string> v, std::string value)
{
    for(int i = 0; i < v.size(); i++) {
        if(v[i].compare(value) == 0) return i;
    }
    return -1;
}

void model_states_callback(gazebo_msgs::ModelStates model_states)
{
    int ball_model_index = getIndex(model_states.name, "ball");
    ball_model_pose = model_states.pose[ball_model_index];
}

void link_states_callback(gazebo_msgs::LinkStates link_states)
{
    int ball_link_index = getIndex(link_states.name, "ball::body");
    ball_link_pose = link_states.pose[ball_link_index];
}

void set_model_state(std::string model_name, std::string reference_frame, 
                    geometry_msgs::Pose pose, geometry_msgs::Twist model_twist)
{
    // Set model service struct
    gazebo_msgs::SetModelState setmodelstate;

    // Model state msg
    gazebo_msgs::ModelState modelstate;
    modelstate.model_name = model_name;
    modelstate.reference_frame = reference_frame;
    modelstate.pose = pose;
    modelstate.twist = model_twist;

    setmodelstate.request.model_state = modelstate;

    // Call the service
    bool success = set_model_state_client.call(setmodelstate);
    if (success) {
        ROS_INFO_STREAM( "Setting position of " << model_name << "model was successful.");
    }
    else {
        ROS_ERROR_STREAM("Setting position of " << model_name << "model was failed.");
    }
}

int main()
{
    // Create ROS node
    ros::init(argc, argv, "model_states_handler");
    ros::NodeHandle n;
  
    // Create suvscribers for Gazebo model and link states
    ros::Subscriber model_states_subscriber = n.subscribe("/gazebo/model_states", 100,
							  model_states_callback);
    ros::Subscriber link_states_subscriber = n.subscribe("/gazebo/link_states", 100,
							 link_states_callback);
  
    // Create service client for setting Gazebo model state
    set_model_state_client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  
    // Pose and velocity for model state setting
    geometry_msgs::Pose model_pose;
    model_pose.position.x = 0;
    model_pose.position.y = 0;
    model_pose.position.z = 1;
    model_pose.orientation.x = 0.0;
    model_pose.orientation.y = 0.0;
    model_pose.orientation.z = 0.0;
    model_pose.orientation.w = 0.0;

    geometry_msgs::Twist model_twist;
    model_twist.linear.x = 0.0;
    model_twist.linear.y = 0.0;
    model_twist.linear.z = 0.0;
    model_twist.angular.x = 0.0;
    model_twist.angular.y = 0.0;
    model_twist.angular.z = 0.0;
  
    // Set model state in Gazebo
    set_model_state("ball", "world", model_pose, model_twist);
  
    return 0;
}
