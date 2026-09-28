// 以下の記事を参考にした。
// ROS 講座 09 ROSメッセージ(と array 型の使い方)
// https://qiita.com/srs/items/080f1ca2ec2b2c480d41         

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

void flasksubCallback(const std_msgs::Float32MultiArray& array)
{
    ROS_INFO("subscribe: (size:%ld) %f %f %f ...", array.data.size(),
	     array.data[0], array.data[1], array.data[2]);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "flask_sub");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/prj4/flask", 10, flasksubCallback);

    ros::spin();

    return 0;
}
