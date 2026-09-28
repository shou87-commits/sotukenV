// pose2class.h  Pose, 姿勢データ読出し用

#ifndef HEADER_POSECLASS
#define HEADER_POSECLASS

#include <geometry_msgs/PoseStamped.h>

#include "const.h"

class POSE {
private:
    ros::NodeHandle nh;
    ros::Subscriber pose_sub;
    const char *topicname; // トピック名
public:
    int id; // 2個以上扱う場合の物体の番号
    float pose[XYZ] = {0.123}; // XYZ 3次元の値が記録される

public:
    POSE(int objid, const char tn[])
    {
    	id = objid; 
    	topicname = tn;  // トピック名
    	pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(topicname, 1, &POSE::PoseCallback, this);
    }

    ~POSE(){}
	
    void PoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
    {
	int sec = msg -> header.seq; // 時刻、もともとあるseqを利用する

	pose[0] = msg -> wrench.force.x;
	pose[1] = msg -> wrench.force.y;
	pose[2] = msg -> wrench.force.z;

	if (0 == msg -> header.seq % 100){	//１秒ごとに表示
	    ROS_INFO("f x:%.3f y:%.3f z:%.3f", msg->wrench.force.x, msg->wrench.force.y,
	                                       msg->wrench.force.z);
	    ROS_INFO("f x:%.3f y:%.3f z:%.3f", pose[0], pose[1], pose[2]); //入ってる値確認
    	}
    }
};

#endif // HEADER_POSECLASS
