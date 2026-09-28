// 以下の記事を参考にした。
// ROS 講座 09 ROSメッセージ(と array 型の使い方)
// https://qiita.com/srs/items/080f1ca2ec2b2c480d41

/*
送信するコマンドの数値とあわせて送るパラメータの解説
※  送信する値は現在は常に３個に固定している
※  最初に 8 0 0 を送るとロボットが静止してわかりやすい

ACT_INITPOSE: 0 0 0
ACT_WALK: 1 方向(1 or -1) 繰り返し回数
例 1 1 3   
ACT_TURN: 3 方向(1 or -1) 繰り返し回数
例 3 -1 3   
ACT_NECK: 4 水平回転角度　垂直回転角度
例 4 10 -10
ACT_BODYSHIFT: 5 動作未確認
ACT_SLEEP: 8 0 0
ACT_RESUME: 9 0 0
ACT_DANCE: 12 秒数
例 12 10 0
ACT_IK: 13 動作未確認 
ACT_STAND: 15 0 0
ACT_STAND2: 16 0 0
ACT_MJDA: 18 関節番号 degree
例 18 3 10
ACT_MJRA: 19 関節番号 radian
例 19 3 0.3
ACT_MJDI: 20 関節番号 degree
例 20 3 10
ACT_MJRI: 21 関節番号 radian
例 21 3 0.3
ACT_PJD: 22 関節番号
例 22 3 0 
ACT_PJR: 23 関節番号
例 23 3 0
ACT_RDYNA: 24 動作未確認 
ACT_RELAX: 25 0 0
ACT_ANGLES: 26 0 0
*/

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
//#include <std_msgs/String.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "flask_pub");
    ros::NodeHandle nh;
    ros::Publisher flask_pub = nh.advertise<std_msgs::Float32MultiArray>("/prj4/flask", 10);
    ros::Rate loop_rate(1);

    while (ros::ok()) {
	std_msgs::Float32MultiArray array;
	array.data.resize(10);
	int num = array.data.size();
	for (int i = 0; i < num; i++) array.data[i] = 0;

	printf("ACT_INITPOSE: 0   ACT_WALK: 1   ACT_TURN: 3   ACT_NECK: 4\n"); 
	printf("ACT_BODYSHIFT: 5  ACT_SLEEP: 8  ACT_RESUME: 9  ACT_DANCE: 12  \n"); 
	printf("ACT_IK: 13  ACT_STAND: 15  ACT_STAND2: 16  ACT_MJDA: 18  \n"); 
	printf("ACT_MJRA: 19  ACT_MJDI: 20  ACT_MJRI: 21  ACT_PJD: 22  ACT_PJR: 23  \n"); 
	printf("ACT_RDYNA: 24  ACT_RELAX: 25  ACT_ANGLES: 26  \n\n");
	
	printf("Input numbers (n1 n2 n3) -> ");
	scanf("%f %f %f", &(array.data[0]), &(array.data[1]), &(array.data[2]));
	
	flask_pub.publish(array);
	ROS_INFO("publish: %.2f %.2f %.2f ...", array.data[0], array.data[1], array.data[2]);
/*
	std_msgs::String msg;
	msg.data = "hello world!";
	flask_pub.publish(msg);
	ROS_INFO("publish: %s", msg.data.c_str());
*/
	ros::spinOnce();
	loop_rate.sleep();
    }

    return 0;
}
