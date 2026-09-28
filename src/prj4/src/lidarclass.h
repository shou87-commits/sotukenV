// lidarclass.h   LIDAR 用の設定

// 動作しないときは次を試す、noetic は適切な名前に書き換える
// $ sudo apt install ros-noetic-urg-node

#ifndef	HEADER_LIDARCLASS
#define	HEADER_LIDARCLASS

const float MINANGLE = (M_PI / 180 * -60); // -60 度の場合
const float DEG1     = (M_PI / 180); // LIDAR の角度の変化が１度の場合

const int LIDAR_BUFX = 192; // LIDAR の結果を保存する配列の横方向のサイズ
const int LIDAR_BUFY = 64;  // LIDAR の結果を保存する配列の縦方向のサイズ

class LIDAR
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber lidar_sub_;
    const char *sname;

public:
    int numdata; // データの個数
    float range_max;
    float h_angle; // センサの水平方向の角度、正面が０度、±６０度
    float v_angle; // センサの垂直方向の角度、正面が０度
    float distance[LIDAR_BUFX]; // 左右に広がる扇形の各ビームの検出距離、要素数はnumdata以上要
    float dist2D[LIDAR_BUFY][LIDAR_BUFX]; // LIDAR の結果を保存する配列、サイズに注意
    float vup, vdown;
    
    LIDAR(const char sn[])
    {
        sname = sn; // 引数による名前の設定

        lidar_sub_ = nh_.subscribe(sname, 1000, &LIDAR::LidarCallback, this);
    }

    ~LIDAR(){}

    void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        numdata = msg -> ranges.size(); // データの個数、±６０度と中央で１２１個
        range_max = msg -> range_max;

        for (int i = 0; i < numdata; i++ ) {
            distance[i] = msg -> ranges[i];
/*          if (distance[i] < msg -> range_max && // ここは今は使っていない
		msg -> range_min < distance[i] &&
		distance[i] < range_min) { // 距離の最小値を求める
                range_min = distance[i];
                h_angle = MINANGLE + (i * DEG1);   // その時の角度を求める
            }
*/
        }
    }

    int Lidar2Xyz(float dist, int idx, float v, float *x, float *y, float *z)
    {
	h_angle = MINANGLE + (idx * DEG1); // 値の正負が気になるがその後の結果は正しい
	                                   // 配列に記録されるのが扇の右からになるため
	v_angle = v * -1;                  // 値の正負が気になるがその後の結果は正しい
	
	*x = dist * cos(v_angle) * cos(h_angle);
	*y = dist * cos(v_angle) * sin(h_angle);
	*z = dist * sin(v_angle);

// body の重心からの座標になるように body, 首のモータとカメラの大きさ分を補正する
	*x += (0.3 / 2); // 本体の長さの半分を足す
//	*y += 0.0; // センサの設置位置、左右方向は中央なので補正不要
// 本体の高さの1/2と首のモータ2個とカメラ2個の高さ、LIDARセンサの高さの1/2を足すとレンズの中心
	*z += (0.15 / 2 + 0.05 + 0.05 + 0.05 + 0.05 + 0.035);
//	printf("\nv:%.1f sin(v):%.1f d*sin(v):%.1f\n", v_angle, sin(v_angle), dist*sin(v_angle));

	return 0;
    }

};

#endif // HEADER_LASERCLASS
