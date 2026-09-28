Link 間に Joint を定義する時の座標指定の考え方

６脚ロボットの本体 body のリンクに左前脚のリンクを接続して Joint
を定義する場合を例として、座標をどのように指定するかを説明する。

body のリンクの大きさは X:300mm, Y:300mm, Z:150mm 、
脚の１番目のリンクの大きさは 50, 50, 50 mm、
脚の２番目のリンクの大きさは 50, 300, 50 mm、

関連する図を doc/images/joint1.jpg ~ joint3.jpg に示すのであわせて
参照するとよい。図中 X軸正値は左斜め手前、Y軸正値は右、Z軸正値は上
となる。

少し長くなるが、サンプルとして用意した urdf ファイル
urdf/example.urdf を以下に示す。
説明のコメントを <!--  --> で囲んで示す。

<?xml version="1.0"?>

<robot name="prj4">

<!--
次のプラグインの設定は Gazebo 上のロボットを ROS で動かすのに必要
となる。
-->

    <gazebo>
        <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
	    <robotNamespace>prj4</robotNamespace>
        </plugin>
    </gazebo>

<!--
最初に定義するリンクで Root link と呼ばれる。大きさなどを定義する
必要はなく、名前だけでよい。空間中のある位置に body を固定したい場合は
base_link ではなく world を指定する。
-->

    <link name="base_link"> </link>
    
<!--
ロボット本体のリンク body を base_link に接続する。座標 xyz と回転 rpy
はすべて０としてよい。この設定でGazebo上にロボットを登場させると地面に
めり込んだ状態になるが、それを変更するには launch ファイルで次のように
位置や回転を指定すればよい。
<node name="spawn_urdf_gz" pkg="gazebo_ros" type="spawn_model"
      args="-param robot_description -urdf -model prj4
      -x -0.15 -y 0 -z 0.75 -R 0 -P 0 -Y 0" />
-->

    <joint name="body_joint" type="fixed">
        <parent link="base_link"/> 
	<child link="body"/>
	<origin xyz= "0 0 0" rpy="0 0 0" />
    </joint>

<!--
赤色のリンク body の大きさや表示、衝突、慣性に関する設定
body の位置は前述の launch ファイル内で設定できるので
ここでの座標や回転は０としてよい。
原点 origin は０とするとリンクの重心になる。
長さの単位はメートル、重さの単位はキログラム
大きさ X:300mm, Y:300mm, Z:150mm はメートルに換算して
ここで定義する。重さは<mass value="2"/>
慣性のパラメータ inertia はこの程度、1e-2 から 1e-3 程度にする
-->

    <link name="body">
        <visual>
	    <origin xyz="0 0 0" rpy="0 0 0"/>
	    <geometry> <box size="0.3 0.3 0.15"/> </geometry>
	</visual>
	<collision>
	    <origin xyz="0 0 0" rpy="0 0 0"/>
	    <geometry> <box size="0.3 0.3 0.15"/> </geometry>
	</collision>
	<inertial>
	    <origin xyz="0 0 0" rpy="0 0 0"/> <mass value="2"/>
	    <inertia ixx="1e-2" ixy="0" ixz="0" iyy="1e-2" iyz="0" izz="1e-2"/>
	</inertial>
    </link>

<!--
リンクの色はこのように指定できる。他の方法もあるが試した結果うまくいかない
ので現在はこの指定法にしている。
-->

    <gazebo reference="body"> <material>Gazebo/Red</material> </gazebo>

<!-- ################################################################### -->

<!--
緑色の肩のリンクのジョイントの設定
ジョイントの原点は parent link の原点からの相対座標で指定する。
左前脚の付け根は 0.15 0.15 0.0 になる。
回転するジョイントでは回転軸を指定する。
この関節は水平に回転するのでZ軸周りの回転で <axis xyz="0 0 1"/>
limit はモータの回転に関する設定、ここでは触れない
-->

    <joint name="jointNN1" type="revolute">
        <parent link="body"/>
	<child link="linkNN1"/>
	<origin xyz= "0.15 0.15 0.0" rpy="0 0 0" />
	<axis xyz="0 0 1"/>
	<limit effort="30" lower="-2.617" upper="2.617" velocity="1.571"/>
    </joint>

<!--
緑色の肩のリンク linkNN1 に関する設定
脚は Y 軸方向に伸びているので、origin の設定は Y だけ 0.025 とする。
0.025 ではなく 0 とするとジョイントの原点とリンクの原点が重なって
表示される。
-->

    <link name="linkNN1">
        <visual>
	    <origin xyz="0 0.025 0" rpy="0 0 0"/>
	    <geometry> <box size="0.05 0.05 0.05"/> </geometry>
	</visual>
	<collision>
	    <origin xyz="0 0.025 0" rpy="0 0 0"/>
	    <geometry> <box size="0.05 0.05 0.05"/> </geometry>
	</collision>
	<inertial>
	    <origin xyz="0 0.025 0" rpy="0 0 0"/> <mass value="0.1"/>
	    <inertia ixx="1e-2" ixy="0" ixz="0" iyy="1e-2" iyz="0" izz="1e-2"/>
	</inertial>
    </link>

    <gazebo reference="linkNN1"> <material>Gazebo/Green</material> </gazebo>

<!-- ################################################################### -->

<!--  
青色の上腕のリンクのジョイントの設定
ジョイントの原点は parent link のジョイント原点からの相対座標で指定する。
この場合 0 0.05 0 になる。
この関節はX軸周りの回転で <axis xyz="1 0 0"/> となる。
-->

    <joint name="jointNN12" type="revolute">
        <parent link="linkNN1"/>
	<child link="linkNN12"/>
	<origin xyz= "0 0.05 0" rpy="0 0 0" />
	<axis xyz="1 0 0"/>
	<limit effort="30" lower="-2.617" upper="2.617" velocity="1.571"/>
    </joint>

<!--  
青色のリンク linkNN2 に関する設定
脚はY軸方向に伸びているので、origin の設定は Y だけ 0.15 とする。
このリンクは長さが 300mm なのでその半分の 150mm を指定している。
-->

    <link name="linkNN12">
        <visual>
	    <origin xyz="0 0.15 0" rpy="0 0 0"/>
	    <geometry> <box size="0.05 0.3 0.05"/> </geometry>
	</visual>
	<collision>
	    <origin xyz="0 0.15 0" rpy="0 0 0"/>
	    <geometry> <box size="0.05 0.3 0.05"/> </geometry>
	</collision>
	<inertial>
	    <origin xyz="0 0.15 0" rpy="0 0 0"/> <mass value="0.3"/>
	    <inertia ixx="1e-2" ixy="0" ixz="0" iyy="1e-2" iyz="0" izz="1e-2"/>
	</inertial>
    </link>

    <gazebo reference="linkNN12"> <material>Gazebo/Blue</material> </gazebo>

</robot>

----------------------------------------------------------

Link を水平に接続する場合と斜めに接続する場合の違い

(1) 水平な Link の場合

Y軸方向に 0.8 の長さを持つ link を Body の (0, 0.6, 0.3) に joint で
接続して、Y軸を回転中心の軸として回転させる。
joint における原点の指定は親リンクから見て (0, 0.6, 0.3)に、
link における原点の指定は前記原点から見て (0, 0, 0)になる。
慣性の計算における原点は重心とした。
READMEjoints.pdf に対応する図を示す。

er14000w.urdf.straight 中の関係部分を抜き出して以下に示す。

    <joint name="jointy" type="revolute">
        <parent link="body"/>
	<child link="linky"/>
	<origin xyz= "0 0.6 0.3" rpy="0 0 0" />
	<axis xyz="0 1 0"/>
	<limit effort="30" lower="-2.617" upper="2.617" velocity="1.571"/>
    </joint>

    <link name="linky">
        <visual>
	    <origin xyz="0 0 0" rpy="0 0 0"/>
	    <geometry> <box size="0.1 0.8 0.1"/> </geometry>
	</visual>
	<collision>
	    <origin xyz="0 0 0" rpy="0 0 0"/>
	    <geometry> <box size="0.1 0.8 0.1"/> </geometry>
	</collision>
	<inertial>
	    <origin xyz="0 0.4 0" rpy="0 0 0"/> <mass value="1"/>
	    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
	</inertial>
    </link>

<!-- ################################################################### -->

(2) 45度回転させた Link の場合

(1) の link を X 軸の周りに45度回転させて付ける場合、次のように
各パラメータの値を変更する。
joint については変更点がないので省略する。
link については回転に伴い、joint に接続する点の座標が移動するので、
そのずれを補正する。回転する前のjointに接続する点の座標を (0, 0, 0) 
とすると、回転後は (0, +0.117157, +0.282843) に移動する(PDF中の図を参照)。
この移動量を符号を反転させて原点に加えることで、もとのjoint　の位置に
付けることができる。よって、link における visual  collision の原点は
(0, -0.117157, -0.282843) となる。ロール、ピッチ、ヨー角の指定は、
X軸周りの回転なので、(-0.7854, 0, 0)となる。45度(=π/4)=0.7854 である。
er14000w.urdf.slant 中の関係部分を抜き出して以下に示す。
(1)の記述と比較するとよい。

    <joint name="jointy" type="revolute">
        略
    </joint>

    <link name="linky">
        <visual>
	    <origin xyz="0 -0.117157 -0.282843" rpy="-0.7854 0 0"/>
	    <geometry> <box size="0.1 0.8 0.1"/> </geometry>
	</visual>
	<collision>
	    <origin xyz="0 -0.117157 -0.282843" rpy="-0.7854 0 0"/>
	    <geometry> <box size="0.1 0.8 0.1"/> </geometry>
	</collision>
	<inertial>
	    <origin xyz="0 0.4 0" rpy="-0.7854 0 0"/> <mass value="1"/>
	    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
	</inertial>
    </link>
