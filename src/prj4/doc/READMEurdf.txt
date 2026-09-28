URDF で記述したロボット、オブジェクトを空間中に固定／自由に
動けるように設定する方法

ロボットを構成する最初の link (現在は body) に関する制限
・名前を base_link にすると parent link が不要になる。
・任意の名前にする場合は parent link が必要になる。
　その場合、次のどちらかにする。
　・world を名前にして link を作成して、parent link にする。
　・大きさ等を指定しない名前だけの base_link を作成して、
　　それを parent link にする。

空間中を自由に動くことができるロボットは、最初の構成要素 body
を次のように base_link を parent として設定する。

    <link name="base_link"> </link>
    <joint name="body_joint" type="fixed">
        <parent link="base_link"/>
	<child link="body"/>
        <origin xyz= "0 0 0" rpy="0 0 0" />
    </joint>

一方、空間中の特定の位置に固定して動くことがないオブジェクトは、
最初の構成要素 body を次のように world を parent として設定する。

    <link name="world"> </link>
    <joint name="body_joint" type="fixed">
        <parent link="world"/> 
	<child link="body"/>
        <origin xyz= "0 0 0" rpy="0 0 0" />
    </joint>

空間中に固定するか、動けるようにするかは次の
<gazebo> <static>true</static> </gazebo>
でも設定できる。world, base_link との組み合わせによる効果は
次のようになる。world では動かないようだ。base_link
に対しては <static> で設定できるようだ。
ただしこの設定が有効なのは単純な物体で、redboxs.urdf では効果が
あった。しかし prg4.urdf では <static> の指定は効果がなかった。

  <link name="world"> </link> // not move

  <gazebo> <static>true</static> </gazebo>
  <link name="world"> </link> // not move

  <gazebo> <static>false</static> </gazebo>
  <link name="world"> </link> // not move

  <link name="base_link"> </link> // move

  <gazebo> <static>true</static> </gazebo>
  <link name="base_link"> </link> // not move

  <gazebo> <static>false</static> </gazebo>
  <link name="base_link"> </link> // move


最初に定義する base_link, world は root link と呼ばれ、1個だけ
定義することができる。2個以上定義するとエラーとなり Gazebo が
動作しない。

base_link, world に対する body の定義で
<origin xyz= "0 0 0" rpy="0 0 0" />
はこのようにしておくのがよいと思われる。
座標や姿勢を指定しないでlaunch ファイルから起動すると
地面にめり込んだ状態で現れることがある。
これを改善するには、空間中の位置と姿勢を launch ファイル中で
<arg name="model" default="$(find prj4)/urdf/prg4.urdf" />
<param name="robot_desc" textfile="$(arg model)" />
<node name="spawn_urdf_gz" pkg="gazebo_ros" type="spawn_model"
      args="-param robot_desc -urdf -model prj4 -x -0.15 -y 0 -z 0.75 -R 0 -P 0 -Y 0" />
の4行目のように、-x, -y, -z, -R, -P, -Y で座標と回転を指定すればよい。

-------------------------------------------------------

Howto generate prg4.urdf
脚、ハンド、首、センサを統合したロボット全体の
URDFファイルの生成の仕方

脚、ハンド、首、センサなどの各構成要素は、それぞれ
単独のファイルになっているので、それらを連結することで
ロボット全体のURDFファイルを生成できる。

具体的な手順は buildurdf.sh を見る。

$ cd prj4/urdf
$ cat buildurdf.sh

cat URDFstart.urdf Body.urdf LeftLeg1.urdf LeftHand1.urdf ... Neck.urdf HeadCamera.urdf HandCamera.urdf DepthCamera.urdf LIDAR.urdf ftall.urdf ft5.urdf URDFend.urdf > buildurdf.urdf

のようにURDFstart.urdf と URDFend.urdf の間に必要な要素を並べて buildurdf.urdf に出力する。
prg4.urdf は buildurdf.urdf へのリンクになっている。

実行の仕方

$ ./buildurdf.sh


chcolor.sh はロボットを構成するリンクの色をまとめて変更するためのツール
