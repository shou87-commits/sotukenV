このファイルには後から追加した機能、新たにわかったこと、注意すべき事柄などを
順次書き加えていく。

------------------------------------------------------------------------------

20200622 追加分

カメラを使えるようにした。例として本体前方と手首に付けてある。必要であれば
台数を増やすことができる。

カメラの機能は class ImageConverter　にまとめられている。カメラを使えるように
するために、main() 中で以下のようにオブジェクトのインスタンスを定義している。
ImageConverter ic?();
この定義はカメラ1台につき1回必要になる。
カメラの定義は URDF ファイルの中にあって、解像度や撮影間隔が定義されている。
現在は 640x480 の解像度で、30frames/sec で撮影する。
画像の取り込みや加工は ImageConverter クラスのメンバ関数 imageCallback() で
行われる。これはサブスクライバのコールバック関数として登録されていて、
取り込んだ画像のメッセージが届くと、それにあわせて実行される。
現在は取り込んだ画像と、その半分の大きさの画像を表示して、2台のカメラの確認用
で画像をファイル /tmp/cam?.ppm に1回記録している。
この関数の呼び出しに合わせて画像処理の機能を追加すれば、ロボット周囲の状況を
認識できる。

取り込んだ画像のサンプルを /tmp/cam1.ppm などとして記録する。

環境内の目印として、様々な色の柱をロボット周囲に立てておいた。その後なくした。

ロボットは一部のリンクの長さや重さを現実に近くなるように URDF ファイル中で変更
した。また慣性の影響を指定するパラメータ( <inertial>内の <inertia> )を標準的な
値に近付けた。その結果挙動が一部変わった。

脚先となるリンクの摩擦係数を標準的な値に近づけた。

ros::spin() の処理を、4個のスレッドで分散して処理するように変更した。

プロジェクトの名前が prj4 で、前回と同じであるので、今まで作業していたワーク
スペースにそのまま展開すると、今までの作業内容が上書きされて消えてしまう。
今までの作業内容を消さないためには、今まで使っていたパッケージの名前を
変えるか、別の場所に移動させておく必要がある。

ここで公開しているサンプルプログラムは、Ubuntu 18.04, ROS Melodic で開発、
動作確認しているため、Ubuntu や ROS の version が異なると実行時の挙動が一部
異なる場合がある。
ROS の version が kinetic の場合、起動時に指定するファイル名が異なる。
この文書の前半部分の使い方の説明のところにも記したが、
$ roslaunch prj4 prg4_rv.launch.kinetic  (kinetic の場合)
$ roslaunch prj4 prg4_gz.launch.kinetic  (kinetic の場合)
となる。また、kineticでは以下の点が異なる。
・半透明の色が指定できず、通常の色として表示される。
・カメラで取り込んだ画像の表示が途中で止まってしまい更新されない。
・カメラで取り込んだ画像をファイルに記録する際に失敗する場合がある。
これらの問題点を除けば、他の部分は問題なく動作した。
kinetic での動作確認は古いノートPCで行ったため、PCの能力が不足することが
原因となっていることも考えられるが確定できていない。
ROS indigo はまだ動作確認ができていないので、今後調べて、必要な部分は
改修する。

------------------------------------------------------------------------------

20200630 追加分

ROS の version が indigo の場合の動作確認をした。

起動時に指定するファイル名が異なり、次となる。
indigo と kinetic は互換性がある。melodic から内容が変わる。
$ roslaunch prj4 prg4_rv.launch.kinetic  (indigo の場合)
$ roslaunch prj4 prg4_gz.launch.kinetic  (indigo の場合)
変更点
indigo, kinetic
<node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" />
melodic
<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />

URDF ファイルの記述内容が kinetic 以降と異なるので、indigo 用の
URDF ファイルを使用する。kinetic と melodic は互換性がある。
urdf/prg4.urdf.indigo を urdf/prg4.urdf に コピー してそれを
読み込ませるようにする。
$ cd launch
$ cp prg4.urdf.indigo prg4.urdf
変更点
indigo
<hardwareInterface>EffortJointInterface</hardwareInterface>
kinetic, melodic
<hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>

問題点
・半透明の色が指定できず、通常の色として表示される。
上記を除けば、他の部分は問題なく動作した。

kinetic ではカメラ画像の表示に問題があったが、indigo では表示できた。
PCの性能に依存する可能性があるので、環境毎に結果は異なる可能性がある。

表示色については URDF ファイル中の色の指定で原色を指定すれば正しく
表示される可能性が高い。

------------------------------------------------------------------------------

20201201 追加分

URDF の先頭部分を変更した。
これまでは base_link (または world, world1 など、最初のリンク) と body を
floating で接続していたが、これはいくつかの問題となり、noetic ではうまく
動かなかったので、調べた結果から、fixed に変更した。この変更により問題が
なくなった。また base_link に余計なパラメータを記述するのも良くないことが
わかった。よって現在は下記の記述にしている。

<?xml version="1.0"?>

<robot name="prj4">
    <gazebo>
        <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
            <robotNamespace>prj4</robotNamespace>
        </plugin>
    </gazebo>

    <link name="base_link"> </link>

    <joint name="body_joint" type="fixed">
        <parent link="base_link"/>
        <child link="body"/>
        <origin xyz= "1 1 1" rpy="0 0 0" />
    </joint>


慣性の影響で物体が時間経過に伴って弾んだり、滑り出す現象について ixx などの値を
1 など大きな値にすると徐々に弾むようになった。
値を 1e-3 など小さな値にするとすぐに滑るようになった。
1e-1 -- 1e-2 程度にすると比較的安定する。
<inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/> <mass value="20"/>
    <inertia ixx="5e-2" ixy="0" ixz="0" iyy="5e-2" iyz="0" izz="5e-2"/>
</inertial>


ROS noetic の rviz でスライダが自動で表示されなくなった現象について 以下にあるとおり、
urdf, launch のどちらかで 'use_gui' がなくなったせいのようだ。
関連する設定を変更すれば直る見込み。
[WARN] [1606815234.049128]: The 'use_gui' parameter was specified,
which is deprecated.  We'll attempt to find and run the GUI,
but if this fails you should install the 'joint_state_publisher_gui'
package instead and run that.
This backwards compatibility option will be removed in Noetic.

------------------------------------------------------------------------------

20210111 追加分

環境内に sdf で記述した物体(ladder.sdf)を出す方法、パッケージ名を
pkg とする。

$ cd  使っているワークスペース/src/pkg
$ mkdir sdf
ladder.sdf を sdf/ 内にコピーする。
$ vim launch/??????.launch
以下の記述例の後半部分のように追記する。
-----
<?xml version="1.0"?>
<launch>
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
      <arg name="paused" value="false"/>
  </include>

中略

  <arg name="ladder" default="$(find prj4)/sdf/ladder.sdf"/>
  <param name="ladder_description" textfile="$(arg ladder)"/>
  <node name="ladder_spawner" pkg="gazebo_ros" type="spawn_model"
    	args="-param ladder_description -sdf -x -2.5 -y 2.0 -z 0.0 -model ladder" />
</launch>
-----

$ roslaunch pkg ??????.launch
です。
やっていることは、物体の sdf データを用意して、それを起動時に
呼び出す指示を ?????? .launch の最後に３，４行追加することです。


kinetic では画像を表示すると途中で更新が止まってしまうので、その機能は
一旦コメントアウトした。他の version では問題ないようだ。
また kinetic 以前では色の表示に一部対応していなくて白くなるようだ。
その他の機能は一通り動いているようだ。

prj4/CMakelists.txtの先頭部分で、g++ のオプションの設定を有効にした。-std=c++11
これにより kinetic 以前で警告を表示しなくなった。

メッセージに従って Ubuntu を update したら、その後何回起動しても gazebo が動かなかった。
ターミナルの再起動は効果がなかったが、Ubuntu を再起動したら直った。

アームが動いている途中で制御のモードを変更すると、タイミングによっては一部の動きが
実行されずスルーされてしまう現象が確認された。よって制御のモードは変える必要がなければ
固定しておく方が良い。

------------------------------------------------------------------------------

20210311 追加分

シンボリックリンクで ROS や Gazebo の version に依存するものとして
以下があるので、version が異なる場合は適切な記述で設定し直す必要がある。

% cd ~/ros/6leg/src/prj4/
% cd launch/
% ls -l
% rm launch
% ln -s /opt/ros/noetic/share/gazebo_ros/launch

% cd ../worlds/
% ls -l
% rm camera.world empty_sky.world everything.world mud.world worlds
% ln -s /usr/share/gazebo-11/worlds/camera.world
% ln -s /usr/share/gazebo-11/worlds/empty_sky.world
% ln -s /usr/share/gazebo-11/worlds/everything.world
% ln -s /usr/share/gazebo-11/worlds/mud.world
% ln -s /usr/share/gazebo-11/worlds

% cd ../sdf/scripts/
% ls -l
% rm gazebo.material.org
% ln -s /usr/share/gazebo-11/media/materials/scripts/gazebo.material gazebo.material.org
% cd ../textures/
% ls -l
% rm textures
% ln -s /usr/share/gazebo-11/media/materials/textures/

ROS noetic, gazebo 11 では URDF ファイル中で以下のような material の指定があると警告が出る
ようになったのでコメントアウトまたは削除した。

    <link name="linkNN14">
        <visual>
            <origin xyz="0 0.025 0" rpy="0 0 0"/>
            <geometry> <box size="0.05 0.05 0.05"/> </geometry>
<!--        <material name="blue"/> -->
        </visual>

ROS noetic, gazebo 11 では SDF のモデルを環境中に配置するときに名前のチェックがしっかり
行われるようになったので、同じ名前の物体モデルを２個以上配置できなくなった。同じ名前のものが
あると、２個目以降が無視されて警告が表示される。melodic までは許されていた。
名前が重複する場合に修正が必要なファイルは各モデルの model.config, model.sdf, sdfhead.sdf
である。

------------------------------------------------------------------------------

20210510 追加分

launch ファイルから sdf で記述した物体を配置する。ロール、ピッチ、ヨー角の指定

prj4/launch/prg4_gz.launch.empty　の
ファイルの最後の部分を見ると以下のようになっていると思います。
これは梯子 ladder を出す設定です。

    <arg name="ladder" default="$(find prj4)/sdf/models/ladder/model.sdf"/>
    <param name="ladder_description" textfile="$(arg ladder)"/>
    <node name="ladder_spawner" pkg="gazebo_ros" type="spawn_model"
          args="-param ladder_description -sdf -x 2.5 -y 4.0 -z 0.0 -model ladder" />

</launch>

この部分に以下のように他の物体、ジャングルジム jgym, ボルダリング壁 bldwall,
山の斜面 slope を出す設定を追加します。不要なものは slope のように
<!--,  --> で囲むことでコメントアウトできます。
それぞれの設定の中の -x -12.0 -y -3.0 -z -3.5 -R 0 -P 0.7854 -Y 0
のような記述は、物体のX,Y,Z 座標と、ロール、ピッチ、ヨーの回転角度です。
座標の単位はメートル、角度はラジアンです。3.1415 で９０度回転します。

    <arg name="ladder" default="$(find prj4)/sdf/models/ladder/model.sdf"/>
    <param name="ladder_description" textfile="$(arg ladder)"/>
    <node name="ladder_spawner" pkg="gazebo_ros" type="spawn_model"
          args="-param ladder_description -sdf -x 2.5 -y 4.0 -z 0.0 -model ladder" />

    <arg name="jgym" default="$(find prj4)/sdf/models/jgym/model.sdf"/>
    <param name="jgym_description" textfile="$(arg jgym)"/>
    <node name="jgym_spawner" pkg="gazebo_ros" type="spawn_model"
          args="-param jgym_description -sdf -x 2.5 -y -3.0 -z 0.0 -model jgym" />

    <arg name="bldwall" default="$(find prj4)/sdf/models/bldwall/model.sdf"/>
    <param name="bldwall_description" textfile="$(arg bldwall)"/>
    <node name="bldwall_spawner" pkg="gazebo_ros" type="spawn_model"
          args="-param bldwall_description -sdf -x 2.5 -y -9.0 -z 0.0 -model bldwall" />
<!--
    <arg name="slope" default="$(find prj4)/sdf/models/slope/model.sdf"/>
    <param name="slope_description" textfile="$(arg slope)"/>
    <node name="slope_spawner" pkg="gazebo_ros" type="spawn_model"
          args="-param slope_description -sdf -x -12.0 -y -3.0 -z -3.5
          -R 0 -P 0.7854 -Y 0 -model slope" />
-->
</launch>

sdf ファイル中のコメントが問題となることがある。上記の指定をした後、実行したら
エラーが出て bldwall, slope は表示できなかった。原因はsdf ファイル中のコメント
で、
<!--  ------------------------------------------  -->
という行があるとそこで止まってしまった。当該行を消したら直った。
world ファイルから配置する場合は問題なかったが、launch ファイルから指定する場合は
問題となった。

slope を正しく配置するには、ピッチ角をプラスで４５度回転させて、高さを -3.5m にする。

ROS noetic では、それまで動いていた６脚のカメラの機能が動かなくなった。rviz で
見ても、画像が配信されていないようだ。同じプログラムを melodic で動かすと問題なく
動いたので、melodic と noetic の間で何かが変わった可能性がある。
※ その後、別のPC, noetic で試したら問題なく動いたので、その個体固有の問題のようだ。

同じプロジェクトの新旧ヴァージョンを入れ替えて動作の確認をするときに、入れ替えた後の
ファイル群でビルドをしても新しい方のソースによる実行ファイルが正しく生成されない
ことがある。その場合は以下のようにいくつかのファイルを touch するとビルドが正しく
行われるようだ。
例としてワークスペースを WS、プロジェクトを PRJ、実行ファイルを BIN とすると、
$ cd ~/WS/src/PRJ
$ touch CMakeLists.txt src/*.cpp src/*.h
あたりが有効だ。
参考までに、ビルドの結果生成される一連のファイルは以下の場所にあるようだ。
$ cd ~/WS/build/PRJ/CMakeFiles/BIN.dir/src
$ ls
ここにはオブジェクトファイルがある。
$ cd ~/WS/devel/lib/PRJ
$ ls
ここには実行ファイルがある。
上記の場所にあるオブジェクトファイル、実行ファイルを削除してから再度
$ catkin_make
を実行すると直るかも。

------------------------------------------------------------------------------

20220303

環境が明るくなったので、mysky_simple.world の補助照明を
コメントアウトした。

カラー画像と深度画像を同時に表示させようとすると以下の
エラーで動かない。
深度画像だけの表示ならできるが、変な結果が混ざる。
rviz ではすべて正しく表示される。
問題の原因は以下のメッセージにあるように multi-threaded か？
// $ rosrun prj4 prg4 
// start ActInitialPose()
// [xcb] Unknown sequence number while processing reply
// [xcb] Most likely this is a multi-threaded client and XInitThreads has not been called
// [xcb] Aborting, sorry about that.
// prg4: ../../src/xcb_io.c:641: _XReply: アサーション `!xcb_xlib_threads_sequence_lost' に失敗。
// 中止 (コアダンプ)

------------------------------------------------------------------------------

20220114

user.cpp

ファイルの最初にある定数の設定は const.h に移動した。

LIDAR を使うための lidarclass.h　が増えた。

LIDAR 測距センサのデータにアクセスするためのポインタ
extern LIDAR *gLIDAR;
が増えた。

MyMotion() の書き方が一部変わった。これまでは
switch 文のそれぞれの選択肢の中で次の行動を決めていたが、
次の行動を決めるif文と、個々の行動の設定と呼び出しを
行うswitch 文に分離した。
MyMotion() の前半が上記のif文、後半がswitch 文になっている。

act_param[] を活用して様々なパラメータのやり取りを行うように
なった。

act_param[] のどの要素が何のパラメータを表すかは、
const.h の AP_???? で指定されている。

act_param[AP_STATE] は行動の開始、途中、終了を表す。
ACT_START, ACT_MOVING, ACT_END を保持する。すべての行動で必要。
これまでは個々の行動の関数内の timer で開始や終了を管理して
いたが、この仕組みだとMyMotion()から開始や終了を指示できない場合
があるので変更した。

個々の行動でどんなパラメータが必要化は MyAction() を見る。

act_param[AP_TIMETOTAL]は、その行動にかかる時間を指定する。
すべての行動で必要。

歩行や旋回などの基本的な行動が用意された。

LIDAR センサを使って物体までの距離を測定する ActLidar()
が使えるようになった。ACT_LIDAR でMyAction() から呼び出す。
このセンサは水平に１２０度広がるレーザで、１度ずつSCANして
距離を求める。

user.cpp からLIDAR センサにアクセスするためのポインタ、gLIDAR 
がある。測定結果は gLIDAR -> distance[] に１２１点記録される。

距離と方向から３次元座標を求める関数
gLIDAR -> Lidar2Xyz(gLIDAR -> distance[i], i, vangle, &x, &y, &z);
がある。結果のx, y, z はローカル座標で、そのままモノを掴むときの
対象の座標として使える。

縦のSCANを行うActLidarVScan() はまだ完成していない。

複数の動きを組み合わせて、まとめて呼び出す書き方ができるように
なった。Act12() を見る。この例では３個の動きが含まれている。
以下のようなパラメータが必要。
act_param[AP_TIMETOTAL] 合計時間
act_param[AP_TIME1]     動き１の時間
act_param[AP_TIME2]     動き２の時間
act_param[AP_TIME3]     動き３の時間
act_param[AP_TIME4]     動き４の時間
act_param[AP_CTRL]      制御の手法
act_param[AP_PHASE]     動きの種類
act_param[AP_ITER]   　 一連の動きの繰り返し回数

ACT_WALK, ACT_TURN は４種類の動きの組み合わせだが、個々の動きの
時間が同じなので、Act12()とは別の仕組みで実現されている。

---------------------------------------------------------------

その他

環境の移行について

変更点が多いので、新しい環境に移行するときは、
・今まで自分が書いたプログラムは別の場所にコピーを作って保管
・新しいワークスペースを作る。
・そこに新しいパッケージを展開する。
・~/.bashrc を新しいワークスペース用に設定を書く。古い
　ワークスペース用の設定は一旦無効にする。
・catkin_make clean を実行
・catkin_make を実行
・自分が作成した機能を統合する。

各種マニュアルを prj4/doc に集約した。

補助照明がついている新しいワールドを用意した。設定は
prj4/doc/README1st.txt の７０行目あたりを見て、
mysky_full.world
mysky_simple.world
を設定する。

launchファイルから直接 ladder, jgym, slope, bldwall を
配置できるようにした。
launch/prg4_gz.launch の末尾を見る。

LIDAR の扇形のビームを消すときは、
prj4/urdf/prg4.urdf 末尾の
<visualize>True</visualize>
を False にする。

ACT_LIDARから始まるデモの動きで、画面表示は以下のように
なる。
$ rosrun prj4 prg4
Dist:1.17 H:0 V:21.0 x:1.24 y:0.00 z:0.68
Dist:1.17 H:1 V:21.0 x:1.24 y:0.02 z:0.68

Dist:1.20 H:-25 V:12.0 x:1.22 y:-0.50 z:0.51
Dist:1.19 H:-24 V:12.0 x:1.22 y:-0.48 z:0.51
Dist:1.21 H:-23 V:12.0 x:1.24 y:-0.46 z:0.51

Dist:1.65 H:-25 V:1.0 x:1.65 y:-0.70 z:0.29
Dist:1.44 H:43 V:1.0 x:1.20 y:0.98 z:0.29
Dist:1.41 H:44 V:1.0 x:1.16 y:0.98 z:0.28
Dist:1.45 H:45 V:1.0 x:1.17 y:1.02 z:0.29

その意味は、
Dist:は距離、H:は水平方向の角度、V:は垂直方向の角度、
x: y: z:　ローカル座標で表した対象物の座標となる。
３個の赤い箱は
(1, 0, 1), (1, 1, 0.6), (1, -0.5, 0.8)、
ロボットの重心の座標は大体
(-0.2, 0, 0.3)
なので、ほぼ正しい座標が求められていることがわかる。
ただし時々間違った座標が出ることもある。

求めた座標は今は表示しているだけなので、次の行動で座標を
利用するようなプログラムにする必要がある。

