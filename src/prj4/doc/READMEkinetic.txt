ROS kinetic などの古い version の ROS で prj4 を動かす時の Tips

最近の prj4 は主として noetic で動作確認をしていて、古い ROS との
互換性を確認する機会が少なくなっているので、
ここにわかっていることをまとめて記録しておく。

・launch ファイル prg4_gz.launch で気をつける点 
robot_state_publisher の設定が以下と異なっていたら以下のように修正する。

<node name="robot_state_publisher_gz" pkg="robot_state_publisher" type="state_publisher" />

・起動する world を指定する記述を見て、どのファイルが読み込まれるか
　特定する。

<include file="$(find prj4)/launch/empty_world.launch">
    <include file="$(find prj4)/launch/myworld_full.launch">
    <include file="$(find prj4)/launch/myworld_simple.launch">

例として empty_sky.launch であれば、次にそのファイルの内容を調べて、
その中で spotlight.world が指定されていたら kinetic にはないので
以下のように別のファイルに変更する。
<arg name="world_name" default="worlds/spotlight.world"/> 
代わりに以下のように変更する。
<arg name="world_name" default="worlds/empty_sky.world"/>

・launch ファイルで legacyModeNS に関するエラーが出たら指示に従って直す。

[ERROR] [1671709279.036139956]: GazeboRosControlPlugin missing <legacyModeNS> while using DefaultRobotHWSim, defaults to true.
This setting assumes you have an old package with an old implementation of DefaultRobotHWSim, where the robotNamespace is disregarded and absolute paths are used instead.
If you do not want to fix this issue in an old package just set <legacyModeNS> to true.

・起動時に以下のエラーが出たら prg4 プログラムのソース中の isnan() を
std::isnan() に書き換えてコンパイル、再実行する。

[ INFO] [1671709052.877851511]: waitForService: Service [/gazebo/set_physics_properties] has not been advertised, waiting...
terminate called after throwing an instance of 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
  what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
Aborted (core dumped)
[gazebo-2] process has died [pid 31430, exit code 134, cmd /opt/ros/kinetic/lib/gazebo_ros/gzserver -u -e ode worlds/spotlight.world __name:=gazebo __log:=/home/any/.ros/log/058c466a-81ed-11ed-a6b8-00e05e680c29/gazebo-2.log].
log file: /home/any/.ros/log/058c466a-81ed-11ed-a6b8-00e05e680c29/gazebo-2*.log


・kinetic rviz に関するエラー

[WARN] [1609052880.135369]: The 'use_gui' parameter was specified, which is deprecated.  We'll attempt to find and run the GUI, but if this fails you should install the 'joint_state_publisher_gui' package instead and run that.  This backwards compatibility option will be removed in Noetic.
[ERROR] [1609052880.135902]: Could not find the GUI, install the 'joint_state_publisher_gui' package
[joint_state_publisher-2] process has died [pid 29416, exit code 1, cmd /opt/ros/kinetic/lib/joint_state_publisher/joint_state_publisher __name:=joint_state_publisher __log:=/home/any/.ros/log/405fbf9a-4812-11eb-837b-00e05e680c29/joint_state_publisher-2.log].
log file: /home/any/.ros/log/405fbf9a-4812-11eb-837b-00e05e680c29/joint_state_publisher-2*.log

・kinetic gazebo に関するエラー

ERROR: cannot launch node of type [controller_manager/spawner]: controller_manager
ROS path [0]=/opt/ros/kinetic/share/ros
ROS path [1]=/home/any/ros/ri/src
ROS path [2]=/home/any/ros/6leg/src
ROS path [3]=/opt/ros/kinetic/share

以下を実行すると直る。
$ sudo apt install ros-kinetic-ros-control

以下がコメントとして残っている。
but robot model was not appeared
