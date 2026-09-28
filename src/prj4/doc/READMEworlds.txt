環境を定義する world ファイルについて

ROS のワークスペースを $WS とする。
$ cd ~/$WS/src/prj4/worlds
$ ls

ここにある world ファイルは環境を定義する。地面や空、照明が設定できる。
自分が作った梯子、ジャングルジム、ボルダリング壁などのオブジェクトも
world ファイルから配置できる。ただし、オブジェクトの配置は launch
ファイルからでもできる。

ここにあるシンボリックリンクは、ここから実行するためではない。
例としてこのようなものがあることを示すためにある。

例として
$ roslaunch prj4 prg4_gz.launch
として起動すると、その中から以下のどれかが実行されて環境が設定される。
launch/empty_sky.launch
launch/empty_world.launch
launch/myworld.launch
上記３個のファイルからは更に、
/usr/share/gazebo-11/worlds/
の中にある world ファイルが呼び出されて環境が設定される。

注意する点として、world ファイルはここに置くだけでは機能せず、
必ず /usr/share/gazebo-11/worlds/ に置くか、そこからシンボリックリンク
を張る必要がある。
例として、補助照明を付加して表示を明るくする mysky_light.world 
を指定できるようにする場合は
$ cd /usr/share/gazebo-11/worlds/
$ ln -s ~/$WS/src/prj4/worlds/mysky_light.world
$ cd
を実行しておいて、prg4_gz.launch 中の
<include file="$(find prj4)/launch/myworld.launch">
または
<include file="$(find prj4)/launch/myworld_simple.launch">
の代わりに
<include file="$(find prj4)/launch/myworld_light.launch">
を有効にする。
    
prj4/worlds/ にある
mysky_light.world
や
spotlight.world
は、照明の設定やオブジェクトの配置をカスタマイズしたものである。

照明の設定をカスタマイズした
lights.world などを実行してエラーが出るときは、最後の光源の
<direction>0 0 -1</direction>
が原因なので例えば以下のように変更する。
<direction>0.01 0.01 -0.99</direction>
これで直る。
