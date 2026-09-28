Gazebo で自作の物体モデル (.sdf)を環境に置く、好きなテクスチャを貼る方法

※
このファイルは sdf で作成した物体モデルを gazebo の環境内に配置する方法を
示すサンプルプロジェクト mysky の README をもとにして必要な部分を書き換えた
ものである。いくつか名前が異なる部分があるが、配置の原理は同じである。

以下の説明では、$ から始まる名前で指定される次の作業環境を仮定する。
この後、$ で始まる名前が出てきた場合は、必ず適切な名前に置き換えて読む。
・Ubuntu は 20.04、$ROSVER は ROS の version で noetic
・ユーザ名は $USER
・ROS のワークスペースは $ROSWS
・$GAZEBO は Gazebo の version で gazebo-11
・パッケージ名 $PRJ は prj4
旧バージョンの Ubuntu 18.04 の場合は、$ROSVER が melodic 、$GAZEBO は gazebo-9
に変わる。

※ Linux のシステムが所有するディレクトリ (/usr/...) での作業 (ln -s など)
　には管理者権限が必要になるので、適宜 sudo を付けて実行する。


ファイル、ディレクトリの紹介

$ cd /home/$USER/$ROSWS/src/$PRJ/
$ ls -l
合計 4796
-rw-r--r-- 1 any any    6997 10月 15 21:35  CMakeLists.txt  // 触らない、見る必要なし
-rw-r--r-- 1 any any    2702 10月 16 10:06  README1st.txt   // 最初に読むマニュアル
drwxr-xr-x 2 any any    4096 10月 15 21:36  config/         // 触らない、見る必要なし
drwxr-xr-x 2 any any    4096 10月 15 21:36  doc/            // マニュアルがまとめられている
drwxr-xr-x 3 any any    4096 10月 15 21:27  include/        // 触らない、中身はほぼ空
drwxr-xr-x 2 any any    4096 10月 15 21:50  launch/         // ★今回必要、重要
-rw-r--r-- 1 any any    2962 10月 15 21:27  package.xml     // 触らない、見る必要なし
drwxr-xr-x 5 any any    4096 10月 15 17:40  sdf/            // ★ 今回必要、重要
drwxr-xr-x 2 any any    4096 10月 15 21:33  src/  	    // 触らない
drwxr-xr-x 3 any any    4096 10月 15 21:34  urdf/  	    // 触らない
drwxr-xr-x 2 any any    4096 10月 16 09:57  worlds/   	    // ★今回必要、重要

なお、2024年の時点ではある程度ディレクトリやファイルが増えている。

今回は環境の設定だけでロボットが登場しないので、上記の「触らない」部分は
実行に関係しない。環境内でロボットを動かす場合は必要になる。


Gazebo のモデル関連の設定ファイルを置くディレクトリ

Gazebo で表示する物体の外観は形、色、テクスチャなど複数の
形式のデータで指定する。データの置き場所は以下になる。
設定内容を調べたいときは各ディレクトリを見る。

・物体表面の色やテクスチャなど
/usr/share/$GAZEBO/media/materials/
/usr/share/$GAZEBO/media/materials/scripts/
/usr/share/$GAZEBO/media/materials/textures/
・仮想環境の太陽や地面など
/usr/share/$GAZEBO/worlds/
・物体のモデルデータ、この種類のデータはローカルに持てる
/usr/share/$GAZEBO/models/
~/.gazebo/models/


設定

以前は手動で様々な設定をしていたが、2024年6月から必要事項を
自動で設定する
prj4init.sh
というスクリプトが使用できるようになった。
README1st.txt に使用方法が記載されている。


動作確認

次のコマンドでは地面と空が表示される。
$ cd ~/$WS
$ roslaunch $PRJ mysky.launch
次のコマンドでは工場や家のモデルが表示される。
$ roslaunch $PRJ mysky_full.launch
mysky_full.launch の表示に必要なデータはダウンロードしたものが
あるので、必要な場合は教員に相談してみる。
工場や家のモデルを配置する場合、
最初に起動する時は、サンプルとして提供されているモデルの
データを外部からダウンロードする場合は時間がかかることがあるので
待ってみる。３０分経過しても表示されない場合は
一旦終了して再度実行してみる。
工場や家の表示は必須ではないのでスキップしてもよい。


その他

launch ファイルから起動する際の処理の流れ

$ roslaunch $PRJ mysky.launch では、
/home/$USER/$ROSWS/src/$PRJ/launch/mysky.launch から
/home/$USER/$ROSWS/src/$PRJ/launch/myworld.launch を起動する。
/home/$USER/$ROSWS/src/$PRJ/launch/myworld.launch は
/home/$USER/$ROSWS/src/$PRJ/worlds/mysky.world を起動する。

/home/$USER/$ROSWS/src/$PRJ/launch/myworld.launch は
/opt/ros/$ROSVER/share/gazebo_ros/launch/empty_world.launch をコピーして、
empty.world の代わりに
/home/$USER/$ROSWS/src/$PRJ/worlds/mysky.world を起動するように変更したものである。

/home/$USER/$ROSWS/src/$PRJ/worlds/mysky.world は
/usr/share/$GAZEBO/worlds/empty_sky.world をコピーして、
用意した物体を配置するように変更したものである。
環境内に配置する物体の変更や位置を変えたい場合はこのファイルを編集する。
また、このファイル内の次の値 0.75 を変えると環境の明るさや光の色が変わる。
<ambient>0.75 0.75 0.75 1</ambient>

物体の形や色、テクスチャを変えたい場合は次のファイルを編集する。
例として自作したモデル woodbox なら、
/home/$USER/$ROSWS/src/$PRJ/sdf/models/woodbox/model.sdf

ROS noetic, gazebo 11 では SDF のモデルを環境中に配置するときに名前のチェックがしっかり
行われるようになったので、同じ名前の物体モデルを２個以上配置できなくなった。同じ名前のものが
あると、２個目以降が無視されて警告が表示される。melodic	までは許されていた。
名前が重複する場合に修正が必要なファイルは各モデルの model.config, model.sdf, sdfhead.sdf
である。


練習

自作の箱のモデル
catbox, dogbox, tigerbox を作って、
表面に cat.png, dog.png, tiger.png のテクスチャを貼った箱を
環境中に置いてみる。
















