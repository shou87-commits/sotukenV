インストール時の手順、初期設定

Gazebo 上で動かす 6脚ロボットのパッケージ名は prj4 である。
現在はインストール時に必要な設定を自動で実行するスクリプト
prj4init.sh を使用する。

※ (重要事項)
以下の説明で $ + すべて大文字の文字列 $WS, $YOU, $VERSION
などは自分の環境に合わせて自分が決めた適切な名前に読み替える。
単語の先頭の $ は実際には使えない文字なので名前には含められない。
-----------------------------------------------------------
$YOU, $USER はユーザのアカウント名
$WS, $ROSWS, $WS1, $WS2 などは ROS のワークスペースの名前の例
$VERSION は ROS の version, 現在は noetic
$DOWNLOAD はパッケージのファイルをダウンロードしたディレクトリで、
　通常は ~/ダウンロード または ~/Downloads となる。
$PKG はパッケージのファイルで prj4.20240101.tgz などのファイル名が
　付けられる。
$BACKUP はデータの消失を防ぐために作成したパッケージのファイル。
　$PKG は配布用に作成した $BACKUP で、両者は本質的に同じものである。
を表す。


状況別のインストールの手順

あらかじめ Ubuntu 20.04, ROS noetic がインストールされている必要がある。
以下を実行する前に必ず上記 (重要事項) を読むこと。


（事例１）ワークスペース $WS1 を新規に作成して prj4 をインストールする手順

以下の＜１＞〜＜４＞を順に最後まで実行する。わからないことがあれば教員や
近くのわかりそうな人に質問する。

＜１＞ワークスペースの作成
※ 以降の $WS1 は独自の名前に置き換える必要があるので注意！！
$ cd
$ mkdir -p ~/$WS1/src
$ cd ~/$WS1/src
$ catkin_init_workspace
$ cd ..
$ catkin_make
$ cd src

＜２＞パッケージファイルの移動と展開
以下で prj4 のパッケージを所定の場所に配置、展開する。
手順：ダウンロードしたパッケージ $PKG を＜１＞で作成した ~/$WS1/src/
に置いてから以下で展開する。
$ cd ~/$WS1/src
$ tar zxvf ./$PKG
$ ls

＜３＞スクリプトによる自動設定
$ cd ~/$WS1/src/prj4
$ ./prj4init.sh
$ source ~/.bashrc

＜４＞実行してみる
$ cd ~/$WS1
$ catkin_make
別ターミナルを開いて
$ cd ~/$WS1
$ roslaunch prj4 prg4_gz.launch
Gazebo が起動できたら別ターミナルを開いて
$ cd ~/$WS1
$ rosran prj4 prg4

Gazebo の最下部左にある再生ボタン ▶ を押すとロボットが動く。
ロボットが歩けばインストールは成功した。歩かない場合は途中で
トラブルが発生しているので相談する。


（事例２）いつも使っている $WS1 に、保管してあったバックアップを戻す手順

戻したい $BACKUP をダウンロード後 ~/$WS1/src/ に置いてから以下で展開する。
$ cd ~/$WS1/src
次のコマンドでこれまであった古い prj4 を削除する。復活させることはできない
ので消しても問題ないかよく確認する。
$ rm -r ./prj4
$ tar zxvf ./$BACKUP
＜３＞は不要
必要なら catkin_make を実行して prg4 を更新する。
あとは普通に作業できる。


（事例３）不調になった $WS1 の代わりに $WS2 で作業できるようにする手順
          $WS1 を削除する場合
	  
事例１の $WS1 を $WS2 に置き換えて＜１＞＜２＞＜３＞＜４＞を実行する。
$WS1 内のファイルをすべて消しても問題ないことが確認できれば
$ cd ~/$WS1/src
$ rm -r ./prj4
で削除することができる。
消したファイルを後で復活させることはできないので注意する。


（事例４）不調になった $WS1 の代わりに $WS2 で作業できるようにする手順
          $WS1 は消さずに残す場合
	  
事例１の $WS1 を $WS2 に置き換えて＜１＞＜２＞＜３＞＜４＞を実行する。
または
$WS1 を残す場合は２行上の＜３＞の代わりに
----------------------------------------
$ cd
$ vim .bashrc
で最後の行の
source ~/$WS1/devel/setup.bash
中の $WS1 を $WS2 に書き換えて保存、終了する。
$ source ~/.bashrc
----------------------------------------
とすることもできる。
この場合 Gazebo は色やテクスチャなどのデータを $WS1 に対してアクセス
するので $WS1 を削除すると色やテクスチャなどがおかしくなる。
修正するには $WS2 用に prj4init.sh を実行すればよい。


（事例５）インストール済の複数のワークスペース（例 $WS1, $WS2）を必要に
　　　　　応じて切り替えながら作業できるようにする手順

今まで使用していた $WS1 を $WS2 に切り替えて作業するには、
$ cd
$ vim .bashrc
で最後の行の
source ~/$WS1/devel/setup.bash
中の $WS1 を $WS2 に書き換えて保存、終了する。
$ source ~/.bashrc
$ cd ~/$WS2


（事例６）バックアップファイル $BACKUP の作成手順

$ cd ~/$WS1/src
$ tar zcvf ./$BACKUP ./prj4
$BACKUP の名前は prj4.YYYYMMDD.tgz のように西暦、月、日を名前に含める
ことで、いつ作成したファイルかわかるようにするのが慣例である。
tgz はファイル形式を表す。こうしてできた $BACKUP を個人で使えるクラウド
の保存領域やUSBメモリなどに保存することで、
PCの故障によるデータ消失を防ぐことができる。
また別PCのワークスペースに展開することで複数の環境で開発の作業
を継続することができる。

＊　　　　　＊　　　　　＊

＊　　　　　＊　　　　　＊

＊　　　　　＊　　　　　＊
-------------------------------------------------------------

以下は以前に手動で設定していたときの設定の手順で、内部で何が行われて
いるか示すために資料として残している。
prj4init.sh が内部で実行している処理にほぼ一致する。
現在は実行する必要はない。

(1) ~/.bashrc の設定

設定ファイルの読み込みを端末起動時に実行するように登録しておくと
毎回手動で実行する必要がなくなる。
$ cd
$ vim .bashrc
開いたファイルの末尾に次の2行のコマンドを書き加えて改行して
変更した内容を保存する。
source /opt/ros/$VERSION/setup.bash
source ~/$WS/devel/setup.bash
$VERSION (noetic, melodic, kinetic 等) は自分の環境に合わせて
書き替える。
セーブ、終了後、次を実行する。
$ source ~/.bashrc

(2) Gazebo で使用する物体のデータを置く場所の確認

$ ls /home/$USER/.gazebo/models/
以前から Gazebo を使用している場合は存在していることがある。
もしなければ次の手順で作る。
$ mkdir -p /home/$USER/.gazebo/models/

(3) 物体の形状を表す sdf ファイルの登録

山の斜面など、sdf 形式で作成された自作の物体のモデルを登録する。
これらのファイルは標準の置き場所が決まっているので、所定の場所から
自分のプロジェクト内にある実体のディレクトリやファイルにリンクを張る。
ここで登録するファイルだけはユーザ領域でローカルに設定できる。
$ cd /home/$USER/.gazebo/models/
-----------------------------------------------------------
$ ls
で既存のファイルを調べる。slope, woodbox などが既に存在していて、もし
過去に作った別のワークスペース内のディレクトリを指定しているようなら
次の操作で一旦削除する。
$ sudo rm slope woodbox ladder jgym bldwall ena
-----------------------------------------------------------
使用するワークスペース内のディレクトリを指定するように次のコマンド
を実行する。
$ sudo ln -s /home/$USER/$ROSWS/src/$PRJ/sdf/models/slope
$ sudo ln -s /home/$USER/$ROSWS/src/$PRJ/sdf/models/woodbox
$ sudo ln -s /home/$USER/$ROSWS/src/$PRJ/sdf/models/ladder
$ sudo ln -s /home/$USER/$ROSWS/src/$PRJ/sdf/models/jgym
$ sudo ln -s /home/$USER/$ROSWS/src/$PRJ/sdf/models/bldwall
$ sudo ln -s /home/$USER/$ROSWS/src/$PRJ/sdf/models/ena

(4) 仮想環境の地面や太陽を表す world ファイルの登録

地面や空の設定、物体の配置を記述した world ファイルを登録する。
これらのファイルは標準の置き場所が決まっているので、所定の場所から
自分のプロジェクト内にある実体のディレクトリやファイルにリンクを張る。
$ cd /usr/share/$GAZEBO/worlds/
-----------------------------------------------------------
$ ls
で既存のファイルを調べる。mysky.world などが既に存在していて、もし
過去に作った別のワークスペース内のディレクトリを指定しているようなら
次の操作で一旦削除する。
$ sudo rm mysky.world mysky_full.world mysky_light.world
$ sudo rm mysky_simple.world spotlight.world
-----------------------------------------------------------
使用するワークスペース内の world ファイルを指定するように次のコマンド
を実行する。
$ sudo ln -s /home/$USER/$ROSWS/src/$PRJ/worlds/mysky.world
$ sudo ln -s /home/$USER/$ROSWS/src/$PRJ/worlds/mysky_full.world
$ sudo ln -s /home/$USER/$ROSWS/src/$PRJ/worlds/mysky_light.world
$ sudo ln -s /home/$USER/$ROSWS/src/$PRJ/worlds/mysky_simple.world
$ sudo ln -s /home/$USER/$ROSWS/src/$PRJ/worlds/spotlight.world

(5) 物体表面の色やテクスチャなどを表すテクスチャファイルの登録

登録するディレクトリに移動
$ cd /usr/share/$GAZEBO/media/materials/textures/
-----------------------------------------------------------
$ ls
で既存のファイルを調べる。tree1.png, tree2.png などが既に存在していて、
もし過去に作った別のワークスペース内のディレクトリを指定しているようなら
次の操作で一旦削除する。
$ sudo rm tree?.png soil?.png cat.png dog.png tiger.png
-----------------------------------------------------------
用意したテクスチャの画像ファイルを登録する。
これらのファイルは標準の置き場所が決まっているので、所定の場所から
自分のプロジェクト内にある実体のディレクトリやファイルにリンクを張る。
使用するワークスペース内のテクスチャを指定するように次のコマンド
を実行する。
$ sudo ln -s /home/$USER/$ROSWS/src/$PRJ/sdf/textures/tree1.png
$ sudo ln -s /home/$USER/$ROSWS/src/$PRJ/sdf/textures/tree2.png
$ sudo ln -s /home/$USER/$ROSWS/src/$PRJ/sdf/textures/tree3.png
$ sudo ln -s /home/$USER/$ROSWS/src/$PRJ/sdf/textures/tree4.png
$ sudo ln -s /home/$USER/$ROSWS/src/$PRJ/sdf/textures/tree5.png
$ sudo ln -s /home/$USER/$ROSWS/src/$PRJ/sdf/textures/tree6.png
$ sudo ln -s /home/$USER/$ROSWS/src/$PRJ/sdf/textures/tree7.png
$ sudo ln -s /home/$USER/$ROSWS/src/$PRJ/sdf/textures/soil1.png
$ sudo ln -s /home/$USER/$ROSWS/src/$PRJ/sdf/textures/soil2.png
$ sudo ln -s /home/$USER/$ROSWS/src/$PRJ/sdf/textures/soil3.png
$ sudo ln -s /home/$USER/$ROSWS/src/$PRJ/sdf/textures/soil4.png
$ sudo ln -s /home/$USER/$ROSWS/src/$PRJ/sdf/textures/grass2.png
$ sudo ln -s /home/$USER/$ROSWS/src/$PRJ/sdf/textures/cat.png
$ sudo ln -s /home/$USER/$ROSWS/src/$PRJ/sdf/textures/dog.png
$ sudo ln -s /home/$USER/$ROSWS/src/$PRJ/sdf/textures/tiger.png

(6) 自分が用意したテクスチャを使用できるようにする登録

次の (6-1), (6-2) のどちらかを実行する。
管理者権限が必要になるので最初に sudo を付ける。
(6-1) ファイルを置き換える方法
$ cd /usr/share/$GAZEBO/media/materials/scripts/
$ sudo cp gazebo.material gazebo.material.org
$ sudo cp /home/$USER/$ROSWS/src/$PRJ/sdf/scripts/gazebo.material.replace gazebo.material
(6-2) 必要な部分のみを追記する方法
$ cd /usr/share/$GAZEBO/media/materials/scripts/
$ sudo cp gazebo.material gazebo.material.org
$ sudo vim gazebo.material
次のファイルの内容を末尾に追加して保存する。
/home/$USER/$ROSWS/src/$PRJ/sdf/scripts/gazebo.material.add

＊　　　　　＊　　　　　＊

以下は README1st.txt に以前記載されていた初期設定の説明である。
これらも実行する必要はない。

その他の設定(launch, world, urdf, sdf, stl)

ここまでの設定で、最低限 Gazebo の起動とロボットを動かすことはできるが、
環境の変更や配置する物体の選択、照明の調整などは以下の操作でできるようになる。
(1) は必須、(2) は設定しておくとよい、(2) をしておくと (3) が可能になる。
(4) を設定しておくと物体表面のテクスチャや色が正しく表示される。


(1)
自作の sdf データを環境に配置する場合は、以下のようにリンクを貼る。
これをしないと山、梯子、ボルダリング壁、瓦礫の山が表示されない。
もし次のディレクトリ
~/.gazebo/models/
が存在しない場合はディレクトリを作成してから以降の設定をする。
$ cd ~/.gazebo/models/
READMEsdf.txt の 50 行目付近に同様の説明がある。

ボルダリング壁を使用する場合
$ ln -s /home/$YOU/$WS/src/prj4/sdf/models/bldwall

山の斜面の新しい地形データを使用する場合
$ ln -s /home/$YOU/$WS/src/prj4/sdf/models/ena

ジャングルジムを使用する場合
$ ln -s /home/$YOU/$WS/src/prj4/sdf/models/jgym

梯子を使用する場合
$ ln -s /home/$YOU/$WS/src/prj4/sdf/models/ladder

山の斜面の旧データを使用する場合
$ ln -s /home/$YOU/$WS/src/prj4/sdf/models/slope

木箱を使用する場合
$ ln -s /home/$YOU/$WS/src/prj4/sdf/models/woodbox

瓦礫の山は URDF 形式で作成されたデータなのでこの設定は必要ない。

(2)
myworld_simple.launch
などから自作の world ファイルを呼び出す場合は以下のように所定の場所に
登録する。例として配布パッケージでは ROS noetic で mysky_simple.world
などを作成して /home/$YOU/$WS/src/prj4/worlds/ に置いてある。
これを登録するときは
$ cd /usr/share/gazebo-11/worlds
$ sudo ln -s /home/$YOU/$WS/src/prj4/worlds/mysky.world
$ sudo ln -s /home/$YOU/$WS/src/prj4/worlds/mysky_simple.world
$ sudo ln -s /home/$YOU/$WS/src/prj4/worlds/mysky_full.world
$ sudo ln -s /home/$YOU/$WS/src/prj4/worlds/mysky_light.world
$ sudo ln -s /home/$YOU/$WS/src/prj4/worlds/spotlight.world

Gazebo で照明などを設定した world ファイルを起動できない場合は
この設定を確認して、設定されてなければ試してみる。

(3)
prg4_gz.launch の先頭部分で myworld_simple.launch, empty_world.launch
などを選択できるので、使わない launch ファイルを <!-- , --> で囲んで
コメントアウト、無効にする。
表示を軽くしたいときは empty_world.launch, mysky_simple.launch などが
よい。

(4)
READMEsdf.txt の 130 行目付近からの
・テクスチャファイルの登録
の指示に従って、物体表面に貼るテクスチャを指定すると、
土や木の表面がリアルに表示される。
READMEsdf.txt の 前半には
・Gazebo のモデル関連の設定ファイルを置くディレクトリの確認
・sdf ファイルの登録
・world ファイルの登録
・~/.bashrc の設定
の説明がある。上記 (1)~(3) とほぼ同じ内容となる。

より詳しい情報は、自作の物体の配置については
prj4/doc/READMEsdf.txt
を見る。
照明の設定や環境の設定は
prj4/doc/READMEworlds.txt
を見る。
