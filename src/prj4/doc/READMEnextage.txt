6脚ロボットと Nextage Clone で使用するファイル、コマンドの違い

制御のプログラムとして prg4 が共通で使えるように、
Nextage とほぼ同じ大きさ、機能を持つ Nextage Clone 
を prj4 のプロジェクトに作成した。

6脚ロボットと Nextage Clone はロボットの構造と脚（アーム）
の本数が異なるので、一部のファイルの内容が異なる。
以下に挙げる5個のファイルは6脚ロボット用である。

prj4/launch/prg4_gz.launch　ロボットのモデルとしてprg4.urdfを呼び出す
prj4/urdf/Body.urdf 　　　　ロボット本体(body)のURDFファイル
prj4/urdf/buildurdf.sh　　　ロボット各部のURDFファイルを連結するスクリプト
prj4/urdf/buildurdf.urdf　　ロボット各部のURDFファイルを連結した結果
prj4/urdf/prg4.urdf 　　　　buildurdf.urdfへのリンクファイル

上記のファイルが Nextage Clone では異なる内容となり別ファイルとなる。
ファイル名の先頭に N を付けて Nextage Clone 用のファイルであることが
わかるようにしている。

prj4/launch/Nprg4_gz.launch ロボットのモデルとしてNprg4.urdfを呼び出す
prj4/urdf/NBody.urdf　　　　Nextage本体(body)のURDFファイル
prj4/urdf/NHead.urdf　　　　Nextageの頭部のURDFファイル、追加
prj4/urdf/Nbuildurdf.sh 　　Nextage各部のURDFファイルを連結するスクリプト
prj4/urdf/Nbuildurdf.urdf 　Nextage各部のURDFファイルを連結した結果
prj4/urdf/Nprg4.urdf　　　　Nbuildurdf.urdfへのリンクファイル

6脚ロボットの起動のコマンド指定は以下になる。
$ roslaunch prj4 prg4_gz.launch
$ rosrun prj4 prg4

対して Nextage Clone の起動のコマンド指定は以下になる。
$ roslaunch prj4 Nprg4_gz.launch
$ rosrun prj4 prg4

現状では制御用のプログラム prg4 は共通であるが、今後別のものになる
可能性がある。

※
2022年06月時点で、プログラム prg4 は共通であるが、１箇所だけ６脚と
Nextageで異なる処理が必要になったので最新のプログラムでは使っている
モデルを区別するために次の措置が必要となった。

今後配布するパッケージでは使うモデルの種類を src/const.h の
冒頭部分で以下のように宣言する。

#define MODEL_6LEG
#define MODEL_NEXTAGE

自分が使わない方のモデルを「コメントアウトして無効」にしてから
catkin_make を実行する。
パッケージ配布時は６脚が有効、Nextageが無効になっているので
Nextageを使う人は忘れずに変更する。

