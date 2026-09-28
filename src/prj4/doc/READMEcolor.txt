gazebo で色が定義されているファイルは以下になる。
/usr/share/gazebo-11/media/materials/scripts/gazebo.material
独自の色を追加したい場合はここに追記する。
山の斜面の描画で必要になる色やテクスチャは独自に追加したデータである。

上記オリジナルの gazebo.material に定義されている色の名前一覧は
prj4/misc/color.txt
にコピーがあり、内容の確認ができる。
ただし prj4/misc/ のファイルの内容が直接プログラムのどこかで
使われているわけではない。実際に参照されるのは
/usr/share/gazebo-11/media/materials/scripts/gazebo.material
である。

物体の色が白くなるのは、色やテクスチャの設定が正しく読み出せてない
ためで、後から追加した色などで発生しやすい。
修正するには
prj4/sdf/scripts/gazebo.material.replace
を /usr/share/gazebo-11/media/materials/scripts/gazebo.material
としてコピーすればよい。
gazebo.material.replace には新しい色やテクスチャが登録されている。
上記コピーの作業は最初にインストールする時の
prj4/misc/prj4init.sh
を実行すると自動的に実行されるが、後から手動でコピーしても効果は
同じである。

prj4/misc/ には色を変更するツール chcolor と、
画像中の指定した位置の色を変えるプログラム ppmmark がある。
chcolor.sh と chcolor.c は機能が異なるので気をつける。
具体的な処理の内容はそれぞれのソースを見る。
