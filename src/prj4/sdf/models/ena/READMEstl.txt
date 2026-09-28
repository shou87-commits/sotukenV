STL地形データの扱いについて

enamesh.stl.org, enamesh.bin.org の座標の分布
X座標は578.25〜614.75
Y座標は110.75〜146.25
Z座標は339.4〜357.51
大体３６ｍ四方を取り出している

Gazeboに読み込んで表示した時の角の座標
読み込み時にZ軸周りに180度回転している
ロボットがX軸正値を正面と見た時に
左下が３６，３６　　最も低い
左上が３６，０　　　低い
右下が０，３５　　　中くらい
右上が０，０　　　　高い

model.sdf で読み込ませる時に、scale を1.0以外にしたら衝突
検出ができなくなって、ロボットがすり抜けてしまったので注意する。

データの加工方法

x, y, z 座標の最大値、最小値を調べる

$ grep vertex enamesh.stl.org > zzz
$ awk '{print $2}' zzz| sort -n > zx
$ head zx
$ tail zx
$ awk '{print $3}' zzz| sort -n > zy
$ head zy
$ tail zy
$ awk '{print $4}' zzz| sort -n > zz
$ head zy
$ tail zy

x, y, z	座標から最小値を減算して原点付近に近づける

$ awk '{if($1~"vertex"){printf("  %s %.2f %.2f %.2f\n", $1, $2-578, $3-110, $4-339);}if ($1!~"vertex"){print $0}}' enamesh.stl.org > enamesh.stl.normal

指定した範囲の高度の地面をほぼ平（たいら）にする

$ awk '{if($1~"vertex"){if(12<$4&&$4<17){v4=12+($4-int($4))*0.3;}else{v4=$4;}printf("  %s %.2f %.2f %.2f\n", $1, $2, $3, v4);}if ($1!~"vertex"){print $0}}' enamesh.stl.normal > enamesh.stl.flat

森に生える木の作成

$ cd meshes
$ gcc -o makeforest makeforest.c
$ ./makeforest > f0.sdf

地形データと森のデータを統合する

$ cat foresthead.sdf f0.sdf foresttail.sdf > forest.sdf

meshes の中のファイル
enamesh.stlはforesthead.sdfから読み込まれる

-rw-rw-r-- 1 any any  354578  3月  4 16:08 enamesh.bin.org   バイナリの元データ、切り出した範囲
lrwxrwxrwx 1 any any      16  3月 26 15:46 enamesh.stl -> enamesh.stl.flat 外部からはこの名前で参照
-rw-rw-r-- 1 any any 1480463  3月 26 16:18 enamesh.stl.flat  normalの一部を平らにしたデータ
-rw-rw-r-- 1 any any 1480463  3月  4 17:13 enamesh.stl.normal 原点付近に座標を移動させたデータ
-rw-rw-r-- 1 any any 1545525  3月  4 16:41 enamesh.stl.org   テキストの元データ、切り出した範囲
