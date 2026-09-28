木を植える

$ gcc -o makeforest makeforest.c
$ ./makeforest > forest.sdf
$ cat sdfhead.sdf forest.sdf sdftail.sdf > model.sdf

草を生やす

$ gcc -o makegrass makegrass.c
$ ./makegrass > grass.sdf
$ cat sdfhead.sdf grass.sdf sdftail.sdf > model.sdf

岩を置く

$ gcc -o makerocks makerocks.c
$ ./makerocks > rocks.sdf
$ cat sdfhead.sdf rocks.sdf sdftail.sdf > model.sdf

地面に段差を付ける

$ gcc -o makestep makestep.c
$ ./makestep > step.sdf
$ cat sdfhead.sdf step.sdf sdftail.sdf > model.sdf

全部のせ

$ cat sdfhead.sdf forest.sdf grass.sdf rocks.sdf step.sdf sdftail.sdf > model.sdf

上記のものは sdf の model の slope への配置を対象としている。
mysky.world は slope を表示するようになっている。

$ gazebo mysky.world
