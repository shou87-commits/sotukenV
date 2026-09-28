
URDFファイルからSDFファイルに変換する方法

$ gz sdf -p abc.urdf > abc.sdf

$ ls -l
合計 16
-rw-r--r-- 1 any any 2369  6月 13 15:25 ChechSmallR.urdf // 小さなチェコ(URDF)
-rw-r--r-- 1 any any  267  6月 13 15:24 model.config
-rw-r--r-- 1 any any 1584  6月 13 18:39 model.sdf // 作成したチェコ(SDF)を修正
-rw-rw-r-- 1 any any 2179  6月 13 18:27 model.urdf.sdf　// urdf->sdf 不要部分削除
