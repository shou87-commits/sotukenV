#! /bin/sh

for file in box*.sdf
do
    sed s/1.01/0.501/ $file > tmp.sdf
    mv tmp.sdf $file
done
