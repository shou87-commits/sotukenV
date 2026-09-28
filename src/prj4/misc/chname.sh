#! /bin/sh

if [ 2 -ne $# ]
then
    echo Usage: ./chname.sh newprojectname newprogramname
    exit
fi

oldprj=prj4
oldprg=prg4
newprj=$1
newprg=$2
tmp=/tmp/tmpfile
files="CMakeLists.txt package.xml config/$oldprg.yaml launch/${oldprg}_gz.launch launch/${oldprg}_rv.launch src/$oldprg.cpp src/$oldprg.h urdf/$oldprg.urdf"

for file in $files
do
    sed -e "s/$oldprj/$newprj/g" -e "s/$oldprg/$newprg/g" ./$oldprj/$file > $tmp
    mv $tmp ./$oldprj/$file
done

mv ./$oldprj/config/$oldprg.yaml ./$oldprj/config/$newprg.yaml
mv ./$oldprj/launch/${oldprg}_gz.launch ./$oldprj/launch/${newprg}_gz.launch
mv ./$oldprj/launch/${oldprg}_rv.launch ./$oldprj/launch/${newprg}_rv.launch
mv ./$oldprj/src/$oldprg.cpp ./$oldprj/src/$newprg.cpp
mv ./$oldprj/src/$oldprg.h ./$oldprj/src/$newprg.h
mv ./$oldprj/urdf/$oldprg.urdf ./$oldprj/urdf/$newprg.urdf

mv ./$oldprj/include/$oldprj ./$oldprj/include/$newprj
mv ./$oldprj ./$newprj

# Usage: ./chname.sh newprojectname newprogramname

# Place this script in the same dir as prj, and run.
