#! /bin/sh

if [ 1 -ne $# ]
then
    echo Usage: ./chcolor.sh
    exit
fi

oldc1=RedTransparent
oldc2=GreenTransparent
oldc3=BlueTransparent
newc1=GreyTransparent
newc2=DarkGrey
newc3=Indigo
tmp=/tmp/tmpfile
files="LeftLeg1.urdf LeftLeg2.urdf LeftLeg3.urdf RightLeg1.urdf RightLeg2.urdf RightLeg3.urdf prg4Camera.urdf"

for file in $files
do
    cp $file $file.org
    sed -e "s/$oldc1/$newc1/g" -e "s/$oldc2/$newc2/g" -e "s/$oldc3/$newc3/g" ./$file > $tmp
    mv $tmp ./$file
done

# Usage: ./chcolor.sh

# Place this script in the same dir as *.urdf
