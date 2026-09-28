#! /bin/sh

if [ $# -ne 2 ]
then
	echo Usage: $0 sourcestr deststr
	exit 1
fi

for file in *.h *.cpp
do
    sed s/$1/$2/ $file > tmp.sdf
    mv tmp.sdf $file
done

# Replace sourcestr to deststr in all *.h *.cpp files
# usage : chstrall.sh sourcestr deststr
