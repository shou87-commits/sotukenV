#! /bin/sh

cat URDFstart.urdf NBody.urdf LeftLeg1.urdf LeftHand1.urdf RightLeg1.urdf RightHand1.urdf Neck.urdf HeadCamera.urdf HandCamera.urdf DepthCamera.urdf LIDAR.urdf NHead.urdf URDFend.urdf > Nbuildurdf.urdf

# cat URDFstart.urdf ColorPoles.urdf URDFend.urdf > prg4env.urdf   # unnecessary now.

# Build urdffile and run
