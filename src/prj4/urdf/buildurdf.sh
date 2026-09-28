#! /bin/sh

cat URDFstart.urdf Body.urdf LeftLeg1.urdf LeftHand1.urdf LeftLeg2.urdf LeftHand2.urdf LeftLeg3.urdf LeftHand3.urdf RightLeg1.urdf RightHand1.urdf RightLeg2.urdf RightHand2.urdf RightLeg3.urdf RightHand3.urdf Neck.urdf HeadCamera.urdf DepthCamera.urdf DepthCameraSL.urdf DepthCameraSR.urdf ftall.urdf ft5.urdf URDFend.urdf > buildurdf.urdf

# cat URDFstart.urdf ColorPoles.urdf URDFend.urdf > prg4env.urdf   # unnecessary now.

# Build urdffile and run
