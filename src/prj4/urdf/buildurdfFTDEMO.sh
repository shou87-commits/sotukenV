#! /bin/sh

cat URDFstart.urdf Body.urdf LeftLeg1.urdf LeftHand1.urdf LeftLeg2.urdf LeftHand2.urdf LeftLeg3.urdf LeftHand3.urdf RightLeg1.urdf RightHand1.urdf RightLeg2.urdf RightHand2.urdf RightLeg3.urdf RightHand3.urdf Neck.urdf HeadCamera.urdf DepthCamera.urdf DepthCameraSL.urdf DepthCameraSR.urdf ftall.urdf ft5.urdf fta1s1.urdf fta2s1.urdf fta3s1.urdf fta4s1.urdf fta5s1.urdf fta6s1.urdf URDFend.urdf > buildurdf.urdf

# cat URDFstart.urdf ColorPoles.urdf URDFend.urdf > prg4env.urdf   # unnecessary now.

# Build urdffile and run
