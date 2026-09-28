#! /bin/sh

cat URDFstart.urdf Body.urdf LeftLeg1.urdf LeftLeg2.urdf LeftHand1.urdf LeftHand2.urdf LeftLeg3.urdf LeftHand3.urdf RightLeg1.urdf RightHand1.urdf RightLeg2.urdf RightHand2.urdf RightLeg3.urdf RightHand3.urdf Neck.urdf FtSensorLF1.urdf FtSensorLF2.urdf FtSensorRF1.urdf FtSensorRF2.urdf HeadCamera.urdf HandCamera.urdf DepthCamera.urdf LIDAR.urdf URDFend.urdf > ../fturdf.urdf

# cat URDFstart.urdf ColorPoles.urdf URDFend.urdf > prg4env.urdf   # unnecessary now.

# Build urdffile and run
