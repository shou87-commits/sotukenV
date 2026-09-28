// horibe.cpp

#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include "const.h"
#include "motorclass.h"

void HoribeMotion(C1 motors[]) // 堀部君の梯子を掴む動き
{
    static int timer = 0;
    static int cycle = 400; // 目標角度を変える周期、4 秒
    static int mode = 2;
    static int motioncount = 0; // 選択する動きのセレクタ

    if (cycle == timer) { // cycle*0.01 秒ごとに目標角度を変更する
        timer = 0;

        motioncount = motioncount + 1;
//      printf("%d\n", motioncount);

	motors[A1J1].ctrlmode = mode;
	motors[A1J2].ctrlmode = mode;
	motors[A1J3].ctrlmode = mode;
	motors[A1J4].ctrlmode = mode;
	motors[A1J5].ctrlmode = mode;
	motors[A1J6].ctrlmode = mode;
	motors[A1J7].ctrlmode = mode;
	motors[A1J8].ctrlmode = mode;

	if (motioncount == 1) {
	    motors[A1J2].targetangle.data = M_PI / 2.0;
        }
        if (motioncount == 2) {
	    motors[A1J1].targetangle.data =-M_PI / 2.0;
	    motors[A1J4].targetangle.data = 0.0;
        }
        if (motioncount == 3) {
	    motors[A1J7].targetangle.data = M_PI / 2.0 ;
	    motors[A1J8].targetangle.data =-M_PI / 2.0 ;
        }
        if (motioncount == 4) {
	    motors[A1J2].targetangle.data = M_PI / 3.0 ;
	    motors[A1J3].targetangle.data =-M_PI / 2.5 ;
        }
        if (motioncount == 5) {
	    motors[A1J4].targetangle.data = 1.0;
        }
        if (motioncount == 6) {
	    motors[A1J7].targetangle.data = 0.0;
	    motors[A1J8].targetangle.data = 0.0;
        }
	if (motioncount == 7) {
	    motors[A1J1].targetangle.data = 0.0;
	    motors[A1J2].targetangle.data = 0.0;
	    motors[A1J3].targetangle.data = 0.0;
	    motors[A1J4].targetangle.data = 0.0;
	    motors[A1J5].targetangle.data = 0.0;
	    motors[A1J6].targetangle.data = 0.0;
	    motors[A1J7].targetangle.data = 0.0;
	    motors[A1J8].targetangle.data = 0.0;
	    motioncount = 0;
        }
    }
    timer++;
}
