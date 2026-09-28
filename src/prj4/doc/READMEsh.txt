sh, 対話的に動作を指示できるシェルの使い方

以下のように prg4 を起動した terminal から
$ rosrun prj4 prg4

sh を起動する文字 '$' を入力すると、次のように起動時の
メッセージとプロンプト prg4$ が表示される。
sh 起動時に実行していた prg4 のプログラムで指定されていた
動作は中断されロボットの動きは止まる。制御は sh に移り
sh の入力待ちになる。

shell opened
prg4$

このプロンプトに対して様々なコマンドを入力することで
様々な動作を対話的に実行できる。
ヘルプを表示するには、"help", 'H', 'h', '?' のいずれかを
入力する。すると以下のコマンド一覧が表示される。
一部の使用頻度が多いコマンドには１文字で起動できる省略形
が割り当てられている。( ) 内の表記（例 quit に対する q, Q）
が省略形を表す。

prg4$ ?

$              : open shell
quit (q, Q)    : close shell
!!             : repeat last command
sleep (S)      : suspend current motion
resume (R)     : resume last motion before entering shell
help (H, h, ?) : print help messages
init (I)       : init pose
ik nl x y z    : move leg by ik (ex. ik a1 0.5 0.5 0.5)
jump           : jump
mjda nj deg    : move joint degree absolute (ex. mjda a1j1 10)
mjra nj rad    : move joint radian absolute (ex. mjra a1j1 0.1)
mjdi nj deg    : move joint degree increment (ex. mjdi a1j1 10)
mjri nj rad    : move joint radian increment (ex. mjri a1j1 0.1)
pjd nj         : print joint degree (ex. pjd a2j4)
pjr nj         : print joint radian (ex. pjr a4j6)
rdyna filename : read dynamixel data
relax          : relax, reset all angles to 0 deg
stand          : stand up
stand2         : stand up another version
turn (T) l/r iter : turn (ex. turn r 3)
walk (W) f/b iter : walk (ex. walk f 5)
angles         : print angles(rad) for Dynamixel
allangles (A)  : print angles(deg) of all joints
dance (D) sec  : dance (ex. dance 10)
lesson         : lesson's original command
lesson2        : lesson's original command 2
lesson3        : lesson's original command 3


以下では各コマンドの機能とオプションの指定の仕方を説明する。

------------------------------------------------------------
$              : open shell
$ は sh を起動するためのコマンドでプロンプト prg4 に対しては
使用しない。sh を起動する前に実行していた動作は中断される。
中断するかどうかは shell.cpp の
const int  SHELL_LASTACT_SUSPEND = YES;
で設定できる。

------------------------------------------------------------
quit (q, Q)    : close shell
sh を終了して本来のプログラムの制御に戻す。sh を起動する前に
実行していて中断された動作を再開する。
再開するかどうかは shell.cpp の
const int  SHELL_LASTACT_RESUME  = YES;
で設定できる。

------------------------------------------------------------
!!             : repeat last command
!! を入力すると直前に実行したコマンドを再度実行する。

------------------------------------------------------------
何もコマンドを指定しないで EnterKey を押すと何もしない

------------------------------------------------------------
sleep (S)      : suspend current motion
プログラムの制御で実行される動きを中断する。
sh の起動時に内部で一度このコマンドが実行されている。

------------------------------------------------------------
resume (R)     : resume last motion before entering shell
中断されているプログラムによる動きを再開する。
再開後はプログラムによる動作をしながら sh からの動作の
指示も受け付けて実行される。

------------------------------------------------------------
help (H, h, ?) : print help messages
コマンドの一覧を表示する。

------------------------------------------------------------
init (I)       : init pose
初期姿勢にする。プログラム起動直後の姿勢と同じ。

------------------------------------------------------------
ik nl x y z    : move leg by inverse kinematics
指定した脚を逆運動学で目標座標まで動かす。
nl は脚の番号(1 -- 6, または次の名前による指定
a1 -- a6)。x, y, z はローカル座標系の目標座標、
手首の根本または指先のどちらを
目標座標に到達させるかはそれまでの指定による。
例
prg4$ ik a1 0.5 0.1 0.3

------------------------------------------------------------
jump           : jump
ロボットを上方にジャンプさせる。未完成なのでまだ使えない。

------------------------------------------------------------
mjda nj deg    : move joint degree absolute
指定したジョイントを指定した角度に回転させる。
nj はジョイントの番号(1 --50, または次の名前による指定
a1j1 -- a6j8, a7j1, a7j2)。deg は目標角度
例
prg4$ mjda a1j3 -45

------------------------------------------------------------
mjra nj rad    : move joint radian absolute
指定したジョイントを指定した角度に回転させる。
nj はジョイントの番号(1 --50, または次の名前による指定
a1j1 -- a6j8, a7j1, a7j2)。rad はラディアンによる目標角度
例
prg4$ mjra a1j4 -3.1416

------------------------------------------------------------
mjdi nj deg    : move joint degree increment
指定したジョイントを現在の角度から指定した角度分回転させる。
nj はジョイントの番号(1 --50, または次の名前による指定
a1j1 -- a6j8, a7j1, a7j2)。deg は現在の角度からの回転角度
例
prg4$ mjdi a1j4 30

------------------------------------------------------------
mjri nj rad    : move joint radian increment
指定したジョイントを現在の角度から指定した角度分回転させる。
nj はジョイントの番号(1 --50, または次の名前による指定
a1j1 -- a6j8, a7j1, a7j2)。rad は現在の角度からのラディアン
による回転角度
例
prg4$ mjri a1j4	1.5708

------------------------------------------------------------
pjd nj         : print joint degree
指定したジョイントの現在の角度を表示する。
nj はジョイントの番号(1 --50, または次の名前による指定
a1j1 -- a6j8, a7j1, a7j2)。
例
prg4$ pjd a1j4

------------------------------------------------------------
pjr nj         : print joint radian
指定したジョイントの現在の角度をラディアンで表示する。
nj はジョイントの番号(1 --50, または次の名前による指定
a1j1 -- a6j8, a7j1, a7j2)。
例
prg4$ pjr a1j4

------------------------------------------------------------
stand          : stand up
機体がひっくり返ってしまった時に立ち上がらせる。

------------------------------------------------------------
stand2         : stand up another version
機体がひっくり返ってしまった時に別の動きで立ち上がらせる。

------------------------------------------------------------
turn l/r iter  : turn
ロボットをその場で旋回させて向きを変える。
l/r は旋回する向きで左右のどちらかを指定する。
iter は単位となる動きの繰り返しの回数を指定する。
例
prg4$ turn l 3

------------------------------------------------------------
walk f/b iter  : walk
ロボットを直進させる。
f/b は進行方向で前後のどちらかを指定する。
iter は単位となる動きの繰り返しの回数を指定する。
例
prg4$ walk f 2

------------------------------------------------------------
dance sec      : dance
逆運動学による滑らかなダンスの動きをする。
sec は継続時間を秒で指定する。
例
prg4$ dance 30

------------------------------------------------------------
angles         : print angles(rad) for Dynamixel
実機のDynamixel用に、肩と肘の関節角度を専用の書式で出力する。
単位は radian

------------------------------------------------------------
allangles (A)  : print angles(deg) of all joints
すべての関節角度を脚1本分ずつ改行して出力する。
単位が degree

------------------------------------------------------------
lesson         : lesson's original command
ユーザが独自で作成したコマンドを追加できる。
オプションの指定も可能

------------------------------------------------------------
lesson2        : lesson's original command 2
ユーザが独自で作成したコマンドを追加できる。
オプションの指定も可能

------------------------------------------------------------
lesson3        : lesson's original command 3
ユーザが独自で作成したコマンドを追加できる。
オプションの指定も可能
