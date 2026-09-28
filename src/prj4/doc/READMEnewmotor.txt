
新しい関節とモータを追加する時に必要となる各ファイル中の記述の例

以下では左前脚のハンドに新しい指を追加する時に、それ用の関節、モータ、
コントローラ、構造を記述した例を示す。
前提として、標準の配布パッケージでは各脚とハンドでモータは8個ずつ、6本で
48個、首に2関節で合計50個になる。
※  その後腹下のハッチが追加され52個になったので ID 50,51 は使用された。
　　そのため以下の話はそのままでは通用しなくなった。(20260108追記)
51個めのモータを追加する際に新しく定義した名前は、以下の三個で
・番号が A1J7b
・コントローラの名前が a1j7b_pos_con
・URDF中の名前が NN17b
で、以下の記述がすべて必要となる。
新しく関節やモータを追加するときはどんな記述が必要か参考にするとよい。

const.h: const int A1J7b = 50; // ０からの通し番号、５１個めは５０になる
         const int NUMALLJOINTS = 51; // Joint の総数、手首や首を含む
                                      // モータコントローラは配列で用意される

motion.cpp:    case LF: j1 = A1J7; j2 = A1J8; j3 = A1J7b; j4 = A1J8b; break;

prg4.cpp:   C1((char*)"/prj4/a1j7b_pos_con/command", A1J7b, param), // ArmLeft1 Hand Finger1b

prg4_gz.launch:     a7j1_pos_con  a7j2_pos_con  a1j7b_pos_con  a1j8b_pos_con
		    a2j7b_pos_con a2j8b_pos_con a3j7b_pos_con  a3j8b_pos_con

prg4.yaml:  a1j7b_pos_con: // PID制御のゲイン

以下はurdf ファイル中で関連する記述を行単位で抜き出したっ例

LeftHand1.urdf:    <joint name="jointNN17b" type="revolute">
LeftHand1.urdf:   <child link="linkNN17b"/>
LeftHand1.urdf:    <link name="linkNN17b">
LeftHand1.urdf:    <gazebo reference="linkNN17b"> <material>Gazebo/Grey</material> </gazebo>
LeftHand1.urdf:    <transmission name="transNN17b">
LeftHand1.urdf:      <joint name="jointNN17b">
LeftHand1.urdf:      <actuator name="motorNN17b">
LeftHand1.urdf:        <parent link="linkNN17b"/>

なお LeftHand1.urdf 中の記述はロボット全体を表す buildurdf.urdf にも含まれる。
個々のパーツをロボット本体に統合する buildurdf.sh の実行を忘れないように気をつける。

----------------------------------------------------------------

もともと固定の設定になっていた関節にモータを追加する場合の設定の変更

上記の一連の追加に対して、以下の記述変更も必要になる。

URDFファイル中で固定の関節の場合は以下になるが、

    <joint name="tail_joint" type="fixed">
        <parent link="body"/>
        <child link="tail"/>
        <origin xyz= "-0.15 0 0.075" rpy="0 0 0" />
    </joint>

モータを追加する場合は以下のようになる。

    <joint name="tail_joint" type="revolute">
        <parent link="body"/>
        <child link="tail"/>
        <origin xyz= "-0.15 0 0.075" rpy="0 0 0" />
        <axis xyz="0 0 1"/>
        <limit effort="30" lower="-2.617" upper="2.617" velocity="1.571"/>
    </joint>

変更点は
・関節の type を fixed から revolute に変更する。
・回転の軸 axis を設定する。上記の例では Z 軸を指定している。
・モータのトルク effort と回転角度の上限と下限 lower, upper と速度 velocity
　を設定する。

これらの設定をしないと以下のエラーメッセージ（現在例なし）が出る。

