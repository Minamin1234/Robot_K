//情報システム実験「ロボットプログラミング」
//プログラム1: 目標への移動
//HT21A099
//2022/6/8 南　李玖
#include "ise-robot.h"
#include <iostream>
#include <cmath>
#include "draw.h"
#include "util.h"
using namespace std;

const int NumTarget = 2;
const Orthogonal targetList[NumTarget] = { {1, 3.5, 0, 0}, {2, 0, 0, 0} };
int iTarget;
bool pause;
bool first;
double goalTime;
Velocity vel;

//このロボットが前進する最大速度
double maxSpeed = 0.3;

//このロボットが回転する最大回転速度
double maxTurnSpeed = 1.0;

//このロボットが前進・後退する際の加速度...値が小さい程加速が早くなる。<->値が大きいと加速が遅くなる。
int AccelSpeed = 10;

//徐々に加速し前進します。
void Forward(double& V)
{
    if (V >= maxSpeed)
    {
        V = maxSpeed;
        return;
    }
    auto addv = (maxSpeed - V) / AccelSpeed;
    V = V + addv > maxSpeed ? maxSpeed : V + addv;
}

//概要：行動決定のための初期設定
//引数：なし
//戻り値：なし
void setup()
{
    vel.v = vel.w = 0;
    iTarget = 0;
    pause = true;
    first = true;
    goalTime = 0;
}

//概要：行動決定のための毎回の処理
//引数：なし
//戻り値：なし
void loop()
{
    double currentTime;
    getCartTime(currentTime); //現在の経過時間を記憶
    Orthogonal pose;
    getCartPose(pose);
    int key;
    getKey(key);
    if (key >= 0) { //キーが押されていれば
        if (pause && key == 'r') { //停止中で「r」であれば
            pause = false;
            if (first) { //初めてであれば
                resetLogging();
                first = false;
            }
        }
        else { //動作中か「r」でなければ
            pause = true;
        }
    }
    if (pause) { //停止中であれば
        drawString(-0.2, 2, 36, 0, 0, 255, "r キーを押して開始");
        vel.v = 0; //これを0以外の値にしないこと！
        vel.w = 0; //これを0以外の値にしないこと！
        setCartVelocity(vel);
        return;
    }
    //目標に十字を描く
    drawCross(targetList[iTarget].x, targetList[iTarget].y, 255, 0, 0, 0.3);
    //ロボット座標系における目標位置
    Orthogonal to = worldToLocal(targetList[iTarget], pose);
    //極座標系へ変換（tp.r 距離，tp.a 角度）
    Polar tp = orthogonalToPolar(to);

    if (tp.r > 0.1) { //遠ければ
        if (abs(tp.a) > 0.05) { //正面になければ
            if (tp.a > 0) { //左にあれば
                vel.w = 1.0;
                Forward(vel.v);
            }
            else { //右にあれば
                vel.w = -1.0;
                Forward(vel.v);
            }
        }
        else { //正面にあれば
            Forward(vel.v);
            vel.w = 0;
        }
    }
    else { //近ければ

        if (iTarget < NumTarget - 1) { //最後の目標でなければ
            iTarget++;
        }
        else { //最後の目標であれば
            if (tp.r <= 0.06)
            {
                vel.v = 0; //これを0以外の値にしないこと！
                vel.w = 0; //これを0以外の値にしないこと！
                drawString(0.75, 2, 36, 0, 0, 255, "ゴール");
                if (goalTime == 0) { //初めてであれば
                    goalTime = currentTime;
                }
                drawString(0.75, 1.5, 36, 0, 0, 255, "%.1f秒", goalTime);
            }
        }
    }
    cout << "v: " << vel.v << ",w: " << vel.w << endl;
    setCartVelocity(vel);
}
