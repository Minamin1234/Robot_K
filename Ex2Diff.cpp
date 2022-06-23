//情報システム実験「ロボットプログラミング」
//例プログラム2: 目標への移動＋障害物回避
//2021/6/30 升谷 保博
#include "ise-robot.h"
#include <iostream>
#include <cmath>
#include <vector>
#include "draw.h"
#include "util.h"
using namespace std;

const int NumTarget = 2;
const Orthogonal targetList[NumTarget] = {{1, 3.5, 0, 0}, {2, 0, 0, 0}};
int iTarget;
bool pause;
bool first;
double goalTime;
enum State {Turn, Forward, Search, Avoid, Stop};
State state;
double avoidTime;
double wSearch;

//概要：行動決定のための初期設定
//引数：なし
//戻り値：なし
void setup()
{
  iTarget = 0;
  pause = true;
  first = true;
  goalTime = 0;
  state = Turn;
  avoidTime = 0;
  wSearch = 0;
}

//概要：行動決定のための毎回の処理
//引数：なし
//戻り値：なし
void loop()
{
  Velocity vel;
  double currentTime;
  getCartTime(currentTime); //現在の経過時間を記憶
  Orthogonal pose;
  getCartPose(pose);
  vector<Orthogonal> obstacle;
  getObstacle(obstacle);
  int key;
  getKey(key);
  if (key >= 0) { //キーが押されていれば
    if (pause && key == 'r') { //停止中で「r」であれば
      pause = false;
      if (first) { //初めてであれば
        resetLogging();
        first = false;
      }
    } else { //動作中か「r」でなければ
      pause = true;
    }
  }
  if (pause) { //停止中であれば
    drawString(-0.2,2,36,0,0,255,"r キーを押して開始");
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
  Polar tp =orthogonalToPolar(to);

  //衝突する可能性のある障害物を調べる
  Orthogonal nearest = {100,0,0,0};
  for (int i=0; i<obstacle.size(); i++) {
    if (abs(obstacle[i].y) < 0.3) {
      if (obstacle[i].x < nearest.x) {
        nearest = obstacle[i];
      }
    }
  }
  Orthogonal nw = localToWorld(nearest,pose);
  drawCircle(nw.x, nw.y, 0.05, 255, 0, 0);
  //cout << "nearest.x: " << nearest.x << endl;
  //状態遷移
  if (state == Stop) {
    if (tp.r > 0.2) { //目標が遠ければ
      state = Turn;
      cout << "Stop -> Turn" << endl;
    }
  } else if (state == Turn) {
    if (abs(tp.a) < 0.1) { //目標が正面にあれば
      state = Forward;
      cout << "Turn -> Forward" << endl;
    }
  } else if (state == Forward) {
    if (tp.r < 0.1) { //目標が近ければ
      state = Stop;
      cout << "Forward -> Stop" << endl;
    } else if (abs(tp.a) > 0.2) { //目標が正面になければ
      state = Turn;
      cout << "Forward -> Turn" << endl;
    } else if (nearest.x < 0.5) { //障害物が近ければ
      state = Search;
      cout << "Forward -> Search" << endl;
      if (nearest.y > 0) { //最近接障害物の位置によってSearchの回転方向を決める．
        wSearch = -0.5;
      } else {
        wSearch = 0.5;
      }
    }  
  } else if (state == Search) {
    if (nearest.x > 0.75) { //障害物が少し遠くなれば
      state = Avoid;
      cout << "Search -> Avoid" << endl;
      avoidTime = currentTime; //Avoidへ移る時刻を記録
    }
  } else if (state == Avoid) {
    if (currentTime - avoidTime > 4) { //Avoidを始めてから一定時間経過していれば
      state = Turn;
      cout << "Avoid -> Turn" << endl;
    } else if (nearest.x < 0.5) { //障害物が近ければ
      state = Search;
      cout << "Avoid -> Search" << endl;
      if (nearest.y > 0) {  //最近接障害物の位置によってSearchの回転方向を決める．
        wSearch = -0.5;
      } else {
        wSearch = 0.5;
      }
    }
  }

  //状態に応じた指令の決定
  if (state == Forward) {
    vel.v = 0.2;
    vel.w = 0;
  } else if (state == Turn) {
    vel.v = 0;
    if (tp.a > 0) { //左にあれば
      vel.w = 0.5;
    } else { //右にあれば
      vel.w = -0.5;
    }
  } else if (state == Search) {
    vel.v = 0;
    vel.w = wSearch;
  } else if (state == Avoid) {
    vel.v = 0.2;
    vel.w = 0;
  } else if (state == Stop) {
    vel.v = 0; //これを0以外の値にしないこと！
    vel.w = 0; //これを0以外の値にしないこと！
    if (iTarget < NumTarget-1) { //最後の目標でなければ
      iTarget++;
    } else { //最後の目標であれば
      drawString(0.75,2,36,0,0,255,"ゴール");
      if (goalTime == 0) { //初めてであれば
        goalTime = currentTime;
      }
      drawString(0.75,1.5,36,0,0,255,"%.1f秒",goalTime);
    }
  }
  setCartVelocity(vel);
}
