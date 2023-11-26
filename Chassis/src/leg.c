#include "leg.h"

//----
// @brief 初始化
// 
// @param leg 
// @param dir 腿的方向
//----
void legInit(Leg* leg, LegDir dir, DJmotor* wheel, Tmotor* front, Tmotor* behind) {
  leg->Fset = -(HALFMASSBODY + MASSLEG) * GRAVITY; // 虚拟力设定值的初始化
  leg->dir = dir;
  
  datastructInit(&leg->dis, 0, 0, 0, 0);
  // TODO: L0 初始位置 angle0的初始值
  datastructInit(&leg->L0, 0, 0, 0, 0);
  datastructInit(&leg->angle0, 0, 0, 0, 0);

  leg->Fnow = leg->Fset;
  leg->Tpset = 0;
  leg->TFset = 0;
  leg->TBset = 0;
  leg->TWheelset = 0;
  leg->normalforce = leg->Fset;

  // 电机绑定
  leg->wheel = wheel;
  leg->front = front;
  leg->behind = behind;

  
  inputInit(&leg->X);
  inputInit(&leg->Xd);
  outputInit(&leg->U);
}

//----
// @brief 计算L0和angle0的原值和导数值
// 
// @param leg 
// @param pitch 俯仰角 用于坐标系的转换
//----
void Zjie(Leg* leg, float pitch) {
  leg->angle1 = PI + leg->front->real.angleRad;
  leg->angle4 = leg->behind->real.angleRad;

  leg->xb = l1 * cos(leg->angle1) - l5 / 2;
  leg->yb = l1 * sin(leg->angle1);
  leg->xd = l5 / 2 + l4 * cos(leg->angle4);
  leg->yd = l4 * sin(leg->angle4);
  float lbd = sqrt(pow((leg->xd - leg->xb), 2) + pow((leg->yd - leg->yb), 2));
  float A0 = 2 * l2 * (leg->xd - leg->xb);
  float B0 = 2 * l2 * (leg->yd - leg->yb);
  float C0 = pow(l2, 2) + pow(lbd, 2) - pow(l3, 2);
  float D0 = pow(l3, 2) + pow(lbd, 2) - pow(l2, 2);
  leg->angle2 = 2 * atan((B0 + sqrt(pow(A0, 2) + pow(B0, 2) - pow(C0, 2))) / (A0 + C0));
  leg->angle3 = PI - 2 * atan((B0 + sqrt(pow(A0, 2) + pow(B0, 2) - pow(D0, 2))) / (A0 + D0));
  leg->xc = leg->xb + l2 * cos(leg->angle2);
  leg->yc = leg->yb + l2 * sin(leg->angle2);

  float dt = (float) (GolbalTimer - leg->timer) / 1000;
  leg->L0.now = sqrt(pow(leg->xc, 2) + pow(leg->yc, 2));
  
  // 乘以pitch的旋转矩阵
  float matrix_R[2][2] = {cos(pitch), -sin(pitch), sin(pitch), cos(pitch)};
  float cor_XY[2][1] = {leg->xc, leg->yc};
  float cor_XY_then[2][1];
  cor_XY_then[0][0] = matrix_R[0][0] * cor_XY[0][0] + matrix_R[0][1] * cor_XY[1][0];
  cor_XY_then[1][0] = matrix_R[1][0] * cor_XY[0][0] + matrix_R[1][1] * cor_XY[1][0];
  leg->angle0.last = leg->angle0.now;
  leg->angle0.now = atan(cor_XY_then[0][0] / cor_XY_then[1][0]);
  if(dt > 0) {
    leg->L0.dot = (leg->L0.now - leg->L0.last) / dt;
    leg->L0.ddot = (leg->L0.dot - leg->L0.lastdot) / dt;
    leg->L0.last = leg->L0.now;
    leg->L0.lastdot = leg->L0.dot;
    leg->angle0.dot = (leg->angle0.now - leg->angle0.last) / dt;
    leg->angle0.ddot = (leg->angle0.dot - leg->angle0.lastdot) / dt;
    leg->angle0.last = leg->angle0.now;
    leg->angle0.lastdot = leg->angle0.dot;
  }
}

//----
// @brief 腿部结构逆解，用于解算出脚端落点的对应的电机角度
// 
// @param leg 
// @param xc 
// @param yc 
//----
void Njie(Leg* leg, float xc, float yc) {
  float m, n, b, x1, y1;
  float A, B, C;

  A = 2 * l1 * yc;
  B = 2 * l1 * (xc + l5 / 2);
  C = l2 * l2 - l1 * l1 - xc * xc - yc * yc - l5 * l5 / 4 + xc * l5;
  leg->angle1set = 2 * atan((A + sqrt(A * A + B * B - C * C)) / (B - C));
  if (leg->angle1set < 0)
    leg->angle1set += 2 * PI;

  // nije_5(&angle1, (void *)0, x, y, l1, l6, l3, l4, l5); //利用L1,L6计算c1;
  m = l1 * cos(leg->angle1set);
  n = l1 * sin(leg->angle1set);
  b = 0;
  // x1 = l2 / l6 * ((x - m) * cos(b) - (y - n) * sin(b)) + m;
  // y1 = l2 / l6 * ((x - m) * sin(b) + (y - n) * cos(b)) + n; //得到闭链五杆端点的坐标
  x1 = ((xc - m) * cos(b) - (yc - n) * sin(b)) + m;
  y1 = ((xc - m) * sin(b) + (yc - n) * cos(b)) + n; // 得到闭链五杆端点的坐标

  A = 2 * y1 * l4;
  B = 2 * l4 * (x1 - l5 / 2);
  // c = l3 * l3 + 2 * l5 * x1 - l4 * l4 - l5 * l5 - x1 * x1 - y1 * y1;
  C = l3 * l3 + l5 * x1 - l4 * l4 - l5 * l5 / 4 - x1 * x1 - y1 * y1;
  leg->angle4set = 2 * atan((A - sqrt(A * A + B * B - C * C)) / (B - C));
  
  // TODO: 根据初始角度来纠正设定角度
  // leg->angle1set = PI - leg->angle1set - PI / 3;
  // leg->angle4set -= PI / 3;
  // nije_5((void *)0, &angle2, x1, y1, l1, l2, l3, l4, l5);        //计算c4 ,
}

//----
// @brief VMC 解算，虚拟力->实际电机扭矩
// 
// @param leg 
//----
void VMC(Leg* leg) {
  float trans[2][2] = {l1 * cos(leg->angle0.now + leg->angle3) * sin(leg->angle1 - leg->angle2) / sin(leg->angle2 - leg->angle3),
                       l1 * sin(leg->angle0.now + leg->angle3) * sin(leg->angle1 - leg->angle2) / (leg->L0.now * sin(leg->angle2 - leg->angle3)),
                       l4 * cos(leg->angle0.now + leg->angle2) * sin(leg->angle3 - leg->angle4) / sin(leg->angle2 - leg->angle3),
                       l4 * sin(leg->angle0.now + leg->angle2) * sin(leg->angle3 - leg->angle4) / (leg->L0.now * sin(leg->angle2 - leg->angle3))};
  leg->TFset = trans[0][0] * leg->Fset + trans[0][1] * leg->Tpset;
  leg->TBset = trans[1][0] * leg->Fset + trans[1][1] * leg->Tpset;
}

// TODO: 没测试过不一定没问题
//----
// @brief 逆向VMC解算 实际电机扭矩->虚拟力
// 
// @param leg 
//----
void INVMC(Leg* leg) {
  float trans[2][2] = {-(sin(leg->angle0.now + leg->angle2)*sin(leg->angle2 - leg->angle3))/(l1*cos(leg->angle0.now + leg->angle2)*sin(leg->angle0.now + leg->angle3)*sin(leg->angle1 - leg->angle2) - l1*cos(leg->angle0.now + leg->angle3)*sin(leg->angle0.now + leg->angle2)*sin(leg->angle1 - leg->angle2)), 
                      (sin(leg->angle0.now + leg->angle3)*sin(leg->angle2 - leg->angle3))/(l4*cos(leg->angle0.now + leg->angle2)*sin(leg->angle0.now + leg->angle3)*sin(leg->angle3 - leg->angle4) - l4*cos(leg->angle0.now + leg->angle3)*sin(leg->angle0.now + leg->angle2)*sin(leg->angle3 - leg->angle4)),
                      (leg->L0.now*cos(leg->angle0.now + leg->angle2)*sin(leg->angle2 - leg->angle3))/(l1*cos(leg->angle0.now + leg->angle2)*sin(leg->angle0.now + leg->angle3)*sin(leg->angle1 - leg->angle2) - l1*cos(leg->angle0.now + leg->angle3)*sin(leg->angle0.now + leg->angle2)*sin(leg->angle1 - leg->angle2)),
                      -(leg->L0.now*cos(leg->angle0.now + leg->angle3)*sin(leg->angle2 - leg->angle3))/(l4*cos(leg->angle0.now + leg->angle2)*sin(leg->angle0.now + leg->angle3)*sin(leg->angle3 - leg->angle4) - l4*cos(leg->angle0.now + leg->angle3)*sin(leg->angle0.now + leg->angle2)*sin(leg->angle3 - leg->angle4))};

  leg->Fnow = trans[0][0] * leg->TFnow + trans[0][1] * leg->TBnow;
  leg->Tpnow = trans[1][0] * leg->TFnow + trans[1][1] * leg->TBnow;
}
