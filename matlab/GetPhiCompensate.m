function [phi] = GetPhiCompensate(len)
angle2 = asin(l1 / 2 / len);

l1 = 0.18;
l2 = 0.18;

m1 = 0.5;
m2 = 0.5;
ml = 1;

I1 = 0.00275;
I2 = 0.00275;

L = 2 * l1 * sin(angle2 / 2);
phi = angle1 + 90 - angle2 / 2;

Lw = 0.5 * L;
Lb = 0.5 * L;

Il = I1 + m1 * 0.09 * 0.09 + I2 + m2 * 0.09 * 0.09;
