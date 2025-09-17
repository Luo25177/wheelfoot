function [ml, IL, Lw, Lb, theta] = GetLegBaryCenterAndPhiCompensate(len)

l1 = 0.18;
l2 = 0.18;
l1b = 0.09;
l2b = 0.09;
lx = 0;

m1 = 0.5;
m2 = 0.5;
ml = 1;
mb = 3;

I1 = 0.00275;
I2 = 0.00275;

phi2 = acos((l1 * l1 + l2 * l2 - len * len) / (2 * l1 * l2));
a1 = acos((l1 * l1 + len * len - l2 * l2) / (2 * l1 * len));

temp_x = (m1 + m2 + mb) * len * sin(a1) - m2 * l2b * cos(phi2);
temp_y = (m1 + m2 + mb) * len * cos(a1) - m2 * l2b * sin(phi2) - m2 * l1 - m1 * l1b;
phi1 = atan2(temp_y, temp_x);

theta = -phi1 - a1 + pi / 2;

x1 = l1b * cos(phi1);
y1 = l1b * sin(phi1);

x2 = l1 * cos(phi1) - l2b * sin(pi / 2 + phi1 - phi2);
y2 = l1 * sin(phi1) + l2b * cos(pi / 2 + phi1 - phi2);

yl = (y1 * m1 + y2 * m2) / (m1 + m2);
xl = yl / tan(phi1 + a1);

Lb = yl / sin(phi1 + a1);
Lw = len - Lb;

IL = I1 + m1 * (power(xl - x1, 2) + power(yl - y1, 2)) + I2 + m2 * (power(xl - x2, 2) * power(yl - y2, 2));

end
