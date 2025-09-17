clc
clear
syms x(t) theta(t) phi(t)
syms Tw Tb Pw Pb Nw Nb
syms x_ddot theta_ddot phi_ddot x_dot theta_dot phi_dot
syms mw R Iw mb Ib_y g Ic_z R_l l ml
syms L Il Lw Lb
% 这里的 mw, Iw, ml, Il 都是指的是两侧的总的参数 

func1 = [ml * diff(diff(x + Lw * sin(theta), t), t) == Nw - Nb;
    ml * diff(diff(Lw * cos(theta), t), t) == Pw - Pb - ml * g;
    mb * diff(diff(x + L * sin(theta) - l * sin(phi), t), t) == Nb;
    mb * diff(diff(L * cos(theta) + l * cos(phi), t), t) == Pb - mb * g;
    ];

[Pw, Pb, Nw, Nb] = solve(func1, [Pw, Pb, Nw, Nb]);

func2 = [diff(diff(x, t), t) == (Tw * R - Nw * R * R) / (Iw + mw * R * R);
    Il * diff(diff(theta, t), t) == Tb - Tw + (Pb * Lb + Pw * Lw) * sin(theta) - (Nb * Lb + Nw * Lw) * cos(theta);
    Ib_y * diff(diff(phi, t), t) == Tb + Nb * l * cos(phi) + Pb * l * sin(phi);];
 
func2 = subs(func2, ...
    [diff(diff(x, t), t), diff(diff(theta, t), t), diff(diff(phi, t), t), diff(x, t), diff(theta, t), diff(phi, t)], ...
    [x_ddot, theta_ddot, phi_ddot, x_dot, theta_dot, phi_dot]);

[x_ddot, theta_ddot, phi_ddot] = solve(func2, [x_ddot theta_ddot phi_ddot]);

X = [theta(t); theta_dot; x(t); x_dot; phi(t); phi_dot];
u = [Tw; Tb];
X_dot = [theta_dot; theta_ddot; x_dot; x_ddot; phi_dot; phi_ddot];
A = jacobian(X_dot, X);
B = jacobian(X_dot, u);
A = subs(A, [x_dot, theta(t), theta_dot, phi(t), phi_dot, Tw, Tb], zeros(1,7))
B = subs(B, [x_dot, theta(t), theta_dot, phi(t), phi_dot, Tw, Tb], zeros(1,7))
