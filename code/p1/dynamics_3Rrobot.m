%% dynamic equation of planar 2R robot
% symbolic derivation of Lagrange equations

close all;clear all;clc;

syms t g

%Link Parameters
l  = sym('l_%d', [1 3]).'; %length of link
ml  = sym('ml_%d', [1 3]).'; %masses of links
mj = sym('mj_%d', [1 3]).'; %masses of joints
Il_xx  = sym('Il_xx%d', [1 3]).';
Il_yy  = sym('Il_yy%d', [1 3]).';
Il_zz  = sym('Il_zz%d', [1 3]).'; %inertia of link about center assuming principle axes
Ij_xx = sym('Ij_xx%d', [1 3]);
Ij_yy  = sym('Ij_yy%d', [1 3]).';
Ij_zz  = sym('Ij_zz%d', [1 3]).'; %inertia of joints about center assuming principle axes

%Define System Coordinates and Derivatives
syms  q1(t)   q2(t)  q3(t)
syms  dq1(t)  dq2(t)  dq3(t)
syms ddx(t) ddy(t) ddz(t) ddq1(t) ddq2(t) ddq3(t)

q   = [q1(t) q2(t) q3(t)].';
dq  = [dq1(t) dq2(t) dq3(t)].';
ddq = [ddq1(t) ddq2(t) ddq3(t)].';
diff_q  = diff(q,t);
diff_dq = diff(dq,t);

%% Kinematics
p_l1 = [
    0;
    0;
    (1/2)*l(1);
];
p_j1 = [
    0;
    0;
    l(1);
];
p_l2 = [
    (1/2)*l(2)*cos(q(1))*cos(q(2));
    (1/2)*l(2)*sin(q(1))*cos(q(2));
    l(1) + (1/2)*l(2)*sin(q(2));
];
p_j2 = [
    l(2)*cos(q(1))*cos(q(2));
    l(2)*sin(q(1))*cos(q(2));
    l(1)+l(2)*sin(q(2));
];
p_l3 = [
    cos(q(1))*(l(2)*cos(q(2)) + (1/2)*l(3)*cos(q(2)+q(3)));
    sin(q(1))*(l(2)*cos(q(2)) + (1/2)*l(3)*cos(q(2)+q(3)));
    l(1) + l(2)*sin(q(2)) + (1/2)*l(3)*sin(q(2)+q(3));
];
p_j3 = [
    cos(q(1))*(l(2)*cos(q(2)) + l(3)*cos(q(2)+q(3)));
    sin(q(1))*(l(2)*cos(q(2)) + l(3)*cos(q(2)+q(3)));
    l(1) + l(2)*sin(q(2)) + l(3)*sin(q(2)+q(3));
];

dp_l2 = diff(p_l2, t);
dp_l2 = subs(dp_l2, diff_q, dq);
dp_j2 = diff(p_j2, t);
dp_j2 = subs(dp_j2, diff_q, dq);
dp_l3 = diff(p_l3, t);
dp_l3 = subs(dp_l3, diff_q, dq);
dp_j3 = diff(p_j3, t);
dp_j3 = subs(dp_j3, diff_q, dq);

%% Energy and Lagrangian
% Original
% KE_l1 = (1/2)*Il(1)*dq(1)^2;
% KE_j1 = (1/2)*Ij_2xx*dq(1)^2 + (1/2)*Ij(2)*dq(2)^2;
% KE_l2 = (1/2)*ml(2)*(dp_l2.')*dp_l2;
% KE_j2 = (1/2)*mj(2)*(dp_j2.')*dp_j2 + (1/2)*Ij(2)*dq(3)^2;
% KE_l3 = (1/2)*ml(3)*(dp_l3.')*dp_l3;
% KE_j3 = (1/2)*mj(3)*(dp_j3.')*dp_j3;

% Updated inertia
% Approximate non-main axis inertia with xx for link and xx for joint
KE_l1 = (1/2)*Il_zz(1)*dq(1)^2;
KE_j1 = (1/2)*Ij_yy(1)*dq(1)^2 + (1/2)*Ij_zz(1)*dq(2)^2;
KE_l2 = (1/2)*ml(2)*(dp_l2.')*dp_l2 + (1/2)*Il_yy(2)*dq(1)^2 + (1/2)*Il_zz(2)*dq(2)^2;
KE_j2 = (1/2)*mj(2)*(dp_j2.')*dp_j2 + (1/2)*Ij_yy(2)*dq(1)^2 + (1/2)*Ij_zz(2)*(dq(2) + dq(3))^2;
KE_l3 = (1/2)*ml(3)*(dp_l3.')*dp_l3 + (1/2)*Il_yy(3)*dq(1)^2 + (1/2)*Il_zz(3)*(dq(2) + dq(3))^2;
KE_j3 = (1/2)*mj(3)*(dp_j3.')*dp_j3 + (1/2)*Ij_yy(3)*dq(1)^2 + (1/2)*Il_zz(3)*(dq(2) + dq(3))^2;

PE_l1 = ml(1)*p_l1(3)*g;
PE_j1 = mj(1)*p_j1(3)*g;
PE_l2 = ml(2)*p_l2(3)*g;
PE_j2 = mj(2)*p_j2(3)*g;
PE_l3 = ml(3)*p_l3(3)*g;
PE_j3 = mj(3)*p_j3(3)*g;

KE = KE_l1 + KE_j1 + KE_l2 + KE_j2 + KE_l3 + KE_j3;
PE = PE_l1 - PE_j1 - PE_l2 - PE_j2 - PE_l3 - PE_j3;
% KE = KE_l1 + KE_l2 + KE_l3;
% PE = PE_l1 - PE_j1 - PE_l2 - PE_j2 - PE_l3 - PE_j3;
% PE = 0;
L = KE - PE;

%% Lagrange derivatives
dLddqdt = sym(zeros(3,1));
dLdq = sym(zeros(3,1));
 
for i = 1:length(q)
    dLdq(i) = functionalDerivative(L, q(i));
    tmp = functionalDerivative(L, dq(i));
    tmp = diff(tmp, t);
    dLddqdt(i) = subs(tmp, [diff_q diff_dq], [dq ddq]);
end
EOM = dLddqdt  - dLdq;
disp('Entire DEOM')
simplify(EOM)

%% h(q,dot q) only

for i = 1:length(q)
    dKdq(i) = functionalDerivative(KE, q(i));
    tmpK = functionalDerivative(KE, dq(i));
    tmpK = diff(tmpK, t);
    dLddqdtK(i) = subs(tmpK, [diff_q diff_dq], [dq zeros(size(diff_dq))]);
end
disp('h(q,dot q)')
EOMH = simplify(dLddqdtK-dKdq);
EOMH = transpose(EOMH);
disp(EOMH)

%% g(q) only

for i = 1:length(q)
    dPdq(i) = functionalDerivative(-PE, q(i));
    %tmpP = functionalDerivative(-PE, dq(i));
    %tmpP = diff(tmpP, t);
    %dLddqdtP(i) = subs(tmpP, [diff_q diff_dq], [0 0]);
end

disp('g(q)')
EOMG = simplify(-dPdq);
EOMG = transpose(EOMG);
disp(EOMG)

%% M(q) only
for i = 1:length(q)
    dKdq(i) = functionalDerivative(KE, q(i));
    tmpK = functionalDerivative(KE, dq(i));
    tmpK = diff(tmpK, t);
    dLddqdtK(i) = subs(tmpK, [diff_q diff_dq], [dq ddq]);
    dLddqdtK(i) = subs(dLddqdtK(i), dq, zeros(size(dq))); % Delete all time derivatives
    dKdq(i) = subs(dKdq(i), dq, zeros(size(dq))); % Delete all time derivatives
end
disp('M(q)')
EOMM = simplify(dLddqdtK-dKdq);
EOMM = transpose(EOMM);
disp(EOMM)
