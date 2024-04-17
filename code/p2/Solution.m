q_ranges = [
    0, 2*pi;
    0, 2*pi;
    0, 2*pi;
];
p = [
    0.45;
    0;
  ];
k = 1;
n_samples = 100;


[best_q, best_w] = Optimize(q_ranges, p, k, n_samples);
disp('best q')
disp(best_q)
disp('best p')
a1 = 0.2;
a2 = 0.15;
a3 = 0.12;
best_p = [
    a1*cos(best_q(1)) + a2*cos(best_q(1)+best_q(2)) + a3*cos(best_q(1)+best_q(2)+best_q(3));
    a1*sin(best_q(1)) + a2*sin(best_q(1)+best_q(2)) + a3*sin(best_q(1)+best_q(2)+best_q(3))
];
disp(best_p)
disp("refrence p")
disp(p)
disp('best w')
disp(best_w)

tf = 0;
tt = 0;
theta1dsim = [
    tt, best_q(1)
];
theta2dsim = [
    tt, best_q(2)
];
theta3dsim = [
    tt, best_q(3)
];