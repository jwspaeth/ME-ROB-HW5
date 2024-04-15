function mass_term = CalculateMassTerm(joint_angles)
%CALCULATEMASSTERM Summary of this function goes here
%   Detailed explanation goes here
Il_1 = 0.0321699;
Ij_2xx = 0.01472;
Ij_2zz = 0.015708;
ml_2 = 18.75;
ml_3 = 18.75;
mj_2 = 3.14159;
mj_3 = 7.53982;
l_2 = 0.2;
l_3 = 1;
q2 = joint_angles(2);
q3 = joint_angles(3);

mass_term = zeros(3,3);
mass_term(1,1) = 8*Ij_2xx + 8*Il_1 + 4*l_2^2*mj_2 + 4*l_2^2*mj_3 + 4*l_3^2*mj_3 + l_2^2*ml_2 + 4*l_2^2*ml_3 + l_3^2*ml_3 + 4*l_2^2*mj_2*cos(2*q2) + 4*l_2^2*mj_3*cos(2*q2) + l_2^2*ml_2*cos(2*q2) + 4*l_2^2*ml_3*cos(2*q2) + 4*l_3^2*mj_3*cos(2*q2 + 2*q3) + l_3^2*ml_3*cos(2*q2 + 2*q3) + 8*l_2*l_3*mj_3*cos(2*q2 + q3) + 4*l_2*l_3*ml_3*cos(2*q2 + q3) + 8*l_2*l_3*mj_3*cos(q3) + 4*l_2*l_3*ml_3*cos(q3)/8;
mass_term(1,2) = 0;
mass_term(1,3) = 0;

mass_term(2,1) = 0;
mass_term(2,2) = Ij_2zz + l_2^2*mj_2 + l_2^2*mj_3 + l_3^2*mj_3 + (l_2^2*ml_2)/4 + l_2^2*ml_3 + (l_3^2*ml_3)/4 + 2*l_2*l_3*mj_3*cos(q3) + l_2*l_3*ml_3*cos(q3);
mass_term(2,3) = l_3^2*mj_3 + (l_3^2*ml_3)/4 + l_2*l_3*mj_3*cos(q3) + (l_2*l_3*ml_3*cos(q3))/2;

mass_term(3,1) = 0;
mass_term(3,2) = l_3^2*mj_3 + (l_3^2*ml_3)/4 + l_2*l_3*mj_3*cos(q3) + (l_2*l_3*ml_3*cos(q3))/2;
mass_term(3,3) = Ij_2zz + l_3^2*mj_3 + (l_3^2*ml_3)/4;

end

