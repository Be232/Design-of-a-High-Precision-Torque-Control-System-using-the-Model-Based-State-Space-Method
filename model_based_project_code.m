L=0.00017;
R=0.43;
J_motor=0.0000209;
B_motor=0.0048157;
J_disk=0.0005;
B_disk=0.1;
Kt=0.0455;

% Electrical Part
num1=[1];
den1=[L R];
sys1=tf(num1,den1);
% Torque constant
sys2=Kt;
% Mechanical Part
num3=[1];
den3=[J_motor B_motor];
sys3=tf(num3,den3);
% Disk Part
num4=[1];
den4=[J_disk B_disk];
sys4=1/tf(num4,den4);
% integrator
num5=[1];
den5=[1 0];
sys5=tf(num5,den5);
G1=series(1/sys1,1/sys2);
G2=series(G1,sys4);
G3=series(sys1,sys2);
G4=series(G3,sys3);
G5=series(G4,sys5);
G=feedback(G5,G2);
step(G);
grid on;

% Durum uzayı matrisi ve kontrol edilebilirlik
A = [0,1,0;0,0,1;-1.21*10^6,-6.48*10^5,-2.78*10^3];
B = [0;0;2.28*10^9];
C = [0.0455,0,0];
D =0; % D matrisi 0 olarak ayarlandı


ctrb_matrix = ctrb(A, B);
disp('Determinant of the controllability matrix:');
disp(det(ctrb_matrix));

pc = [-8-10.62i,-8+10.62i,-25];
K = place(A, B, pc);
disp('State feedback gain K:');
disp(K);



syms k1 k2 k3 k4 s;
A_bar = [0,1,0,0;0,0,1,0;-1.21*10^6,-6.48*10^5,-2.78*10^3,0;-0.0455,0,0,0];
B_bar = [0;0;2.28*10^9;0];
K_bar = [k1, k2, k3,k4];
C_bar=[0.0455,0,0,0];
I = eye(4);
chareq = det(s * I - (A_bar - B_bar * K_bar));
disp('Characteristic Equation:');
disp(chareq);
K_bar1=[-5.267*10^-4,-2.8351*10^-4,-1.1886*10^-6,3.523*10^-4];

