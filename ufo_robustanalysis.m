%% Initial Conditions
x0 = [pi;  % pi radians
      0;   % 0 rad/s
      0];  % 0 rad/s^2

% System Dynamics
A = [0    1; %%%Angulo
     0.01 0]; %%%%Velocidad angular
B = [0; 
     1];
C = [1 0];
D = 0;
sys = ss(A,B,C,D);
Gnom = tf(sys);
W = makeweight(.05,9,10);
Delta = ultidyn('Delta',[1 1]);
G = Gnom*(1+W*Delta);

xi = 0.707;
wn1 = 3;
wn2 = 7.5; 

Kp1 = 2*xi*wn1/5 - 1;
Ki1 = (wn1^2)/5;
C1 = tf([Kp1,Ki1],[1 0]);
T1 = feedback(G*C1,1);
t_final = 3;
step(T1,t_final)

%% Bode
opt = robOptions('Display','on');
stabmarg1 = robstab(T1,opt)
S1 = feedback(1,G*C1);
[maxgain1,wcu1] = wcgain(S1);
wcS1 = usubs(S1,wcu1);
bodemag(S1.NominalValue,'b',wcS1,'b')