clc
clear
close all

%% Exercise 11.1: Observer-Based State Feedback Controller Design

% Define system matrices
A = [-1 1; -1 -1];
B = [1; 1];
C = [1 1];
L = [2; -9];

% 11.1.1: Augmented system matrices
Ae = [A, zeros(2,1); C, 0];
Be = [B; 0];
Ce = [C, 0];

% 11.1.2: State feedback gain
Fe = -place(Ae, Be, [-1, -2, -3]);

% Extract feedback gain and observer gain
F = Fe(1:2);
FI = Fe(3);

% 11.1.3: Closed-loop system matrices
Acl = [A, B*F, B*FI; -L*C, A+B*F+L*C, B*FI; C, zeros(1,3)];
Bcl = [zeros(4,1); -1];
Ccl = [C, zeros(1,3)];
Dcl = 0;

% Create state-space system
sys = ss(Acl, Bcl, Ccl, Dcl);

% 11.1.5: Plot step response
figure
step(sys)
title('Step Response of the Closed Loop System')

%% Exercise 11.2: LQR Controller Design

% Define system matrices
A = [-1 1; -1 -1];
B = [1; 1];
C = [1 1];
D = 0;

% 11.2.1: Design LQR controller for different p values
p = [1, 10, 100];
for i = 1:length(p)
    disp(['Fopt with p value: ' num2str(p(i))])
    Fopt = -lqr(A, B, p(i)*(C')*C, 1);
    eig(A + B*Fopt)
end

% 11.2.2: Design LQR controller using Bryson's rule
Q = [1/1 0; 0 1/2];
R = 1/3;
Fopt = -lqr(A, B, Q, R);
disp('Fopt: Bryson''s rule')
eig(A + B*Fopt)

% 11.2.3: Plot step response of the closed-loop system with LQR controller
[K1,~,~] = lqr(A,B,Q,R);
sys = ss(A-B*K1,B,C,D);
figure
step(sys)
title('Step Response of the Closed Loop System')
