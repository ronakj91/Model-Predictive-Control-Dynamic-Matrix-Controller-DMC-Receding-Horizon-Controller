%DMC SISO Data Generator 

clear;
close all;
clc;


N = 1700;       %no. of past control moves
Ts = 0.01;      %Integration Rate for plant
T_sampling = 0.1; %Sample rate for collection of step response coefficents


u_old = 0;      %This is the control which was implemented at previous
s = tf('s');    %instant
K11 = (12.8*exp(-s))/(16.7*s+1);    %Transfer Function
t_vec = 0:T_sampling:100;           %
g11 = step(K11,t_vec);              %step response coefficients are being
                                    %collected in y11

g11 = [g11;g11(end)*ones(N-length(g11),1)];


P=600;            %Prediction Horizon
M=200;            %Control Horizon
lambda=50;       %Tuning Factor
u = zeros(N,1);   %Past control moves
y_meas = 0;     
G=zeros(P,M+1);
g=g11;    %Step response coefficients for the plant has

 
for k=1:M+1
    for i = k:P
         j = k;
         G(i,j)= g(i-k+1);        
    end
end
Fac=(inv(G'*G+lambda*eye(M+1)))*(G');

g_1 = [g;g(N)*ones(P,1)];

temp_g2 = repmat(g_1(1:N)',P,1);
temp_g1 = g_1(2:P+1);
for i = 1:N-1
    temp_g1 = [temp_g1 g_1(2+i:P+1+i)];
end
temp_f = temp_g1 - temp_g2;