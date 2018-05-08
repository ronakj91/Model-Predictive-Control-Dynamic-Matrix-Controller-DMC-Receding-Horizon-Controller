% DMC MIMO Data Generator
clear
clc

T_sampling = 0.1;
Ts = 0.01;
u_old_1 = 0;
u_old_2 = 0;

%% Definition of transfer functions
s = tf('s');
K11 = (12.8*exp(-s))/(16.7*s+1);
K12 = (-18.9*exp(-3*s))/(21*s+1);
K21 = (6.6*exp(-7*s))/(10.9*s+1);
K22 = (-19.4*exp(-3*s))/(14.4*s+1);

t_vec = 0:T_sampling:199.9;

g11 = step(K11,t_vec);
g12 = step(K12,t_vec);
g21 = step(K21,t_vec);
g22 = step(K22,t_vec);


lambda=100;       %Tuning Factor
N = 1000; % Past control moves;
P=900;          %Prediction Horizon
M=600;          %Control Horizon
u1 = zeros(N,1); %Past control moves
u2 = zeros(N,1);

g11 = [g11;g11(end)*ones(P,1)];
g12 = [g12;g12(end)*ones(P,1)];
g21 = [g21;g21(end)*ones(P,1)];
g22 = [g22;g22(end)*ones(P,1)];

G11=zeros(P,M+1);
G12=zeros(P,M+1);
G21=zeros(P,M+1);
G22=zeros(P,M+1);

for k=1:M+1
    for i = k:P
         j = k;
         G11(i,j)= g11(i-k+1); 
         G12(i,j)= g12(i-k+1); 
         G21(i,j)= g21(i-k+1); 
         G22(i,j)= g22(i-k+1); 
    end
end

G = [G11 G12;G21 G22];

Fac=(inv(G'*G+lambda*eye(2*(M+1))))*(G');  % 2(M+1)x 2P

temp_g2_g11 = repmat(g11(1:N)',P,1);
temp_g2_g12 = repmat(g12(1:N)',P,1);
temp_g2_g21 = repmat(g21(1:N)',P,1);
temp_g2_g22 = repmat(g22(1:N)',P,1);

temp_g1_g11 = g11(2:P+1);
temp_g1_g12 = g12(2:P+1);
temp_g1_g21 = g21(2:P+1);
temp_g1_g22 = g22(2:P+1);


for i = 1:N-1
    temp_g1_g11 = [temp_g1_g11 g11(2+i:P+1+i)];
    temp_g1_g12 = [temp_g1_g12 g12(2+i:P+1+i)];
    temp_g1_g21 = [temp_g1_g21 g21(2+i:P+1+i)];
    temp_g1_g22 = [temp_g1_g22 g22(2+i:P+1+i)];
end

temp_f_11 = temp_g1_g11 - temp_g2_g11;
temp_f_12 = temp_g1_g12 - temp_g2_g12;
temp_f_21 = temp_g1_g21 - temp_g2_g21;
temp_f_22 = temp_g1_g22 - temp_g2_g22;

temp_f = [temp_f_11 temp_f_12;temp_f_21 temp_f_22];

u = [u1;u2];
save('dmc_mimo_data');