clear all
X=[0.2168 0.02329 0 0;-140 -2.1911 0 0;-1 0 0 0;0 0 0 0];
Y=[0 15;1.4e6 0;0 0;0 -1];
Q=[0.65 0 0 0;0 0 0 0;0 0 2.2 0;0 0 0 14];%Final 
%Q=[1.3 0 0 0;0 0 0 0;0 0 4.4 0;0 0 0 28]; %Multiplied by 2
%R=[22.26e4 0;0 2]; %Multiplied by 2
R=[11.13e4 0;0 1];
k=lqr(X,Y,Q,R);


