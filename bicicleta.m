close 'all', clc, clear
% bicicleta

% rueda delantera
L1=5; a1=0; b1=0; r1=1; phi=2;  v1=r1*phi;
b1=b1+pi/4;

% rueda trasera
L2=5; a2=pi; b2=pi;


T2=transformada(L2,a2,b2);
T1=transformada(L1,a1,b1);

J1=[T2(1,:) ; T2(2,:);T1(2,:)];
J2=[v1;0;0];
e_R=J1^(-1)*J2;

e_2=T2*e_R;

e_1=T1*e_R;

disp("      e_R ,     e_2,       e_1")
[e_R,e_2,e_1]

e_1=rotation(-a1-b1)*e_1;
e_2=rotation(-a2-b2)*e_2;

[e_R,e_2,e_1]
figure()
axis([-10,10,-10,10])
grid on

drawArrow = @(x,y) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0,'linewidth',1.5,'MaxHeadSize',2.5,'color','k')
 hold on

x=[L1,L1+e_1(1)];       y=[0,e_1(2)];
drawArrow(x,y)