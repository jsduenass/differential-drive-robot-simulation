%% Differential drive robot
% clean workspace
close ('all'), clear, clc

%% Variables Initialization

L=3;                  % Longitud 
r=1;                  % diametro de rueda (radio centro instantaneo de rotación)
phi1=pi/4;            % velocidad angular de rueda derecha [rad/ seg]
phi2=-pi/4;           % velocidad angular de rueda izquierda


x=0;
y=0;
theta=pi;
dt=0.01;
dy_prima=0;
dx_prima=0;
t=0:dt:100;

n=1000;
signal_phi1= zeros(1,n);
signal_phi2= zeros(1,n);


signal_phi1= [ones(1,300),-5*ones(1,100),ones(1,600)];
signal_phi2= [ones(1,300),ones(1,100),ones(1,600)];


M=r/2*[1,-L;1,L];     % Transference matrix between wheel speed and robots speed   
v_space=[];         % velocity space

variables=[];

for i=1:n

    phi1=signal_phi1(i);
    phi2=signal_phi2(i);

    v_space= M\[phi1; phi2];
    vc=v_space(1);
    omega= v_space(2);

    dx_prima=vc*dt;
    dy_prima=0;         % slip condition
    theta=theta+ omega*dt;

    % coordinate transformation
    [x,y]=Transform(x,y,dx_prima,dy_prima, theta);
    
    
    variables(i,:)=[x, y, theta, vc, omega];
end

x=variables(:,1);
y=variables(:,2);


plot(x,y)
ylim([-10,10])
xlim([-10,10])
axis('equal')
hold on
comet(x,y)
hold off

%% functions 
function R =rotation(theta)
    R=[cos(theta),-sin(theta); sin(theta), cos(theta)];
end

function [x, y]= Transform(x0,y0,x_prima,y_prima,theta)
    % coordinate transform
    R =rotation(theta);
    position=[x0;y0]+R*[x_prima;y_prima];
    x=position(1);
    y=position(2);
end



