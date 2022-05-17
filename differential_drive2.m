close 'all', clear, clc

%% parametros 

% vector de coordenadas objetivo

x_obj=3*[ -3.5 -3.5 1.5 1.5 3.5 3.5 -2.5 -2.5 1.5 1.5 -1];
y_obj=-3*[ 0 3.5 3.5 -1.5 -1.5 -8 -8 -5.5 -5.5 -3.5 -3.5];

theta_obj= pi*linspace(0,1,length(x_obj));             
theta_obj=[pi, pi/2, 0, -pi/2, 0, -pi/2, pi, pi/2, 0, pi/2, pi]; 

Trayectoria=[x_obj', y_obj', theta_obj'];

%Trayectoria=[10, 0, -pi/2];

k=[0.05, 0.09, -0.03]        % k=[kx,k_a,k_b] constantes del controlador

E0=[0,0,0];

% tamaño del espacio de trabajo
space_x=30;
space_y=30;

dt=0.001;
variables=[];

%% trayectoria
% calculo de la trayectorio utilizando un cotrolador proporcional 
[n,m]=size(Trayectoria)

for i=1:n
    
    E_obj=Trayectoria(i,:)
    E_s= Transformada(E0,E_obj);            % diferencia entre posicion del robot y objetivo
    dx= E_s(1);    dy=E_s(2);   d_tetha=E_s(3);
    
    while max(abs([dx, dy]))>1  | mod(d_tetha,2*pi)> (10*pi/180)
    %for var_contadora=1:400
        E_s= Transformada(E0,E_obj);
        dx= E_s(1);    dy=E_s(2);   d_tetha=E_s(3);
    
        [E0,dE]=update(E0,E_s,k);

        variables(end+1,:)=[E0,E_s]; 
    end
   z=1; 
end



%% draw 
% simula el movimiento del robot y lo dibuja

%  persipectiva del robot 
% figure(1)
% x_s=variables(:,4)';
% y_s=variables(:,5)';
%comet(x_s,y_s)
% plot(x_s,y_s)
% 
% xlim([-20 20]); ylim([-20 20]);
% xline(0); yline(0);

x0=variables(:,1)';
y0=variables(:,2)';
theta=variables(:,3)';


% vector de movimiento
% 
% figure(1)
% t=linspace(1,5,length(x0));
% 
% subplot(2,2,1)
% plot(t,x0)
% ylim([-space_x space_x])
% 
% subplot(2,2,2)
% plot(t,y0)
% ylim([-space_y space_y])
% 
% 
% subplot(2,2,3)
% plot(t,theta*180/pi)
% %ylim([-180 180])


%% animacion
% save to file
filename = 'differential-robot.gif';


film=false;
% if(film)
%     myVideo = VideoWriter('movile_robot_animation.avi'); %open video file
%     myVideo.FrameRate = 30;  %can adjust this, 5 - 10 works well for me
%     open(myVideo)
% end

h=figure('Renderer', 'painters', 'Position', [50 50 800 500])
set(gca, 'OuterPosition', [0,0,1,1])
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

%laberinto
%x_lab=[0 -3.5 -3.5 1.5 1.5 3.5 3.5 -2.5 -2.5 1.5 1.5 -1]
%y_lab=[0 0 3.5 3.5 -1.5 -1.5 -8 -8 -5.5 -5.5 -3.5 -3.5]
x_lab=x_obj;
y_lab=y_obj
plot(x_lab,y_lab)

hold on

% trayectoria
plot(x0,y0)



for i=1:n 
    objective(Trayectoria(i,:))
end

% robot
x_prima=[0,0,-1,-1,0,1,0]; 
y_prima=[0,1,1,-1,-1,0,1]; 

[x, y]= Trans(x0(1),y0(1),theta(1),x_prima,y_prima);

robot = line(x,y,'LineWidth',2,'Color','g');
center = animatedline('lineStyle','--','Color','r');
xlim([-space_x space_x])
ylim([-space_y space_y])

 x_rob=[];
 y_rob=[];
 axis equal
for i=1:length(x0)
    
    [x, y]= Trans(x0(i),y0(i),theta(i),x_prima,y_prima);
    
    x_rob=[x_rob,x(1)]; y_rob=[y_rob,y(1)]  ;  
    
    robot.XData=[x]; robot.YData=[y];
    addpoints(center,x(1),y(1));
    
    % write to 
%     if (film)
%         frame = getframe(gcf); %get frame
%         writeVideo(myVideo, frame);
%     end 
    drawnow
    
    if (film & mod(i,5)==1)   
      % Capture the plot as an image 
      frame = getframe(h); 
      im = frame2im(frame); 
      [imind,cm] = rgb2ind(im,256); 
      % Write to the GIF File 
      if i == 1 
          imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
      else 
          imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',dt); 
      end 
    end

    %pause(dt)
end

save('robot_location.mat','x_rob','y_rob')

%% salidas -----------------------------------
variables=variables(1:10:end,:);
variables(:,3)=180/pi*variables(:,3);
variables(:,6)=180/pi*variables(:,6);

%variables



%% funciones ---------------------------------

function R=R_matrix(theta)
     R=[cos(theta), sin(theta), 0; 
        -sin(theta),  cos(theta), 0; 
              0   ,        0   , 1 ];
end

function E_s= Transformada(E0,E_g)
    theta=E0(3);
    
    E_s= R_matrix(theta)*(E_g - E0)';
    E_s=E_s';
end


function [E0,dE,vx,w]=update(E0,E_obj,k)

    %E0(3)=mod(E0(3)+pi,2*pi)-pi;
    kx=k(1);        k_a=k(2);      k_b= k(3);
    
    x=E_obj(1);     y=E_obj(2);   
    
    alpha=cart2pol(x,y);   
    
    betha= mod(E_obj(3)-alpha+pi,2*pi)-pi ;
    
    
    %alpha=mod(alpha,pi);
    
    
    
    vx= kx*x;
    w=  k_b*betha + k_a*alpha;
    
    theta=E0(3);
    dE=(R_matrix(-theta)*[vx, 0, w]')';
    
    E0=E0 + dE;
    
    
end

%draw functions

function objective(E_obj)
    x_obj=E_obj(1); y_obj=E_obj(2);     theta_obj=E_obj(3);

    punt_objetivo = plot(x_obj,y_obj,'o','MarkerFaceColor','red');
    L=1;
    [x, y]= Trans(x_obj,y_obj,theta_obj,[0,L],[0,0]);
    line(x,y,'LineWidth',1.5,'Color','k');
end

function [x, y]= Trans(x0,y0,theta,x_prima,y_prima)
    R=R_matrix(-theta);
    x=[];
    y=[];
    
    for i=1:length(y_prima)
        
        v=[x0;y0;0] + R*[x_prima(i);y_prima(i);0];
        x(i)=v(1);
        y(i)=v(2);
        %fprintf("Estas son las coordenadas x %4.4f y y %4.4f \n",v)
    end
    
end

