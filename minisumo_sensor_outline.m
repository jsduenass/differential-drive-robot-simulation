clear, clc, close 

% addVisual plotTransforms addCollision
% Simulate Different Kinematic Models for Mobile Robots

%% Parameters
% distances [mm]

pos=[0,  0;   
    -1,  1];
angle=[20,-20];     % [degrees]
distance=[2.5,1];
sensorActivated=[true;true];
%%strategy= pointFollower
sensors.Position=pos;
sensors.angle=angle;
sensors.distance=distance;
sensors.Activated=sensorActivated;

%% simulate
state0=[0 0 0]';
tspan=linspace(0,40,100);
tspan=[0 20]
[t,states] = ode45(@(t,state)movement(t,state,sensors),tspan,state0);
xTraject=states(:,1);
yTraject=states(:,2);

%% Plot
% 'plane.stl'
figure(1)
subplot(2,2,1)
plot(t,xTraject)

subplot(2,2,2)
plot(t,yTraject)

subplot(2,1,2)
plot(xTraject,yTraject)


%% animate
figure(2)
n=size(states,1)

for k = 1:100:n
    translations=states(k,:);
    %rotations=zeros(size(translations,1),4);
    rotations=quaternion([0,0,0],'rotvec');
    %
    ax = plotTransforms(translations,rotations,"MeshFilePath","robot.stl","MeshColor",[0.6 0.6 0.6]);
    axis equal
    xline(0)
    yline(0)
    axis([-2 2 -2 2 -0.1 1])

    pause(0.5)

end


%%
[linearVelocity,angularVelocity,target,xBeam,yBeam]=pointFollower(sensors)




x_robot=[0,0,-1,-1,0,1,0]; 
y_robot=[0,1,1,-1,-1,0,1]; 

plot(x_robot,y_robot)                             % draw robot frame
hold on


quiver(pos(1,:),pos(2,:),xBeam,yBeam,'off')       % draw sensor signals

quiver(0,0,target(1),target(2),'off')             % draw target

 axis equal
xline(0)
yline(0)
axis([-2 2 -2 2])
hold off
%% functions

function [linearVelocity,angularVelocity,target,xBeam,yBeam]=pointFollower(sensors)
    kV=3;
    kOmega=1;

    sensorPosition=sensors.Position;
    sensorAngle=sensors.angle;
    sensorDistance=sensors.distance;
    sensorActivated=sensors.Activated;
   
    [xBeam,yBeam]=pol2cart(sensorAngle*pi/180,sensorDistance);
    
    beam=sensorPosition+[xBeam;yBeam];
    
    target=mean(beam(:,sensorActivated),2);
    
    [theta,r]=pol2cart(target(1),target(2));
    
    linearVelocity=kV*r;
    angularVelocity=kOmega*theta;


end

function dState=movement(t,state,sensors)
    x = state(1);
    y = state(2);
    [theta,rho]=cart2pol(x,y);
    [dRho,dTheta] = pointFollower(sensors);
    dx = dRho*cos(theta) - rho*sin(theta)*dTheta ;
    dy = dRho*sin(theta) + rho*cos(theta)*dTheta ;
    dz = 0 ;
    dState = [dx dy dz]';
end
