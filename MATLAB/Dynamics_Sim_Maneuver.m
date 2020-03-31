clear all;
close all;
%% Goal of this sim: Simulation motion of the car

%% ICC and Radius of Curvature:
% Have user input a steering angle and then determine:
%   Angle of each individual wheels
%   Instantaneus Center of Curvature given current car position
%   Radius of curvature
%
% Have the user input a radius of curvature then determine:
%    Steering angle, and individual wheels
%    ICC

%% Forward Kinematics:
% Have the user enter
%    initial position and orientation (heading)
%    speed of car
%    time of movement
% Simulation should:
%     Draw the car moving as specified

%% Perform a given task (moguls)

%%%%%%%%%%%%%%%%%%%%%%%%%%% VARIABLES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global carBody carFrontAxle carRearAxle carLeftRearWheel carRightRearWheel carLeftFrontWheel carRightFrontWheel maneuver
h = figure; set(h, 'Visible', 'off');
hold on;
maneuver = 0;
% Axles
carBody = plot(0,0, '-g');
carFrontAxle = plot(0,0, '-g');
carRearAxle =  plot(0,0, '-g');

% Wheels
carLeftRearWheel = plot(0,0, '-k', 'LineWidth', 2);
carRightRearWheel = plot(0,0, '-k', 'LineWidth', 2);
carLeftFrontWheel = plot(0,0, '-k', 'LineWidth', 2);
carRightFrontWheel = plot(0,0, '-k', 'LineWidth', 2);

%%%%%%%%%%%%%%%%%% SIMULATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
carDynamicsType = input("How would you like the vehicle to determine the car dynamics? (1: Using Radius, 2: Using Steering Angle) ");
if carDynamicsType == 1
    R = input("Enter Radius in mm ");
    carDynamics = getCarDynamicsGivenRadius(R);
else
    phi = input("Enter Steering Angle (0 is center) ");
    carDynamics = getCarDynamicsGivenPhi(phi);
end
R = carDynamics(1);
phi = carDynamics(2);
phiR = carDynamics(3);
phiL = carDynamics(4);
s = input("Enter desires speed for forward kinematics: ");
t = input("Enter desired time for forward kinematics: ");
theta = input("Enter initial heading for forward kinematics: ");
set(h, 'Visible', 'on');
carParams = forwardKinematics(0, 0, phi,phiL,phiR,R,theta,s,t)

maneuver = input("Would you like to see a maneuver? (1: yes, 0: no)");

if(maneuver == 1)
    %Drive Forward
    carDynamics = getCarDynamicsGivenPhi(0);
    R = carDynamics(1);
    phi = carDynamics(2);
    phiR = carDynamics(3);
    phiL = carDynamics(4);
    carParams = forwardKinematics(carParams(1),carParams(2),phi,phiL,phiR,R,carParams(3),200,5);
    
    % Make a Circle
    carDynamics = getCarDynamicsGivenRadius(300);
    R = carDynamics(1);
    phi = carDynamics(2);
    phiR = carDynamics(3);
    phiL = carDynamics(4);
    carParams = forwardKinematics(carParams(1),carParams(2),phi,phiL,phiR,R,carParams(3),100,5);
    
    % Make a Circle (Other Way)
    carDynamics = getCarDynamicsGivenRadius(-300);
    R = carDynamics(1);
    phi = carDynamics(2);
    phiR = carDynamics(3);
    phiL = carDynamics(4);
    carParams = forwardKinematics(carParams(1),carParams(2),phi,phiL,phiR,R,carParams(3),100,5);
    
    %Drive Forward
    carDynamics = getCarDynamicsGivenPhi(0);
    R = carDynamics(1);
    phi = carDynamics(2);
    phiR = carDynamics(3);
    phiL = carDynamics(4);
    carParams = forwardKinematics(carParams(1),carParams(2),phi,phiL,phiR,R,carParams(3),100,10);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%
function carDynamics = getCarDynamicsGivenRadius(R)
W = 224; %mm
L = 212; %mm

disp("Given Radius:")
disp(R);

% Steering angle
phi = atand(L/R);
disp("Steering Angle:")
disp(phi);
% Find the angles of the individual wheels
phiRight = atand(L/(R - W/2));
phiLeft = atand(L/(R + W/2));

disp("Left Wheel Angle:")
disp(phiLeft);

disp("Right Wheel Angle:")
disp(phiRight);

carDynamics = [R phi phiRight phiLeft];
end

function carDynamics = getCarDynamicsGivenPhi(Phi)
W = 224; %mm
L = 212; %mm

% Steering angle
disp("Given Steering Angle:");
disp(Phi);

phi = Phi; %(degrees)
if(phi > 45)
    disp("Steering Angle locked to 45 degrees")
    phi = 45;
elseif (phi < -45)
    disp("Steering Angle locked to -45 degrees")
    phi = -45;
end

% Turning radius (R):
R = L/tand(phi);

disp("Radius:");
disp(R);

% Find the angles of the individual wheels
phiRight = atand(L/(R - W/2));
phiLeft = atand(L/(R + W/2));

disp("Left Wheel Angle:")
disp(phiLeft);

disp("Right Wheel Angle:")
disp(phiRight);

carDynamics = [R phi phiRight phiLeft];
end
%% Simulation Code starts here:
% Proporties of the car

function carParams = forwardKinematics(carXInit, carYInit, phi, phiLeft, phiRight, R, thetaInit, s, t)
global maneuver
W = 224; %mm
L = 212; %mm
carX = []; %initial x position
carY = []; %intiial y posiion
phi = phi; %initial steering angle
theta = thetaInit; %initial heading
speed = s;% mm/s
%% Finding ICC and R
% Car X and Y
carXinit = carXInit;
carYinit = carYInit;
carX = [carX carXinit];
carY = [carY carYinit];

%% Forward Kinematics
%sim = figure;
pbaspect([1 1 1])
hold on;
lastPos = plot(carX(1),carX(1), '-o','MarkerSize', 5, 'MarkerFaceColor', '#D9FFFF');

%This condition tests if we are demonstrating dynamics or showing off a
%maneuver
if(maneuver == 0)
    if phi == 0
        xlim([-2*s*t, 2*s*t]);
        ylim([-2*s*t, 2*s*t]);
    elseif R < 0
        xlim([3*R, -3*R]);
        ylim([3*R, -3*R]);
    else
        xlim([-3*R, 3*R]);
        ylim([-3*R, 3*R]);
    end
else
    xlim([-3000, 3000]);
    ylim([-3000, 3000]);
end

xlabel("X Axis (mm)")
ylabel("Y Axis (mm)")

% Calculate the ICC
ICCx = carX(1) - R*sind(theta)
ICCy = carY(1) + R*cosd(theta)

% Plot the ICC
ICC = plot(ICCx, ICCy, '-o','MarkerSize', 5);

% Plot the Trail
Trail = plot(carX, carY);

dt = .05; %seconds (time)
i = 0;

drawCar(carX(1), carY(1), theta, phiLeft, phiRight);

for j = 1:dt:t+1
    i = i + 1;
    % This plots the head of the trail
    lastPos.XData = carX(i);
    lastPos.YData = carY(i);
    
    drawCar(carX(i), carY(i), theta, phiLeft, phiRight);
    
    ICC.XData = ICCx;
    ICC.YData = ICCy;
    
    Trail.XData = carX;
    Trail.YData = carY;
    
    %At the end of the loop, update position
    Vx = cosd(theta)*speed;
    Vy = sind(theta)*speed;
    
    ThetaDot = (speed*tand(phi))/L; %ThetaDot in radians
    theta = theta + rad2deg(ThetaDot)*dt;
    
    
    carX = [carX (carX(i) + Vx*dt)];
    carY = [carY (carY(i) + Vy*dt)];
    %     ICCx = carX(i+1) - R*sind(theta);
    %     ICCy = carY(i+1) + R*cosd(theta);
    
    drawnow;
end
carParams = [carX(i+1) carY(i+1) theta];
end

function drawCar(carX, carY, theta, phiLeft, phiRight)
global carBody carFrontAxle carRearAxle carLeftRearWheel carRightRearWheel carLeftFrontWheel carRightFrontWheel
W = 224; %mm
L = 212; %mm
wheelWidth = 25;
middleOfRear = [carX, carY];
middleOfFront = [carX + L*cosd(theta), carY + L*sind(theta)];
middleOfFrontRightWheel = [middleOfFront(1) + W/2*cosd(theta + 90),middleOfFront(2) + W/2*sind(theta + 90)];
middleOfFrontLeftWheel = [middleOfFront(1) - W/2*cosd(theta + 90),middleOfFront(2) - W/2*sind(theta + 90)];
middleOfRearRightWheel = [middleOfRear(1) + W/2*cosd(theta + 90),middleOfRear(2) + W/2*sind(theta + 90)];
middleOfRearLeftWheel = [middleOfRear(1) - W/2*cosd(theta + 90),middleOfRear(2) - W/2*sind(theta + 90)];

% Body
carBody.XData = [middleOfRear(1), middleOfFront(1)];
carBody.YData = [middleOfRear(2), middleOfFront(2)];

% Axels
carFrontAxle.XData = [middleOfFrontRightWheel(1), middleOfFrontLeftWheel(1)];
carFrontAxle.YData = [middleOfFrontRightWheel(2), middleOfFrontLeftWheel(2)];
carRearAxle.XData = [middleOfRearRightWheel(1), middleOfRearLeftWheel(1)];
carRearAxle.YData = [middleOfRearRightWheel(2), middleOfRearLeftWheel(2)];

% Wheels
% Rear Wheels
carLeftRearWheel.XData = [middleOfRearLeftWheel(1) + wheelWidth*cosd(theta), middleOfRearLeftWheel(1) - wheelWidth*cosd(theta)];
carLeftRearWheel.YData = [middleOfRearLeftWheel(2) + wheelWidth*sind(theta), middleOfRearLeftWheel(2) - wheelWidth*sind(theta)];
carRightRearWheel.XData = [middleOfRearRightWheel(1) + wheelWidth*cosd(theta), middleOfRearRightWheel(1) - wheelWidth*cosd(theta)];
carRightRearWheel.YData = [middleOfRearRightWheel(2) + wheelWidth*sind(theta), middleOfRearRightWheel(2) - wheelWidth*sind(theta)];

% Front Wheels
carRightFrontWheel.XData = [middleOfFrontRightWheel(1) + wheelWidth*cosd(theta + phiRight), middleOfFrontRightWheel(1) - wheelWidth*cosd(theta + phiRight)];
carRightFrontWheel.YData = [middleOfFrontRightWheel(2) + wheelWidth*sind(theta + phiRight), middleOfFrontRightWheel(2) - wheelWidth*sind(theta + phiRight)];
carLeftFrontWheel.XData = [middleOfFrontLeftWheel(1) + wheelWidth*cosd(theta + phiLeft), middleOfFrontLeftWheel(1) - wheelWidth*cosd(theta + phiLeft)];
carLeftFrontWheel.YData = [middleOfFrontLeftWheel(2) + wheelWidth*sind(theta + phiLeft), middleOfFrontLeftWheel(2) - wheelWidth*sind(theta + phiLeft)];

end