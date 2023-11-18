clc;
close all;
clear;
%% Setup Robot
% add a forth row to the DH table to calculate the end-effector position add 
% a forth frame for setup robot

% Definitions
robot = rigidBodyTree("DataFormat","column");
base = robot.Base;
rotating = rigidBody("rotating_base");
arm1 = rigidBody("arm1");
arm2 = rigidBody("arm2");
arm3 = rigidBody("arm3");
arm4 = rigidBody("arm4");
endeffector = rigidBody("endeffector");

% Design collisionboxes
collrota = collisionCylinder(0.03,0.1025);
collrota.Pose = trvec2tform([0 0 -0.1025/2]);
coll1 = collisionBox(0.105,0.025,0.053);  
coll1.Pose = trvec2tform([0.105/2 0 0]);  
coll2 = collisionBox(0.097,0.025,0.053);  
coll2.Pose = trvec2tform([0.097/2 0 0]);  
coll3 = collisionBox(0.05,0.06,0.05);  
coll3.Pose = trvec2tform([0.05/2 0.03 0]);  

% Design the endeffector % 6 by 3 mesh
tribox = zeros(6,3); 
tribox(1,:) = [0.02/2,  0.056/2,  0];
tribox(2,:) = [0.02/2, -0.056/2,  0];
tribox(3,:) = [-0.02/2,  0.056/2, 0];
tribox(4,:) = [-0.02/2, -0.056/2, 0];
tribox(5,:) = [0,  0.056/2, 0.105];
tribox(6,:) = [0, -0.056/2, 0.105];
% Mesh shape of endeffector
collEndeffector = collisionMesh(tribox);
% Center of endeffector
collEndeffector.Pose = trvec2tform([0 0 -0.105/2]); 

% Add Collision to the links
addCollision(rotating,collrota)           
addCollision(arm1,coll1)
addCollision(arm2,coll2)
addCollision(arm3,coll3)
addCollision(endeffector,collEndeffector)

% Set joints as revolute joints
jntrota = rigidBodyJoint("rotation_joint","revolute");    
jnt1 = rigidBodyJoint("jnt1","revolute");
jnt2 = rigidBodyJoint("jnt2","revolute");
jnt3 = rigidBodyJoint("jnt3","revolute");
jnt3.HomePosition = -pi/2;
jnt4 = rigidBodyJoint("jnt4","revolute");
jntend = rigidBodyJoint("endeffector_joint","revolute");

%          a        alpha   d       theta      modified DH table 
dhparams = [0   	0    	0.115  	0;
            0    	pi/2   0       0;
            0.105   0       0       0;
            0.097   0       0       0;
            0.03   -pi/2    0       0;
            0       0       0.105   0];
       
bodies = {base,rotating,arm1,arm2,arm3,arm4,endeffector};  
joints = {[],jntrota,jnt1,jnt2,jnt3,jnt4,jntend};

% Visualise the robot
figure("Name","Robot","Visible","on")               
for i = 2:length(bodies)
    setFixedTransform(joints{i},dhparams(i-1,:),"mdh");
    bodies{i}.Joint = joints{i};
    addBody(robot,bodies{i},bodies{i-1}.Name)
    show(robot,"Collisions","on","Frames","off");
    drawnow;     
end
figure;
show(robot)
%% 
% 
% Given final angle calculate the target position

% Setting final angle
finalconfig = [pi/2 pi/2 -pi/2 0 pi/2 0]';
syms th1 th2 th3 th4 th5 th6  

% Transformation Matrix that transforms vectors defined in frame 1 to their description in base frame 0
T01 = [cos(th1),  -sin(th1),    0,      0; 
       sin(th1),  cos(th1),     0,      0;
       0,         0,            1,      0.115;
       0,         0,            0,      1];
% Transformation Matrix that transforms vectors defined in frame 2 to their description in frame 1
T12 = [cos(th2),  -sin(th2),    0,      0; 
       0,          0,           -1,      0;
       sin(th2),  cos(th2),    0,      0;
       0,          0,           0,      1];
% Transformation Matrix that transforms vectors defined in frame 3 to their description in frame 2
T23 = [cos(th3),  -sin(th3),    0,      0.105; 
       sin(th3),   cos(th3),    0,      0;
       0,          0,           1,      0;
       0,          0,           0,      1];
% Transformation Matrix that transforms vectors defined in frame 4 to their description in frame 3
T34 = [cos(th4),   -sin(th4),   0,      0.097; 
       sin(th4),   cos(th4),    0,      0;
       0,          0,           1,      0;
       0,          0,           0,      1];
% Transformation Matrix that transforms vectors defined in frame 5 to their description in frame 4
T45 = [cos(th5),   -sin(th5),   0,      0.03; 
       0,          0,           1,      0;
       -sin(th5),   -cos(th5),    0,      0;
       0,          0,           0,      1];
% Transformation Matrix that transforms vectors defined in frame 6 to their description in frame 5
T56 = [1,           0,          0,      0; 
       0,           1,          0,          0;
       0,           0,          1,          0.165;
       0,           0,           0,         1];

% Transformation from frame 0 to frame 6
T06 = T01 * T12 * T23 * T34 * T45 * T56;
T = simplify(T06);  

% Translation Vector
Ts = T(1:3,4);

% Jacobian Matrix 5 by 3
JT = jacobian(Ts,[th1,th2,th3,th4,th5,th6]);             

% Substitute each angle from symbol to number in initial condition.
P1 = subs(Ts,th1,finalconfig(1));             
P2 = subs(P1,th2,finalconfig(2));
P3 = subs(P2,th3,finalconfig(3));
P4 = subs(P3,th4,finalconfig(4));
P5 = subs(P4,th5,finalconfig(5));
P6 = subs(P5,th6,finalconfig(6));

% Calculating the final position from given final angle, convert from symbolic type to double type.
p_target = double(P6);                       
disp(p_target)
%% 
% 
%% Robot Simulation & Creating Video

% Define the initial position
Initial = homeConfiguration(robot)
% Define the final position
Final = finalconfig;
Step = max(round(abs(Final - Initial)/pi*180));
Q = Final(1:6) - Initial;
delta_t = 1/30;

% angle and angle velocity store
Q_matrix(:, 1) = Initial;
V_matrix(:, 1) = [0;0;0;0;0;0];

% point position and linear velocity
Point_position_matrix(:, 1) = [0;0;0];
Linear_velocity_matrix = zeros(Step, 1);

figure
for i = 1:Step
    q = Initial + Q / Step * i;
    Q_matrix(:, i) = q;

    show(robot,q,"Collisions","on",'PreservePlot',false,"Frames","off");
    F(i) = getframe(gcf);
    drawnow
    if i >= 2
        V_matrix(:, i) = (Q_matrix(:, i) - Q_matrix(:, i-1))/delta_t;
    
    end

    T1 = subs(Ts,th1,q(1));
    T2 = subs(T1,th2,q(2));
    T3 = subs(T2,th3,q(3));
    T4 = subs(T3,th4,q(4));
    T5 = subs(T4,th5,q(5));
    T6 = subs(T5,th6,q(6));
    T_k = double(vpa(T6,12));         % Translation vector for newly computed theta_k, Convert from symbolic to numeric   
        
    Point_position_matrix(:, i) = T_k;
    if i >= 2
        Linear_velocity_matrix(i) = ((Point_position_matrix(1, i) - Point_position_matrix(1, i-1))^2 + ...
            (Point_position_matrix(2, i) - Point_position_matrix(2, i-1))^2 + ....
            (Point_position_matrix(3, i) - Point_position_matrix(3, i-1))^2)^0.5/delta_t;
    end
    hold on
    plot3(Point_position_matrix(1, i), Point_position_matrix(2, i), Point_position_matrix(3, i),'r.','MarkerSize',5)

end
% create the video writer with 1 fps
writerObj = VideoWriter('.\figure\task2_DH6.avi');
writerObj.FrameRate = 10;
% set the seconds per image
% open the video writer
open(writerObj);
% write the frames to the video
for i=1:length(F)
    % convert the image to a frame
    frame = F(i) ;    
    writeVideo(writerObj, frame);
end
% close the writer object
close(writerObj);
%% Plot angle, angle velocity, point position, linear velocity

% time = (1:step)./30;
time = (1:Step).*delta_t;
% Plot of joint angles during motion
a = figure
subplot(3,2,1)
plot(time,Q_matrix(1,:),'r--')
xlabel('Time [s]')
ylabel('\theta_1 [rad]')
xlim([time(1) time(end)]);
title("Angle for \theta_1")
subplot(3,2,2)
plot(time,Q_matrix(2,:),'r--')
xlabel('Time [s]')
ylabel('\theta_2 [rad]')
xlim([time(1) time(end)]);
title("Angle for \theta_2")
subplot(3,2,3)
plot(time,Q_matrix(3,:),'r--')
xlabel('Time [s]')
ylabel('\theta_3 [rad]')
xlim([time(1) time(end)]);
title("Angle for \theta_3")
subplot(3,2,4)
plot(time,Q_matrix(4,:),'r--')
xlabel('Time [s]')
ylabel('\theta_4 [rad]')
xlim([time(1) time(end)]);
title("Angle for \theta_4")
subplot(3,2,5)
plot(time,Q_matrix(5,:),'r--')
xlabel('Time [s]')
ylabel('\theta_5 [rad]')
xlim([time(1) time(end)]);
title("Angle for \theta_5")
subplot(3,2,6)
plot(time,Q_matrix(6,:),'r--')
xlabel('Time [s]')
ylabel('\theta_6 [rad]')
xlim([time(1) time(end)]);
title("Angle for \theta_6")
saveas(a,'.\figure\Task2_DH6_simulation_angle.png')

a = figure;
% Plot of joint velocities during motion
subplot(3,2,1)
plot(time(1:end),V_matrix(1,:),'r--')
xlabel('Time [s]')
ylabel('$$\frac{d\theta_1}{dt}[rad/s]$$','Interpreter','latex')
xlim([time(1) time(end)]);
title("Angle Velocity for \theta_1")
subplot(3,2,2)
plot(time(1:end),V_matrix(2,:),'r--')
xlabel('Time [s]')
ylabel('$$\frac{d\theta_2}{dt}[rad/s]$$','Interpreter','latex')
xlim([time(1) time(end)]);
title("Angle Velocity for \theta_2")
subplot(3,2,3)
plot(time(1:end),V_matrix(3,:),'r--')
xlabel('Time [s]')
ylabel('$$\frac{d\theta_3}{dt}[rad/s]$$','Interpreter','latex')
xlim([time(1) time(end)]);
title("Angle Velocity for \theta_3")
subplot(3,2,4)
plot(time(1:end),V_matrix(4,:),'r--')
xlabel('Time [s]')
ylabel('$$\frac{d\theta_4}{dt}[rad/s]$$','Interpreter','latex')
xlim([time(1) time(end)]);
title("Angle Velocity for \theta_4")
subplot(3,2,5)
plot(time(1:end),V_matrix(5,:),'r--')
xlabel('Time [s]')
ylabel('$$\frac{d\theta_5}{dt}[rad/s]$$','Interpreter','latex')
xlim([time(1) time(end)]);
title("Angle Velocity for \theta_5")
subplot(3,2,6)
plot(time(1:end),V_matrix(6,:),'r--')
xlabel('Time [s]')
ylabel('$$\frac{d\theta_6}{dt}[rad/s]$$','Interpreter','latex')
xlim([time(1) time(end)]);
title("Angle Velocity for \theta_6")
saveas(a,'.\figure\Task2_DH6_simulation_angle_velocity.png')

a = figure;
plot3(Point_position_matrix(1, :), Point_position_matrix(2, :), Point_position_matrix(3, :));
saveas(a,'.\figure\Task2_DH6_simulation_trajectory.png')

%% Plot Linear Velocity

a = figure;
plot(time(1:end), Linear_velocity_matrix)
xlabel('Time [s]')
ylabel("Linear Velocity of end-effector [m/s]")
xlim([0 max(time)])
ylim([0 max(Linear_velocity_matrix)*1.1])
title("Linear Velocity of end-effector")
saveas(a,'.\figure\Task2_DH6_simulation_linear_velocity.png')

compensation = [pi/2 0    pi/2 pi/2   pi/2] - [0    0    0     -pi/2 0]
% final_a = compensation' + Final(1:5)
%