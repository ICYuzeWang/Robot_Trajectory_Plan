clc;
close all;
clear;
%% Setup Robot
% add a forth row to the DH table to calculate the end-effector position add 
% a forth frame for setup robot

robot = rigidBodyTree("DataFormat","column");
base = robot.Base;
rotating = rigidBody("rotating_base");
arm1 = rigidBody("arm1");
arm2 = rigidBody("arm2");
arm3 = rigidBody("arm3");
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

tribox = zeros(6,3); %6 by 3 mesh
tribox(1,:) = [0.02/2,  0.056/2,  0];
tribox(2,:) = [0.02/2, -0.056/2,  0];
tribox(3,:) = [-0.02/2,  0.056/2, 0];
tribox(4,:) = [-0.02/2, -0.056/2, 0];
tribox(5,:) = [0,  0.056/2, 0.105];
tribox(6,:) = [0, -0.056/2, 0.105];
collEndeffector = collisionMesh(tribox); % Mesh shape
% collEndeffector = collisionBox(0.105,0.06,0.025);
collEndeffector.Pose = trvec2tform([0 0 0.06]); % centre

addCollision(rotating,collrota)           % add Collision to the links
addCollision(arm1,coll1)
addCollision(arm2,coll2)
addCollision(arm3,coll3)
addCollision(endeffector,collEndeffector)

% set joints as revolute joints
jntrota = rigidBodyJoint("rotation_joint","revolute");    
jnt1 = rigidBodyJoint("jnt1","revolute");
% jnt1.HomePosition = pi/2;
jnt2 = rigidBodyJoint("jnt2","revolute");
% jnt1.HomePosition = pi/2;
jnt3 = rigidBodyJoint("jnt3","revolute");
jnt3.HomePosition = -pi/2;
jntend = rigidBodyJoint("endeffector_joint","revolute");
% jntend.HomePosition = pi/2;

%          a        alpha   d       theta      modified DH table 
dhparams = [0   	0    	0.115  	0;
            0    	pi/2    0       0;
            0.105   0       0       0;
            0.097   0       0       0;
            0.03   -pi/2    0       0];
       
bodies = {base,rotating,arm1,arm2,arm3,endeffector};  
joints = {[],jntrota,jnt1,jnt2,jnt3,jntend};

figure("Name","Robot","Visible","on")               % Visualise the robot
for i = 2:length(bodies)
    setFixedTransform(joints{i},dhparams(i-1,:),"mdh");
    bodies{i}.Joint = joints{i};
    addBody(robot,bodies{i},bodies{i-1}.Name)
    show(robot,"Collisions","on","Frames","off");
    drawnow;     
end
figure;
show(robot)
%% Robot Simulation & Creating Video

homeconfig = homeConfiguration(robot)
% Initial = homeconfig;
% % Final = theta_final;
% % Final = [pi/2 0 0 0 0]';
% % Final = [pi/2 pi/2 -pi/2 0 0]';
% Final = [pi/2 pi/2 -pi/2 0 pi/2]';
% Step = 90;
% framesPerSecond = 15;
% Q = Final(1:5) - Initial;
% % Q = Final - theta_k_1;
% figure
% for i = 1:Step
%     q = Initial + Q / Step * i;
%     show(robot,q,"Collisions","on","Frames","off");
%     F(i) = getframe(gcf);
%     drawnow
% end
% % create the video writer with 1 fps
% writerObj = VideoWriter('myVideo.avi');
% writerObj.FrameRate = 10;
% % set the seconds per image
% % open the video writer
% open(writerObj);
% % write the frames to the video
% for i=1:length(F)
%     % convert the image to a frame
%     frame = F(i) ;    
%     writeVideo(writerObj, frame);
% end
% % close the writer object
% close(writerObj);

% Task 3:
% Find the end-effector position from the given pose 
% 
% Derive the Transformation matrix of forward kinematics using *Symbolic Math 
% Toolbox*
% Given final angle calculate the target position

homeconfig = homeConfiguration(robot);
% finalconfig = [pi/4, pi/4, pi/4, 0, 0]';     % Setting final angle
% homing_pos= [90 0 90 90 90 90];
% homing_pos= [pi/2 0 0 0 0];
% homing_pos= [0 pi/2 0 0 0];
% homing_pos= [pi/2 pi/4 0 0 0];
homing_pos= [pi/2 pi/2 -pi/2 0 0];
% homing_pos= [-pi/2 pi/2 pi/2 -pi/2 0];
finalconfig = homing_pos;
syms th1 th2 th3 th4 th5   % Do not forget to install "Symbolic Math Toolbox"

T01 = [cos(th1),  -sin(th1),    0,      0; % Transformation Matrix that transforms vectors defined in frame 1 to their description in base frame 0
       sin(th1),  cos(th1),     0,      0;
       0,         0,            1,      0.115;
       0,         0,            0,      1];
T12 = [cos(th2),  -sin(th2),    0,      0; % Transformation Matrix that transforms vectors defined in frame 2 to their description in frame 1
       0,          0,           -1,      0;
       sin(th2),  cos(th2),    0,      0;
       0,          0,           0,      1];
T23 = [cos(th3),  -sin(th3),    0,      0.105; % Transformation Matrix that transforms vectors defined in frame 3 to their description in frame 2
       sin(th3),   cos(th3),    0,      0;
       0,          0,           1,      0;
       0,          0,           0,      1];
T34 = [cos(th4),   -sin(th4),   0,      0.097; % Transformation Matrix that transforms vectors defined in frame 3 to their description in frame 2
       sin(th4),   cos(th4),    0,      0;
       0,          0,           1,      0;
       0,          0,           0,      1];
T45 = [cos(th5),   -sin(th5),   0,      0.03; % Transformation Matrix that transforms vectors defined in frame 3 to their description in frame 2
       0,          0,           1,      0;
       -sin(th5),   -cos(th5),    0,      0;
       0,          0,           0,      1];
   
T05 = T01 * T12 * T23 * T34 * T45;
T = simplify(T05);                       % Transformation Matrix that transforms vectors defined in frame 4 to their description in base frame 0

Ts = T(1:3,4);                               % Translation Vector
JT = jacobian(Ts,[th1,th2,th3,th4,th5]);             % Jacobian Matrix 5 by 3

P1 = subs(Ts,th1,finalconfig(1));            % Substitute each angle from symbol to number in initial condition. 
P2 = subs(P1,th2,finalconfig(2));
P3 = subs(P2,th3,finalconfig(3));
P4 = subs(P3,th4,finalconfig(4));
P5 = subs(P4,th5,finalconfig(5));
p_target = double(P5);                       % Calculating the final position from given final angle, convert from symbolic type to double type.

disp(p_target)

%% 
% Display forward 

homeconfig = homeConfiguration(robot)
Initial = homeconfig;
% Final = theta_final;
% Final = [pi/2 0 0 0 0]';
% Final = [pi/2 pi/2 -pi/2 0 0]';
Step = 90;
framesPerSecond = 15;
Q = homing_pos' - Initial;
% Q = Final - theta_k_1;
figure
for i = 1:Step
    q = Initial + Q / Step * i;
    show(robot,q,"Collisions","on","Frames","off");
    F(i) = getframe(gcf);
    drawnow
end
% create the video writer with 1 fps
writerObj = VideoWriter('myVideo.avi');
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
%% 
% Task 3: 
% 
% Calculate the Jacobian matrix of kinematic expressions

TS = T(1:3,4);                               % Translation Vector
JT = jacobian(Ts,[th1,th2,th3,th4,th5]);             % Jacobian Matrix 5 by 3

P1 = subs(Ts,th1,finalconfig(1));            % Substitute each angle from symbol to number in initial condition. 
P2 = subs(P1,th2,finalconfig(2));
P3 = subs(P2,th3,finalconfig(3));
P4 = subs(P3,th4,finalconfig(4));
P5 = subs(P4,th5,finalconfig(5));
p_target = double(P5);                       % Calculating the final position from given final angle, convert from symbolic type to double type.

theta_k_1 = homeconfig(1:5);                   % Initial interative, note that since you have four rows, the homeconfig is a 4 by 1 vector, we only need first 3 elements
theta_list = theta_k_1;
%% 
% Task 3:
% 
% Calculate the target joint space configuration using the Newton-Raphson iterative 
% technique

theta = [-pi/2, pi/2;      %Angle constraints
         -pi/2, pi/2;
         -pi/2, pi/2;
         -pi/2, pi/2;
         -pi/2, pi/2];

T1 = subs(Ts,th1,theta_k_1(1));
T2 = subs(T1,th2,theta_k_1(2));
T3 = subs(T2,th3,theta_k_1(3));
T4 = subs(T3,th4,theta_k_1(4));
T5 = subs(T4,th5,theta_k_1(5));
T_k_1 = double(vpa(T5,12)); % Translation vector for theta_k_1, Convert from symbolic to numeric 

Vp_to_target = p_target-T_k_1; % vector from guessed position to desired target position 
p_to_target = norm(Vp_to_target); % Distance between target position and guess 


while p_to_target > 0.0001
    disp(p_to_target)
    J0 = subs(JT,th1,theta_k_1(1));
    J1 = subs(J0,th2,theta_k_1(2));
    J2 = subs(J1,th3,theta_k_1(3));
    J3 = subs(J2,th4,theta_k_1(4));
    J4 = subs(J3,th5,theta_k_1(5));
    Jacob = double(vpa(J4,12));       % Jacobian for theta_k_1, Convert from symbolic to numeric       
    Jinv = pinv(Jacob);

    theta_k = theta_k_1 + Jinv * Vp_to_target;     % Using the iterative method
    
    for i = 1:size(theta_k,1)                                        % Angle Constrains 
        theta_k(i) = max(min(theta_k(i), theta(i,2)),theta(i,1));    
    end
    
    theta_list = [theta_list, theta_k];     

    T1 = subs(Ts,th1,theta_k(1));
    T2 = subs(T1,th2,theta_k(2));
    T3 = subs(T2,th3,theta_k(3));
    T4 = subs(T3,th4,theta_k(4));
    T5 = subs(T4,th5,theta_k(5));
    T_k = double(vpa(T5,12));         % Translation vector for newly computed theta_k, Convert from symbolic to numeric   
       
    Vp_to_target = p_target-T_k;         % vector from estimated position to desired target position
    p_to_target = norm(Vp_to_target);     
    
    theta_k_1 = theta_k; 
    T_k_1 = T_k;   
end
theta_final = [theta_list(:,end); 0];      % We need to add one row to visualize in Matlab.
disp(theta_final)

%% 
% Record Video

Initial = homeconfig;
Final = theta_final;
% Final = [pi/2 0 0 0 0]';
% Final = [pi/2 pi/2 -pi/2 0 0]';
% Final = [pi/2 pi/2 -pi/2 0 pi/2]';
Step = 90;
delta_t = 1;
framesPerSecond = 15;
Q = Final(1:5) - Initial;
% Q = Final - theta_k_1;

% angle and angle velocity store
Q_matrix(:, 1) = Initial;
V_matrix(:, 1) = [0;0;0;0;0];

% point position and linear velocity
Point_position_matrix(:, 1) = T_k_1;
Linear_velocity_matrix(:, 1) = [0; 0; 0];
a = figure(10)
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
    T_k = double(vpa(T5,12));         % Translation vector for newly computed theta_k, Convert from symbolic to numeric   
    
    Point_position_matrix(:, i) = T_k;
    if i >= 2
        Linear_velocity_matrix(:, i) = (Point_position_matrix(:, i) - Point_position_matrix(:, i-1)/delta_t);
    end
    hold on
    plot3(Point_position_matrix(1, i), Point_position_matrix(2, i), Point_position_matrix(3, i),'r.','MarkerSize',5)
end

% Save the video

% create the video writer with 1 fps
writerObj = VideoWriter('.\figure\task3_DH5.avi');
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

time = 1:90;

% Plot of joint angles during motion
figure
subplot(5,1,1)
plot(time,Q_matrix(1,:),'r--')
xlabel('Time [s]')
ylabel('Theta_1 [rad]')
xlim([time(1) time(end)]);
subplot(5,1,2)
plot(time,Q_matrix(2,:),'g--')
xlabel('Time [s]')
ylabel('Theta_2 [rad]')
xlim([time(1) time(end)]);
subplot(5,1,3)
plot(time,Q_matrix(3,:),'g--')
xlabel('Time [s]')
ylabel('Theta_3 [rad]')
xlim([time(1) time(end)]);
subplot(5,1,4)
plot(time,Q_matrix(4,:),'g--')
xlabel('Time [s]')
ylabel('Theta_4 [rad]')
xlim([time(1) time(end)]);
subplot(5,1,5)
plot(time,Q_matrix(5,:),'g--')
xlabel('Time [s]')
ylabel('Theta_5 [rad]')
xlim([time(1) time(end)]);

figure;
% Plot of joint velocities during motion
subplot(5,1,1)
plot(time(1:end),V_matrix(1,:),'r--')
xlabel('Time [s]')
ylabel('dTheta_1/dt [rad/s]')
xlim([time(1) time(end)]);
subplot(5,1,2)
plot(time(1:end),V_matrix(2,:),'g--')
xlabel('Time [s]')
ylabel('dTheta_2/dt [rad/s]')
xlim([time(1) time(end)]);
subplot(5,1,3)
plot(time(1:end),V_matrix(3,:),'g--')
xlabel('Time [s]')
ylabel('dTheta_2/dt [rad/s]')
xlim([time(1) time(end)]);
subplot(5,1,4)
plot(time(1:end),V_matrix(4,:),'g--')
xlabel('Time [s]')
ylabel('dTheta_2/dt [rad/s]')
xlim([time(1) time(end)]);
subplot(5,1,5)
plot(time(1:end),V_matrix(5,:),'g--')
xlabel('Time [s]')
ylabel('dTheta_2/dt [rad/s]')
xlim([time(1) time(end)]);

figure;
plot3(Point_position_matrix(1, :), Point_position_matrix(2, :), Point_position_matrix(3, :));

figure;
% Plot of joint velocities during motion
subplot(5,1,1)
plot(time(1:end),Linear_velocity_matrix(1,:),'r--')
xlabel('Time [s]')
ylabel('dTheta_1/dt [rad/s]')
xlim([time(1) time(end)]);
subplot(5,1,2)
plot(time(1:end),Linear_velocity_matrix(2,:),'g--')
xlabel('Time [s]')
ylabel('dTheta_2/dt [rad/s]')
xlim([time(1) time(end)]);
subplot(5,1,3)
plot(time(1:end),Linear_velocity_matrix(3,:),'g--')
xlabel('Time [s]')
ylabel('dTheta_2/dt [rad/s]')
xlim([time(1) time(end)]);
%% 
% Arduino real control
%% 
% 

% matlab_input_final = [pi/2    pi/2    -pi/2        0       0]
% real_final =     [pi          pi/2    0           pi      pi/2]

% matlab_initial = [0    0    0     -pi/2 0]
% real_initial =   [pi/2 0    pi/2 pi/2   pi/2]

compensation = [pi/2 0  pi/2 pi/2   pi/2] - [0  0    0     -pi/2 0]
final_a = compensation' + theta_final(1:5)