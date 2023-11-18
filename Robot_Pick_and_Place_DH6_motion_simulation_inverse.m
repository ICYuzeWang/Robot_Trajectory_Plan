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

tribox = zeros(6,3); %6 by 3 mesh
tribox(1,:) = [0.02/2,  0.056/2,  0];
tribox(2,:) = [0.02/2, -0.056/2,  0];
tribox(3,:) = [-0.02/2,  0.056/2, 0];
tribox(4,:) = [-0.02/2, -0.056/2, 0];
tribox(5,:) = [0,  0.056/2, 0.105];
tribox(6,:) = [0, -0.056/2, 0.105];
collEndeffector = collisionMesh(tribox); % Mesh shape
% collEndeffector = collisionBox(0.105,0.06,0.025);
collEndeffector.Pose = trvec2tform([0 0 -0.105/2]); % centre

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
jnt4 = rigidBodyJoint("jnt4","revolute");
jntend = rigidBodyJoint("endeffector_joint","revolute");
% jntend.HomePosition = pi/2;

%          a        alpha   d       theta      modified DH table 
dhparams = [0   	0    	0.115  	0;
            0    	pi/2   0       0;
            0.105   0       0       0;
            0.097   0       0       0;
            0.03   -pi/2    0       0;
            0       0       0.105   0];
       
bodies = {base,rotating,arm1,arm2,arm3,arm4,endeffector};  
joints = {[],jntrota,jnt1,jnt2,jnt3,jnt4,jntend};

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
%% Defind Jocobian

finalconfig = [pi/2 pi/2 -pi/2 0 pi/2 0]';
syms th1 th2 th3 th4 th5 th6  % Do not forget to install "Symbolic Math Toolbox"

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
T56 = [1,           0,          0,          0; % Transformation Matrix that transforms vectors defined in frame 3 to their description in frame 2
       0,           1,          0,          0;
       0,           0,          1,          0.165;
       0,           0,           0,         1];
T05 =T01 * T12 * T23 * T34 * T45;
T06 = T01 * T12 * T23 * T34 * T45 * T56;
T_dh5 = simplify(T05);      % Transformation Matrix that transforms vectors defined in frame 4 to their description in base frame 0
T_dh6 = simplify(T06);
Ts_dh5 = T_dh5(1:3,4);    % Translation Vector
Ts_dh6 = T_dh6(1:3,4);
JT_dh5 = jacobian(Ts_dh5,[th1,th2,th3,th4,th5,th6]);             % Jacobian Matrix 5 by 3
JT_dh6 = jacobian(Ts_dh6,[th1,th2,th3,th4,th5,th6]); 
%% Robot Simulation & Creating Video

homeconfig = homeConfiguration(robot)
% Task 3:
% Find the end-effector position from the given pose 
% 
% Derive the Transformation matrix of forward kinematics using *Symbolic Math 
% Toolbox*
% Given final angle calculate the target position

P1 = subs(Ts_dh6,th1,finalconfig(1));            % Substitute each angle from symbol to number in initial condition. 
P2 = subs(P1,th2,finalconfig(2));
P3 = subs(P2,th3,finalconfig(3));
P4 = subs(P3,th4,finalconfig(4));
P5 = subs(P4,th5,finalconfig(5));
P6 = subs(P5,th6,finalconfig(6));
p_target = double(P6);                       % Calculating the final position from given final angle, convert from symbolic type to double type.
% p_target2 = double(P6); 
disp(p_target)
% disp(p_target2)
% Task 3: 
% Calculate the Jacobian matrix of kinematic expressions

JT_dh6 = jacobian(Ts_dh6,[th1,th2,th3,th4,th5,th6]);             % Jacobian Matrix 5 by 3

P1 = subs(Ts_dh6,th1,finalconfig(1));            % Substitute each angle from symbol to number in initial condition. 
P2 = subs(P1,th2,finalconfig(2));
P3 = subs(P2,th3,finalconfig(3));
P4 = subs(P3,th4,finalconfig(4));
P5 = subs(P4,th5,finalconfig(5));
P6 = subs(P5,th5,finalconfig(6));
p_target = double(P6);                       % Calculating the final position from given final angle, convert from symbolic type to double type.

theta_k_1 = homeconfig(1:6);                   % Initial interative, note that since you have four rows, the homeconfig is a 4 by 1 vector, we only need first 3 elements
theta_list = theta_k_1;
% Calculate compensation

compensation = [pi/2 0    pi/2 pi/2   pi/2] - [0    0    0     -pi/2 0]
compensation(1,6) = 0
% 
% Target_List

% targets_list = [[90 0 90 90 90 90]' [90 0 150 90 90 90]' [180 0 150 90 90 90]']
targets_list = [90 0 90 90 90 90]';
targets_list(:,2) = [90 0 150 90 90 90]';
targets_list(:,3) = [180 0 150 90 90 90]';
targets_list(:,4) = [180 0 90 90 90 90]';
targets_list(:,5) = [180 0 60 130 90 90]';
targets_list(:,6) = [180 0 60 130 90 160]';
targets_list(:,7) = [180 0 60 150 90 160]';
targets_list(:,8) = [0 0 60 130 90 150]';
targets_list(:,9) = [0 0 60 130 90 90]';
targets_list(:,10) = [0 0 150 130 90 90]';
targets_list(:,11) = [90 0 90 90 90 90]';
targets_list = targets_list ./ 180 * pi;
targets_list = targets_list - compensation';
disp(targets_list)

%% Defind Jocobian

syms th1 th2 th3 th4 th5 th6  % Do not forget to install "Symbolic Math Toolbox"

theta = [-pi/2, pi/2;      %Angle constraints
         0, pi;
         -pi/2, pi/2;
         -pi, 0;
         0, pi;
         0, pi];

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
T56 = [1,           0,          0,          0; % Transformation Matrix that transforms vectors defined in frame 3 to their description in frame 2
       0,           1,          0,          0;
       0,           0,          1,          0.165;
       0,           0,           0,         1];
T05 =T01 * T12 * T23 * T34 * T45;
T06 = T01 * T12 * T23 * T34 * T45 * T56;
T_dh5 = simplify(T05);      % Transformation Matrix that transforms vectors defined in frame 4 to their description in base frame 0
T_dh6 = simplify(T06);
Ts_dh5 = T_dh5(1:3,4);    % Translation Vector
Ts_dh6 = T_dh6(1:3,4);
JT_dh5 = jacobian(Ts_dh5,[th1,th2,th3,th4,th5,th6]);             % Jacobian Matrix 5 by 3
JT_dh6 = jacobian(Ts_dh6,[th1,th2,th3,th4,th5,th6]); 

for i = 1: length(targets_list)
    % forward kinematics
    P1 = subs(Ts_dh6,th1,targets_list(1,i));            % Substitute each angle from symbol to number in initial condition. 
    P2 = subs(P1,th2,targets_list(2,i));
    P3 = subs(P2,th3,targets_list(3,i));
    P4 = subs(P3,th4,targets_list(4,i));
    P5 = subs(P4,th5,targets_list(5,i));
    P6 = subs(P5,th5,targets_list(6,i));
    p_target = double(P6);                       % Calculating the final position from given final angle, convert from symbolic type to double type.

    if i == 1
        theta_list = homeconfig(1:6);
    else
        theta_list = targets_list(:,i-1)
    end

    T1 = subs(Ts_dh6,th1,theta_list(1));
    T2 = subs(T1,th2,theta_list(2));
    T3 = subs(T2,th3,theta_list(3));
    T4 = subs(T3,th4,theta_list(4));
    T5 = subs(T4,th5,theta_list(5));
    T6 = subs(T5,th6,theta_list(6));
    T_k_1 = double(vpa(T6,12)); % Translation vector for theta_k_1, Convert from symbolic to numeric 

    Vp_to_target = p_target - T_k_1; % vector from guessed position to desired target position
    p_to_target = norm(Vp_to_target); % Distance between target position and guess

    if i == 11
        continue
    else
        while p_to_target > 0.001
            % disp(p_to_target)
            J0 = subs(JT_dh6,th1,theta_k_1(1));
            J1 = subs(J0,th2,theta_k_1(2));
            J2 = subs(J1,th3,theta_k_1(3));
            J3 = subs(J2,th4,theta_k_1(4));
            J4 = subs(J3,th5,theta_k_1(5));
            J5 = subs(J4,th6,theta_k_1(6));
            Jacob = double(vpa(J5,12));       % Jacobian for theta_k_1, Convert from symbolic to numeric
            Jinv = pinv(Jacob);
    
            theta_k = theta_k_1 + Jinv * Vp_to_target;     % Using the iterative method
    
            for ii = 1:size(theta_k,1)                                        % Angle Constrains
                theta_k(ii) = max(min(theta_k(ii), theta(ii,2)),theta(ii,1));
            end
    
            theta_list = [theta_list, theta_k];
    
            T1 = subs(Ts_dh6,th1,theta_k(1));
            T2 = subs(T1,th2,theta_k(2));
            T3 = subs(T2,th3,theta_k(3));
            T4 = subs(T3,th4,theta_k(4));
            T5 = subs(T4,th5,theta_k(5));
            T6 = subs(T5,th6,theta_k(6));
            T_k = double(vpa(T6,12));         % Translation vector for newly computed theta_k, Convert from symbolic to numeric
    
            Vp_to_target = p_target-T_k;         % vector from estimated position to desired target position
            p_to_target = norm(Vp_to_target);
    
            theta_k_1 = theta_k;
            T_k_1 = T_k;
        end
    end
    targets_list(:,i) = theta_list(1:6,end);
end
disp(targets_list)
% Record Video

% targets_list = [[90 0 150 90 90 90]' []']
figure
count_qvp = 1;
Linear_velocity_matrix = zeros(1);
for ii = 1:size(targets_list,2)-1
%     Initial = homeconfig;
%     Final = theta_final(1:6);
    Initial = targets_list(:,ii);
    Final = targets_list(:,ii+1);
    % Final = [pi/2 0 0 0 0]';
    % Final = [pi/2 pi/2 -pi/2 0 0]';
    % Final = [pi/2 pi/2 -pi/2 0 pi/2]';
    Step = max(round(abs(Final - Initial)/pi*180));
    delta_t = 1/30;
    % framesPerSecond = 15;
    Q = Final(1:6) - Initial;
    % Q = Final - theta_k_1;
    % angle and angle velocity store
%     Q_matrix(:, 1) = Initial;
%     V_matrix(:, 1) = [0;0;0;0;0;0];
    %可能有问题，initial和上一个last一样
    Q_matrix(:, count_qvp) = Initial;
    V_matrix(:, count_qvp) = [0;0;0;0;0;0]; 
    % point position and linear velocity
%     Point_position_matrix(:, 1) = T_k_1;
    Point_position_matrix(:, count_qvp) = T_k_1;
%     Linear_velocity_matrix = zeros(Step, 1);
%     figure
    
    for i = 1:Step
        q = Initial + Q / Step * i;
%         Q_matrix(:, i) = q;
        Q_matrix(:, count_qvp) = q;
    
        show(robot,q,"Collisions","on",'PreservePlot',false,"Frames","off");
        F(count_qvp) = getframe(gcf);
        drawnow
        if i >= 2
%             V_matrix(:, i) = (Q_matrix(:, i) - Q_matrix(:, i-1))/delta_t;
            V_matrix(:, count_qvp) = (Q_matrix(:, count_qvp) - Q_matrix(:, count_qvp-1))/delta_t;
        end
    
        T1 = subs(Ts_dh6,th1,q(1));
        T2 = subs(T1,th2,q(2));
        T3 = subs(T2,th3,q(3));
        T4 = subs(T3,th4,q(4));
        T5 = subs(T4,th5,q(5));
        T6 = subs(T5,th6,q(6));
        T_k = double(vpa(T6,12));         % Translation vector for newly computed theta_k, Convert from symbolic to numeric   
        
%         Point_position_matrix(:, i) = T_k;
        Point_position_matrix(:, count_qvp) = T_k;
        if i >= 2
%             Linear_velocity_matrix(i) = ((Point_position_matrix(1, i) - Point_position_matrix(1, i-1))^2 + ...
%                 (Point_position_matrix(2, i) - Point_position_matrix(2, i-1))^2 + ....
%                 (Point_position_matrix(3, i) - Point_position_matrix(3, i-1))^2)^0.5/delta_t;
            Linear_velocity_matrix(count_qvp) = ((Point_position_matrix(1, count_qvp) - Point_position_matrix(1, count_qvp-1))^2 + ...
                            (Point_position_matrix(2, count_qvp) - Point_position_matrix(2, count_qvp-1))^2 + ....
                            (Point_position_matrix(3, count_qvp) - Point_position_matrix(3, count_qvp-1))^2)^0.5/delta_t;
        end
        hold on
        plot3(Point_position_matrix(1, count_qvp), Point_position_matrix(2, count_qvp), Point_position_matrix(3, count_qvp),'r.','MarkerSize',5)

        count_qvp = count_qvp + 1;
    end
end
% Save the video

% create the video writer with 1 fps
writerObj = VideoWriter('.\figure\task4_simulation_inverse.avi');
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
disp(T_k)
%% Plot angle, angle velocity, point position, linear velocity

% time = (1:step)./30;
% time = (1:Step).*delta_t;
time = (1:size(Q_matrix,2)).*delta_t;
time = (1:size(Q_matrix,2))/10;
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
saveas(a,'.\figure\task4_simulation_inverse_angle.png')
%%
a = figure;
% Plot of joint velocities during motion
subplot(3,2,1)
plot(time(1:end)',V_matrix(1,:),'r--')
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
saveas(a,'.\figure\task4_simulation_inverse_angle_velocity.png')
%%
a = figure;
plot3(Point_position_matrix(1, 2:end), Point_position_matrix(2, 2:end), Point_position_matrix(3, 2:end));
saveas(a,'.\figure\task4_simulation_inverse_trajectory.png')
%% Plot Linear Velocity

a = figure;
plot(time(1:end), Linear_velocity_matrix)
xlabel('Time [s]')
ylabel("Linear Velocity of end-effector [m/s]")
xlim([0 max(time)])
ylim([0 max(Linear_velocity_matrix)*1.1])
title("Linear Velocity of end-effector")
saveas(a,'.\figure\task4_simulation_inverse_linear_velocity.png')
% % Plot of joint velocities during motion
% subplot(3,1,1)
% plot(time(1:end),Linear_velocity_matrix(1,:),'r--')
% xlabel('Time [s]')
% ylabel('dTheta_1/dt [rad/s]')
% xlim([time(1) time(end)]);
% subplot(3,1,2)
% plot(time(1:end),Linear_velocity_matrix(2,:),'g--')
% xlabel('Time [s]')
% ylabel('dTheta_2/dt [rad/s]')
% xlim([time(1) time(end)]);
% subplot(3,1,3)
% plot(time(1:end),Linear_velocity_matrix(3,:),'g--')
% xlabel('Time [s]')
% ylabel('dTheta_2/dt [rad/s]')
% xlim([time(1) time(end)]);
%% 
% 

compensation = [pi/2 0    pi/2 pi/2   pi/2] - [0    0    0     -pi/2 0]
% final_a = compensation' + theta_final(1:5)