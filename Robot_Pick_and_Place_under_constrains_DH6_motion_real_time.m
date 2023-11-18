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
% Calculate compensation

compensation = [pi/2 0    pi/2 pi/2   pi/2] - [0    0    0     -pi/2 0]
compensation(1,6) = 0
% Target_List

% targets_list = [[90 0 90 90 90 90]' [90 0 150 90 90 90]' [180 0 150 90 90 90]']
% Connect to serial port
s = serialport('COM3', 115200);  %Set this to the port/baudrate of your arduino
pause(3);
fprintf("Connection established\n")
%servo control rate in Hz (the higher the number, the faster joint angles are sent) 
r = rateControl(20);  

%Initial angle (homing) for all servos. In degrees 
%Make sure it matches the hardcoded angle in the Arduino code.
%Expect servos moving fast here if not done correctly


fprintf("Executed trajectory completed.\n")
real_homing_pos= [90 90 90 90 90 90]
str = compose("<Angle, %d, %d, %d, %d, %d, %d>", real_homing_pos);
write(s, str, "char")
% targets_list = [[90 0 90 90 90 90]' [90 0 150 90 90 90]' [180 0 150 90 90 90]']
targets_list = [100 90 90 90 90 90]';
% targets_list(:,2) = [90 0 150 90 90 90]';
targets_list(:,2) = [180 90 90 90 90 90]';
targets_list(:,3) = [180 20 90 90 90 90]';
targets_list(:,4) = [180 20 80 90 90 90]';
targets_list(:,5) = [180 20 60 80 90 90]';
targets_list(:,6) = [180 20 60 80 90 160]';
targets_list(:,7) = [180 20 60 120 90 160]';
targets_list(:,8) = [0 20 60 90 120 150]';
targets_list(:,9) = [0 20 60 90 120 90]';
targets_list(:,10) = [0 20 170 130 90 90]';
% targets_list(:,11) = [90 0 90 90 90 90]';
targets_list(:,11) = [100 90 90 90 90 90]';
targets_list = targets_list ./ 180 * pi;
targets_list = targets_list - compensation';
% Record Video

% calculate initial position
homeconfig = real_homing_pos - compensation;
T1 = subs(Ts_dh6,th1,homeconfig(1));
T2 = subs(T1,th2,homeconfig(2));
T3 = subs(T2,th3,homeconfig(3));
T4 = subs(T3,th4,homeconfig(4));
T5 = subs(T4,th5,homeconfig(5));
T6 = subs(T5,th6,homeconfig(6));
T_k = double(vpa(T6,12));         % Translation vector for newly computed theta_k, Convert from symbolic to numeric 

t1 = clock;
time = zeros(1);
% targets_list = [[90 0 150 90 90 90]' []']
figure
count_qvp = 1;
Linear_velocity_matrix = zeros(1);
for ii = 1:size(targets_list,2)-1
    Initial = targets_list(:,ii);
    Final = targets_list(:,ii+1);
    q = Initial;
    Step = max(round(abs(Final - Initial)/pi*180));
    delta_t = 1/30;
    Q = Final(1:6) - Initial;
    angle_step = Q / Step;
    % count_qvp = count_qvp + 1;
    % angle and angle velocity store
    Q_matrix(:, count_qvp) = Initial;
    V_matrix(:, count_qvp) = [0;0;0;0;0;0]; 
    Point_position_matrix(:, count_qvp) = T_k;
    iteration = 1;
    framesave = 1;  

    while iteration == 1
        tic;
        q = q + angle_step;
        Q_matrix(:, count_qvp) = q;
        show(robot,q,"Collisions","on",'PreservePlot',false,"Frames","off");
        F(count_qvp) = getframe(gcf);
        drawnow

        goto_position = (q' + compensation)/pi * 180;
        str = compose("<Angle,%d,%d,%d,%d,%d,%d>", floor(goto_position));
        write(s, str, "char")
        waitfor(r);

        if count_qvp >= 2
            V_matrix(:, count_qvp) = (Q_matrix(:, count_qvp) - Q_matrix(:, count_qvp-1))/delta_t;
        end

        framesave = framesave + 1;
        P1 = subs(Ts_dh6,th1,q(1));       % Substitute each angle from symbol to number in initial condition.
        P2 = subs(P1,th2,q(2));
        P3 = subs(P2,th3,q(3));
        P4 = subs(P3,th4,q(4));
        P5 = subs(P4,th5,q(5));
        P6 = subs(P5,th6,q(6));
        p_target = double(P6);         % Calculating the final position from given final angle, convert from symbolic type to double type.
        hold on
        plot3(p_target(1), p_target(2), p_target(3),'r.','MarkerSize',5)

        % end for each loop
        time(count_qvp) = etime(clock,t1);
        delta_t = toc;

        Point_position_matrix(:, count_qvp) = p_target;
        if count_qvp >= 2
            Linear_velocity_matrix(count_qvp) = ((Point_position_matrix(1, count_qvp) - Point_position_matrix(1, count_qvp-1))^2 + ...
                            (Point_position_matrix(2, count_qvp) - Point_position_matrix(2, count_qvp-1))^2 + ....
                            (Point_position_matrix(3, count_qvp) - Point_position_matrix(3, count_qvp-1))^2)^0.5/delta_t;
        end
        count_qvp = count_qvp + 1;

        if p_target(1) > 0.25
            while p_target(1) > 0.25
                tic;
                Step_path_change = 10
                Q_angle_change = [0,0,pi/5,0,0,0]'
                for i = 1:Step_path_change
                    
                    q = q + Q_angle_change / Step_path_change;
                    Q_matrix(:, count_qvp) = q;
                    show(robot,q,"Collisions","on",'PreservePlot',false,"Frames","off");
                    F(count_qvp) = getframe(gcf);
                    drawnow

                    goto_position = (q' + compensation)/pi * 180;
                    str = compose("<Angle,%d,%d,%d,%d,%d,%d>", floor(goto_position));
                    write(s, str, "char")
                    waitfor(r);
            
                    if i >= 2
                        V_matrix(:, count_qvp) = (Q_matrix(:, count_qvp) - Q_matrix(:, count_qvp-1))/delta_t;
                    end

                    framesave = framesave + 1;
                    P1 = subs(Ts_dh6,th1,q(1));       % Substitute each angle from symbol to number in initial condition.
                    P2 = subs(P1,th2,q(2));
                    P3 = subs(P2,th3,q(3));
                    P4 = subs(P3,th4,q(4));
                    P5 = subs(P4,th5,q(5));
                    P6 = subs(P5,th6,q(6));
                    p_target = double(P6);         % Calculating the final position from given final angle, convert from symbolic type to double type.
                    

                    Point_position_matrix(:, count_qvp) = p_target;
                    if i >= 2
                        Linear_velocity_matrix(count_qvp) = ((Point_position_matrix(1, count_qvp) - Point_position_matrix(1, count_qvp-1))^2 + ...
                                        (Point_position_matrix(2, count_qvp) - Point_position_matrix(2, count_qvp-1))^2 + ....
                                        (Point_position_matrix(3, count_qvp) - Point_position_matrix(3, count_qvp-1))^2)^0.5/delta_t;
                    end

                    hold on
                    plot3(p_target(1), p_target(2), p_target(3),'r.','MarkerSize',5)

                    % end for each loop
                    time(count_qvp) = etime(clock,t1);
                    delta_t = toc;
                    count_qvp = count_qvp + 1;
                end
            end
        end

        Q = Final - q;
        Step = max(round(abs(Q)/pi*180));
        angle_step = Q / Step;
        if max(abs(angle_step)) < 0.0001
            break
        end

        if max(round(abs(Q)/pi*180)) < 5
            iteration = 2;
        end
    end
end
% Save the video

% create the video writer with 1 fps
writerObj = VideoWriter('.\figure\task5_real.avi');
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

%%
a = figure;
plot3(Point_position_matrix(1, 2:end), Point_position_matrix(2, 2:end), Point_position_matrix(3, 2:end));
%% Plot Linear Velocity

a = figure;
plot(time(1:end), Linear_velocity_matrix)
xlabel('Time [s]')
ylabel("Linear Velocity of end-effector [m/s]")
xlim([0 max(time)])
ylim([0 max(Linear_velocity_matrix)*1.1])
title("Linear Velocity of end-effector")
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
final_a = compensation' + theta_final(1:5)