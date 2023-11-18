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
collEndeffector.Pose = trvec2tform([0 0 0.06]); 

% Add Collision to the links
addCollision(rotating,collrota)           
addCollision(arm1,coll1)
addCollision(arm2,coll2)
addCollision(arm3,coll3)
addCollision(endeffector,collEndeffector)

% set joints as revolute joints
jntrota = rigidBodyJoint("rotation_joint","revolute");    
jnt1 = rigidBodyJoint("jnt1","revolute");
jnt2 = rigidBodyJoint("jnt2","revolute");
jnt3 = rigidBodyJoint("jnt3","revolute");
jnt3.HomePosition = -pi/2;
jntend = rigidBodyJoint("endeffector_joint","revolute");

%          a        alpha   d       theta      modified DH table 
dhparams = [0   	0    	0.115  	0;
            0    	pi/2   0       0;
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
    view(2)
    drawnow;     
end
figure;
show(robot)
%% 
% 
% Given final angle calculate the target position

syms th1 th2 th3 th4 th5  
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
   
T05 = T01 * T12 * T23 * T34 * T45;
T = simplify(T05);
% Translation Vector
Ts = T(1:3,4);  
% Jacobian Matrix 5 by 3
JT = jacobian(Ts,[th1,th2,th3,th4,th5]);

%% Robot Simulation & Creating Video

% Define the initial position
Initial = homeConfiguration(robot)
% Define the final position
Final = [pi/2 pi/2 -pi/2 0 pi/2]';
Step = 90;
framesPerSecond = 15;
Q = Final(1:5) - Initial;
figure
for i = 1:Step
    q = Initial + Q / Step * i;
    show(robot,q,"Collisions","on",'PreservePlot',false,"Frames","off");
    F(i) = getframe(gcf);
    drawnow

    % Substitute each angle from symbol to number in initial condition. 
    P1 = subs(Ts,th1,q(1));       
    P2 = subs(P1,th2,q(2));
    P3 = subs(P2,th3,q(3));
    P4 = subs(P3,th4,q(4));
    P5 = subs(P4,th5,q(5));
    % Calculating the final position from given final angle, convert from symbolic type to double type.
    p_target = double(P5);         
    
    hold on
    plot3(p_target(1), p_target(2), p_target(3),'r.','MarkerSize',5)
end
% create the video writer with 1 fps
writerObj = VideoWriter('.\figure\task2_DH5.avi');
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
% Given final angle calculate the target position

% Setting final angle
finalconfig = Final;
% Substitute each angle from symbol to number in initial condition. 
P1 = subs(Ts,th1,finalconfig(1));            
P2 = subs(P1,th2,finalconfig(2));
P3 = subs(P2,th3,finalconfig(3));
P4 = subs(P3,th4,finalconfig(4));
P5 = subs(P4,th5,finalconfig(5));
% Calculating the final position from given final angle, convert from symbolic type to double type.
p_target = double(P5);                       

disp(p_target)
%