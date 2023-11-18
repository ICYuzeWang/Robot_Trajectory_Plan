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
% centre of endeffector
collEndeffector.Pose = trvec2tform([0 0 0.06]); 

% Add Collision to the links
addCollision(rotating,collrota)           
addCollision(arm1,coll1)
addCollision(arm2,coll2)
addCollision(arm3,coll3)
addCollision(endeffector,collEndeffector)

% Set joints as revolute joints
jntrota = rigidBodyJoint("rotation_joint","revolute");    
jnt1 = rigidBodyJoint("jnt1","revolute");
jnt1.HomePosition = pi/2;
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

% Visualise the robot
a = figure("Name","Robot","Visible","on")               
for i = 2:length(bodies)
    setFixedTransform(joints{i},dhparams(i-1,:),"mdh");
    bodies{i}.Joint = joints{i};
    addBody(robot,bodies{i},bodies{i-1}.Name)
    show(robot,"Collisions","on","Frames","off");
    drawnow;     
end
% save the figure
saveas(a,'.\figure\Task1_DH5.png')
% show the plot
a = figure;
show(robot)
% save the figure
saveas(a,'.\figure\Task1_DH5_coordinate.png')