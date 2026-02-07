clc; clear; close all;


robot = importrobot("Robot_URDF_Ground_Plane_Ref.urdf");
robot.DataFormat = "column";
robot.Gravity = [0 0 -9.81];


bodies = robot.Bodies;
jointNames = strings(0);

for i = 1:numel(bodies)
    if ~strcmp(bodies{i}.Joint.Type,"fixed")
        jointNames(end+1) = bodies{i}.Joint.Name; %#ok<SAGROW>
    end
end

nJoints = numel(jointNames);


leftArmJoints = [
    "Left_Shoulder_Joint"
    "Left_Arm_Joint"
    "Left_Wrist_Joint"
];

rightArmJoints = [
    "Right_Shoulder_Joint"
    "Right_Arm_Joint"
    "Right_Wrist_Joint"
];

leftIdx  = find(ismember(jointNames,leftArmJoints));
rightIdx = find(ismember(jointNames,rightArmJoints));


qStart = zeros(nJoints,1);


disp("Enter common Cartesian goal for BOTH hands");

% User input
pGoal_L = input("Position [x y z]: ");          % 1x3 vector
rpy_L   = input("Orientation [roll pitch yaw] (rad): "); % 1x3 vector

% Extract components (MATLAB-safe way)
x = pGoal_L(1);
y = pGoal_L(2);
z = pGoal_L(3);

roll  = rpy_L(1);
pitch = rpy_L(2);
yaw   = rpy_L(3);

% LEFT hand target
Tgoal_L = trvec2tform([x  y  z]) * eul2tform([roll pitch yaw]);

% RIGHT hand target (mirror across Y-axis)
Tgoal_R = trvec2tform([-x -y  -z]) * eul2tform([-roll -pitch -yaw]);


ik = inverseKinematics("RigidBodyTree",robot);
weights = [1 1 1 1 1 1];

% ---- Left arm IK
[qL,infoL] = ik("Left_Wrist_Link",Tgoal_L,weights,qStart);
if infoL.ExitFlag <= 0
    error("Left arm IK failed");
end

% ---- Right arm IK
[qR,infoR] = ik("Right_Wrist_Link",Tgoal_R,weights,qStart);
if infoR.ExitFlag <= 0
    error("Right arm IK failed");
end

qGoal = qStart;
qGoal(leftIdx)  = qL(leftIdx);
qGoal(rightIdx) = qR(rightIdx);

nVia = input("Enter number of via points: ");
qVia = zeros(nJoints,nVia);

for i = 1:nVia
    alpha = i/(nVia+1);
    qVia(:,i) = (1-alpha)*qStart + alpha*qGoal;
end

Tf = 2;
dt = 0.05;
t  = 0:dt:Tf;

waypoints = [qStart qVia qGoal];
timePoints = linspace(0,Tf,size(waypoints,2));

[qTraj,qdTraj,qddTraj] = cubicpolytraj(waypoints,timePoints,t);

% ---------------- GET ALL NON-FIXED JOINT NAMES ----------------
bodies = robot.Bodies;

allJointNames = strings(0,1);

for i = 1:numel(bodies)
    jnt = bodies{i}.Joint;
    if ~strcmp(jnt.Type,'fixed')
        allJointNames(end+1,1) = string(jnt.Name); %#ok<SAGROW>
    end
end

nJoints = numel(allJointNames);


viaTimes = timePoints(2:end-1);
viaIdx   = round(viaTimes / (t(2)-t(1))) + 1;


leftWristIdx  = find(allJointNames == "Left_Wrist_Joint");
rightWristIdx = find(allJointNames == "Right_Wrist_Joint");


if isempty(leftWristIdx) || isempty(rightWristIdx)
    error("Wrist joint names not found. Check URDF joint names.");
end



t = linspace(0, timePoints(end), size(qTraj,2));


%left arm

figure('Name','LEFT ARM – Wrist Joint');

% -------- Position --------
subplot(3,1,1)
plot(t, qTraj(leftWristIdx,:), 'b','LineWidth',1.8)
ylabel('Position (rad)')
title('Left Wrist Joint Position')
grid on

% -------- Velocity --------
subplot(3,1,2)
plot(t, qdTraj(leftWristIdx,:), 'b','LineWidth',1.8)
ylabel('Velocity (rad/s)')
title('Left Wrist Joint Velocity')
grid on

% -------- Acceleration --------
subplot(3,1,3)
plot(t, qddTraj(leftWristIdx,:), 'b','LineWidth',1.8)
ylabel('Acceleration (rad/s^2)')
xlabel('Time (s)')
title('Left Wrist Joint Acceleration')
grid on


%right arm
figure('Name','RIGHT ARM – Wrist Joint');

% -------- Position --------
subplot(3,1,1)
plot(t, qTraj(rightWristIdx,:), 'g','LineWidth',1.8)
ylabel('Position (rad)')
title('Right Wrist Joint Position')
grid on

% -------- Velocity --------
subplot(3,1,2)
plot(t, qdTraj(rightWristIdx,:), 'g','LineWidth',1.8)
ylabel('Velocity (rad/s)')
title('Right Wrist Joint Velocity')
grid on

% -------- Acceleration --------
subplot(3,1,3)
plot(t, qddTraj(rightWristIdx,:), 'g','LineWidth',1.8)
ylabel('Acceleration (rad/s^2)')
xlabel('Time (s)')
title('Right Wrist Joint Acceleration')
grid on


%left arm joint
figure(findobj('Name','LEFT ARM – Wrist Joint'))

subplot(3,1,1)
hold on
plot(viaTimes, qTraj(leftWristIdx,viaIdx), 'ro','MarkerSize',8,'LineWidth',1.5)

subplot(3,1,2)
hold on
plot(viaTimes, qdTraj(leftWristIdx,viaIdx), 'ro','MarkerSize',8,'LineWidth',1.5)

subplot(3,1,3)
hold on
plot(viaTimes, qddTraj(leftWristIdx,viaIdx), 'ro','MarkerSize',8,'LineWidth',1.5)


%right arm joint
figure(findobj('Name','RIGHT ARM – Wrist Joint'))

subplot(3,1,1)
hold on
plot(viaTimes, qTraj(rightWristIdx,viaIdx), 'ro','MarkerSize',8,'LineWidth',1.5)

subplot(3,1,2)
hold on
plot(viaTimes, qdTraj(rightWristIdx,viaIdx), 'ro','MarkerSize',8,'LineWidth',1.5)

subplot(3,1,3)
hold on
plot(viaTimes, qddTraj(rightWristIdx,viaIdx), 'ro','MarkerSize',8,'LineWidth',1.5)


%simulating the robot
figure('Name','Dual Arm Simulation');
ax = show(robot, qStart, ...
    "Frames","off", ...
    "Visuals","on");
axis equal
view(135,25)
hold on
grid on

% Storage for end-effector paths
pLeft  = zeros(3,size(qTraj,2));
pRight = zeros(3,size(qTraj,2));

for k = 1:size(qTraj,2)

    % Animate robot
    show(robot, qTraj(:,k), ...
        "PreservePlot", false, ...
        "Frames","off", ...
        "Visuals","on");

    % End-effector transforms
    TL = getTransform(robot, qTraj(:,k), "Left_Gripper_Link");
    TR = getTransform(robot, qTraj(:,k), "Right_Gripper_Link");

    % Positions
    pLeft(:,k)  = TL(1:3,4);
    pRight(:,k) = TR(1:3,4);

    % Plot path (BLUE)
    plot3(pLeft(1,1:k), pLeft(2,1:k), pLeft(3,1:k), ...
          'b','LineWidth',2);
    plot3(pRight(1,1:k),pRight(2,1:k),pRight(3,1:k), ...
          'b','LineWidth',2);

    drawnow
end

% ---- Plot VIA POINTS (RED) ----
for i = 1:nVia
    TLv = getTransform(robot, qVia(:,i), "Left_Gripper_Link");
    TRv = getTransform(robot, qVia(:,i), "Right_Gripper_Link");

    plot3(TLv(1,4), TLv(2,4), TLv(3,4), ...
          'ro','MarkerSize',10,'LineWidth',2);
    plot3(TRv(1,4), TRv(2,4), TRv(3,4), ...
          'ro','MarkerSize',10,'LineWidth',2);
end

title("Dual-Arm Trajectory with Via Points and End-Effector Paths");


%%
% Examples:
% postion matrix: [-0.2 -0.5 -0.9]
% orientation matrix: [-1 -0.3 -0.6]