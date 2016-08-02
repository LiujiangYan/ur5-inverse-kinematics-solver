% inverse kinematics solution from solver
% size: time step * 6
qc = csvread('');
% should be your chosen pose of first time step
pose = [];
% store the pose along the time array
traj = [pose];

[row,col]=size(qc);
for i=1:row/6-1
    minimum = inf;
    for j=1:6
        distance = norm(pose - qc(i*6+j,:));
        if distance < minimum
            refer = qc(i*6+j,:);
            minimum = distance;
        end
    end
    pose = refer;
    traj = [traj;pose];
end

% construct the full UR5
ur5_L(1) = Link('d', 0.089159, 'a', 0, 'alpha', pi/2);
ur5_L(2) = Link('d', 0, 'a', -0.425, 'alpha', 0);
ur5_L(3) = Link('d', 0, 'a', -0.39225, 'alpha', 0);
ur5_L(4) = Link('d', 0.10915, 'a', 0, 'alpha', pi/2);
ur5_L(5) = Link('d', 0.09565, 'a', 0, 'alpha', -pi/2);
ur5_L(6) = Link('d', 0.0823, 'a', 0, 'alpha', 0);
% ur5_L(4) = Link('d', 0, 'a', 0, 'alpha', pi/2);
% ur5_L(5) = Link('d', 0, 'a', 0, 'alpha', -pi/2);
% ur5_L(6) = Link('d', 0, 'a', 0, 'alpha', 0);
ur5 = SerialLink(ur5_L, 'name', 'ur5-6axis');
% ur5.ikineType = 'puma';

ur5.plot(traj);
csvwrite('iksolution.csv',output);
