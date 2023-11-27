%Dati presi da simulazione con controllore vecchio e valori di PID1 ! !
%Import .bag file in matlab
bag = rosbag('subset.bag'); 
%create a struct
jointData = struct('joint1', [], 'joint2', [], 'joint3', [], 'joint4', [], 'joint5', [], 'joint6', [], 'joint7', []);
%Fill in the structure with topics
jointData.joint1 = select(bag,'Topic','/iiwa/iiwa_joint_1_effort_controller/command');
jointData.joint2 = select(bag,'Topic','/iiwa/iiwa_joint_2_effort_controller/command');
jointData.joint3 = select(bag,'Topic','/iiwa/iiwa_joint_3_effort_controller/command');
jointData.joint4 = select(bag,'Topic','/iiwa/iiwa_joint_4_effort_controller/command');
jointData.joint5 = select(bag,'Topic','/iiwa/iiwa_joint_5_effort_controller/command');
jointData.joint6 = select(bag,'Topic','/iiwa/iiwa_joint_6_effort_controller/command');
jointData.joint7 = select(bag,'Topic','/iiwa/iiwa_joint_7_effort_controller/command');

%Plot all
for R = 1:7
    jointTopic = sprintf('joint%d', R);
    data = jointData.(jointTopic);
    timeVector = linspace(data.StartTime, data.EndTime, data.NumMessages);
    msgStructs = readMessages(data,'DataFormat','struct');
    Points = cellfun(@(m) double(m.Data),msgStructs);
    plot(timeVector,Points)
    hold on
end

legend('iiwaJoint1','iiwaJoint2','iiwaJoint3','iiwaJoint4','iiwaJoint5','iiwaJoint6','iiwaJoint7');
