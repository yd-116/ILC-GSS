% Time vector
t = 0:0.01:10;

% Controller parameters
Kp = 100;
Kd = 100;
L  = 4.15;

% Initialize torque arrays (7 joints)
torque1 = [t', zeros(1001, 1)];
torque2 = [t', zeros(1001, 1)];
torque3 = [t', zeros(1001, 1)];
torque4 = [t', zeros(1001, 1)];
torque5 = [t', zeros(1001, 1)];
torque6 = [t', zeros(1001, 1)];
torque7 = [t', zeros(1001, 1)];

% First simulation run
sim('WAM_Arm.slx');

% ILC iteration loop (10 iterations)
i = 0;
while i < 10
    i = i + 1;
    
    % Update control torque for each joint
    torque1(:,2) = torque1(:,2) + L*Kp*ans.ep1 + L*Kd*ans.ev1;
    torque2(:,2) = torque2(:,2) + L*Kp*ans.ep2 + L*Kd*ans.ev2;
    torque3(:,2) = torque3(:,2) + L*Kp*ans.ep3 + L*Kd*ans.ev3;
    torque4(:,2) = torque4(:,2) + L*Kp*ans.ep4 + L*Kd*ans.ev4;
    torque5(:,2) = torque5(:,2) + L*Kp*ans.ep5 + L*Kd*ans.ev5;
    torque6(:,2) = torque6(:,2) + L*Kp*ans.ep6 + L*Kd*ans.ev6;
    torque7(:,2) = torque7(:,2) + L*Kp*ans.ep7 + L*Kd*ans.ev7;
    
    % Run simulation with updated torque
    sim('WAM_Arm.slx');

    % Calculate root sum of squares (RSS) for tracking error
    RSS = sqrt(sum(ans.ep1.^2) + sum(ans.ep2.^2) + sum(ans.ep3.^2) + sum(ans.ep4.^2) + sum(ans.ep5.^2) + sum(ans.ep6.^2) + sum(ans.ep7.^2));    
end