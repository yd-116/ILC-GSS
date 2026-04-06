% Time vector
t = 0:0.01:10;

% Initialize torque matrices
torque1 = [t', zeros(1001,1)];
torque2 = [t', zeros(1001,1)];
torque3 = [t', zeros(1001,1)];
torque4 = [t', zeros(1001,1)];
torque5 = [t', zeros(1001,1)];
torque6 = [t', zeros(1001,1)];
torque7 = [t', zeros(1001,1)];

% Run Simulink simulation
sim('WAM_Arm');

% Calculate Root Sum of Squares (RSS) tracking error
RSS = sqrt(sum(ans.ep1.^2) + sum(ans.ep2.^2) + sum(ans.ep3.^2) + sum(ans.ep4.^2) + sum(ans.ep5.^2) + sum(ans.ep6.^2) + sum(ans.ep7.^2));