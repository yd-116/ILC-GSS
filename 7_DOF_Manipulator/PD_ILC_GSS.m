% Time vector
t = 0:0.01:10;

% PD controller parameters
Kp = 100;
Kd = 100;

% Golden Section Search (GSS) parameters
a = 0;
b = 10;
tau = 0.618;
n = 10;

fprintf('GSS for optimal L: ');

% Main Golden Section Search loop
for iter = 1:n
    LL = b - tau * (b - a);
    LR = a + tau * (b - a);

    % Evaluate objective function at LL
    torque1 = [t', zeros(1001, 1)];
    torque2 = [t', zeros(1001, 1)];
    torque3 = [t', zeros(1001, 1)];
    torque4 = [t', zeros(1001, 1)];
    torque5 = [t', zeros(1001, 1)];
    torque6 = [t', zeros(1001, 1)];
    torque7 = [t', zeros(1001, 1)];
    
    sim('WAM_Arm.slx');

    i = 0;
    while i < 10
        i = i + 1;
        torque1(:, 2) = torque1(:, 2) + LL * Kp * ans.ep1 + LL * Kd * ans.ev1;
        torque2(:, 2) = torque2(:, 2) + LL * Kp * ans.ep2 + LL * Kd * ans.ev2;
        torque3(:, 2) = torque3(:, 2) + LL * Kp * ans.ep3 + LL * Kd * ans.ev3;
        torque4(:, 2) = torque4(:, 2) + LL * Kp * ans.ep4 + LL * Kd * ans.ev4;
        torque5(:, 2) = torque5(:, 2) + LL * Kp * ans.ep5 + LL * Kd * ans.ev5;
        torque6(:, 2) = torque6(:, 2) + LL * Kp * ans.ep6 + LL * Kd * ans.ev6;
        torque7(:, 2) = torque7(:, 2) + LL * Kp * ans.ep7 + LL * Kd * ans.ev7;
        
        sim('WAM_Arm.slx');
    end
    errorL = sqrt(sum(ans.ep1.^2) + sum(ans.ep2.^2) + sum(ans.ep3.^2) + sum(ans.ep4.^2) + sum(ans.ep5.^2) + sum(ans.ep6.^2) + sum(ans.ep7.^2));

    % Evaluate objective function at LR
    torque1 = [t', zeros(1001, 1)];
    torque2 = [t', zeros(1001, 1)];
    torque3 = [t', zeros(1001, 1)];
    torque4 = [t', zeros(1001, 1)];
    torque5 = [t', zeros(1001, 1)];
    torque6 = [t', zeros(1001, 1)];
    torque7 = [t', zeros(1001, 1)];
    
    sim('WAM_Arm.slx');
    
    i = 0;
    while i < 10
        i = i + 1;
        torque1(:, 2) = torque1(:, 2) + LR * Kp * ans.ep1 + LR * Kd * ans.ev1;
        torque2(:, 2) = torque2(:, 2) + LR * Kp * ans.ep2 + LR * Kd * ans.ev2;
        torque3(:, 2) = torque3(:, 2) + LR * Kp * ans.ep3 + LR * Kd * ans.ev3;
        torque4(:, 2) = torque4(:, 2) + LR * Kp * ans.ep4 + LR * Kd * ans.ev4;
        torque5(:, 2) = torque5(:, 2) + LR * Kp * ans.ep5 + LR * Kd * ans.ev5;
        torque6(:, 2) = torque6(:, 2) + LR * Kp * ans.ep6 + LR * Kd * ans.ev6;
        torque7(:, 2) = torque7(:, 2) + LR * Kp * ans.ep7 + LR * Kd * ans.ev7;
        
        sim('WAM_Arm.slx');
    end
    errorR = sqrt(sum(ans.ep1.^2) + sum(ans.ep2.^2) + sum(ans.ep3.^2) + sum(ans.ep4.^2) + sum(ans.ep5.^2) + sum(ans.ep6.^2) + sum(ans.ep7.^2));
    
    % Update search interval
    if errorL < errorR
        b = LR;
    else
        a = LL;
    end
end

% Compute optimal learning gain L
L_opt = (a + b) / 2;
fprintf('L_opt = %.2f\n', L_opt);