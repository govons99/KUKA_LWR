%% INIT
clc;

clear all;

parameters;

while(t0 < tf)
    disp(t0);  

    %% TRAJ (CIRCLE)
    
    p_ref(1,1) = p_0(1) -  0.1 * (1 - cos(frequency * (t0)));
    dp_ref(1,1) = dp_0(1) - 0.1 * frequency * sin(frequency * (t0));
    d2p_ff(1,1) = d2p_0(1) - 0.1 * frequency * frequency * cos(frequency * (t0));

    p_ref(2,1) = p_0(2) -  0.1 * (sin(frequency * (t0)));
    dp_ref(2,1) = dp_0(2) - 0.1 * frequency * cos(frequency * (t0));
    d2p_ff(2,1) = d2p_0(2) + 0.1 * frequency * frequency * sin(frequency * (t0));
    
    p_ref(3,1) = p_0(3);
    dp_ref(3,1) = dp_0(3);
    d2p_ff(3,1) = d2p_0(3);
     
    Jold = J;

    qold = q0;
    
    p = f(q0(1),q0(2),q0(3),q0(4),q0(5),q0(6));         
    
    J = J_LWR(q0(1),q0(2),q0(3),q0(4),q0(5),q0(6));
    
    dJ = dJdt_LWR(q0(8),q0(9),q0(10),q0(11),q0(12),q0(13),q0(1),q0(2),q0(3),q0(4),q0(5),q0(6));
    
    [~,S,~] = svd(J);
    
    sigma = min(diag(S));
    
    dp = J * q0(8:14)';
    
    d2p = dJ * q0(8:14)' + J * accs(end,:)';
    
    err_p = p_ref - p;
    
    err_dp = dp_ref - dp;
    
    
    %% PD KINEMATIC    
    d2p_ref = Kp * err_p + Kd * err_dp + d2p_ff;
    
    tspan =[t0,t0+DeltaT];
    
    
    %% EXACT PROJECTOR
    Pj = eye(7) - pinv(J) * J;  
    
    % Friction calculation
    
    %% TO BE FIXED
%     friction =  0.1*(-A_friction .* sign(q0(8:14)) - 0.001 * q0(8:14));
    friction =  0.0;   
    
    %%DAMPED-LEAST SQUARE    
    
%     A = (J' * J + damping * eye(length(q0(1:7))));
%     B = J' * (d2p_ref - dJ * q0(8:14)');              
%     X1 = lsqminnorm(A,B);    
%     Uref_task = X1';
    
    %% PSEUDOINVERSE
    Uref_task = (pinv(J) * (d2p_ref - dJ * q0(8:14)'))';
    
    uopt = zeros(7,1);
    
    Uref_redundant = (Pj * uopt)';
    
    Uref = Uref_task + Uref_redundant;                     
    
    TauFL = gravityTorque(controller,q0(1:7)) + velocityProduct(controller,q0(1:7),q0(8:14)) + (massMatrix(controller,q0(1:7)) * Uref')';                
    
    M = massMatrix(controller,q0(1:7));
    
    n = gravityTorque(controller,q0(1:7)) + velocityProduct(controller,q0(1:7),q0(8:14));
        
    %% COMPUTED TORQUE FL
    
    acc = (inv(massMatrix(controller,q0(1:7))) * (TauFL - friction - velocityProduct(controller,q0(1:7),q0(8:14)) - gravityTorque(controller,q0(1:7)))')';                    
    
    % ARRAYS UPDATE
    accs = vertcat(accs,acc);
    
    accs_ref = vertcat(accs_ref,[Uref,Uref_task, Uref_redundant]);            
    
    task_vec = vertcat(task_vec, [p',dp',d2p',p_ref',dp_ref',d2p_ref']);
    
    torque_fl = vertcat(torque_fl,TauFL);
    
    singular_values = vertcat(singular_values,sigma);       
    
    % EULER INTEGRATION    
    q0(1:7)  = q0(1:7) + q0(8:14) * DeltaT;    
    q0(8:14) = acc * DeltaT + q0(8:14);                

    t0 = t0+DeltaT;
    
    joints = vertcat(joints,q0);    
    
    time = vertcat(time,t0);
    
    %Updating the step
    index = index + 1;    
end