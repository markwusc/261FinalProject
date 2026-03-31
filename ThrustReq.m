function [T_R, P_R] = ThrustReq(W_To, S, b, CL_max, CD_0, h_wing)
    %{
    This function takes the plane geometry as inputs and calculates the
    thrust/power requirement the plane needs due to the 500ft takeoff
    requirement.
    Set h_wing = 5m if unsure.
    %}
     
    % Weight and geometry
    W = W_To; % [N]
    rho = 1.225; % density of air at sea level
    AR = b^2/S;
    e = 0.9; % given by design report document
    k = 1/(pi*AR*e);
    V_stall = sqrt((2*W)/(rho*S*CL_max));
    V_Lo = 1.2*V_stall;
    V_avg = 1.2*V_stall*0.7;
    mu_r = 0.4; % coefficient of rolling friction, 0.4 due to gravel runway
    g = 9.81;
    h = 15.24; % height of 50ft clearance [m]

    % ground effect calc
    phi = (16*h_wing/b)^2 / ((16*h_wing/b)^2 + 1);

    % dLo drag calc
    q_avg = 0.5*rho*V_avg^2;
    D_phi = q_avg*S*CD_0 + phi*k*W^2/(q_avg*S);
    L = q_avg*S*CL_max;

    % climb drag calc
    q_Lo = 0.5*rho*V_Lo;
    D_Lo = q_Lo*S*CD_0 + k*W^2/(q_Lo*S); % drag at liftoff/climb [N]
    
    d_To = 130; % initial takeoff distance guess [m]
    T_A_est = 18000; % initial thrust available estimate [N]
    d_To_max = 152; % max takeoff distance [m] (498.7ft)
    tol = 1; % max takeoff distance tolerance [m]
    count = 0;

    % iteration loop to find T_R
    while d_To > d_To_max || d_To < d_To_max - tol
        d_Lo = (1.44*W^2)/(rho*g*S*CL_max*(T_A_est-(D_phi-mu_r*(W-L))));
        ROC = V_Lo*(T_A_est-D_Lo)/W;
        d_extra = h*V_Lo/ROC; % distance needed to clear obstacle
        d_To = d_Lo+d_extra;
        
        if d_To > d_To_max
            T_A_est = T_A_est + 1;
        elseif d_To < d_To_max - tol
            T_A_est = T_A_est - 1;
        end
        
        count = count+1;
    end
    T_R = T_A_est;
    P_R = T_A_est * V_Lo;
    fprintf('Iteration count: %d\n', count)
    fprintf('The thrust requirement is : %3.2f kN (%3.1f lbf)\n', T_R/1000, T_R*0.2248089)
    fprintf('The power requirement is : %3.2f kW (%3.1f hp)\n', P_R/1000, P_R*0.001341022)
end