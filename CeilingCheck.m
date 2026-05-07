function [state] = CeilingCheck(P_A_sl, W, S, AR, CD0)
    % Does a check given plane geometry and engine selection to see if
    % altitude requirements are met.
    rho_sl = 1.225; % density of air at sea level
    e = 0.9; % project constant
    k = 1/(pi*e*AR);
    CL_Dmin = sqrt(CD0/k);

    h_req = 5487; % 18000ft altitude in meters
    [~, rho_req, ~, ~] = StdAtmos(h_req); % calls Standard Atmosphere
    V_Dmin = sqrt(2*W/(rho_req*S*CL_Dmin));
    
    sigma = rho_req/rho_sl;
    
    q_req = 0.5 * rho_req * V_Dmin^2;
    CDmin = 2*CD0;
    Dmin = q_req * S * CDmin;
    P_R = Dmin * V_Dmin;
    
    P_A = sigma * P_A_sl;

    if P_R > P_A
        state = false;
    else
        state = true;
    end
end

