function [state] = RocCheck(P_A_sl, W, S, AR, CD0, h)
    ROC_needed = 4.064; % required rate of climb [m/s]

    rho_sl = 1.225; % density of air at sea level
    e = 0.9; % project constant
    k = 1/(pi*e*AR);
    CD_Pmin = 4*CD0;
    CL_Pmin = sqrt(3*CD0/k);

    [~, rho, ~, ~] = StdAtmos(h);
    sigma = rho/rho_sl;
    V_Pmin = sqrt(2*W/(rho*S*CL_Pmin));
    q_Pmin = 0.5 * rho * V_Pmin^2;

    D_Pmin = q_Pmin * S * CD_Pmin;
    P_A = sigma * P_A_sl;
    P_R = D_Pmin * V_Pmin;

    ROC = (P_A - P_R)/W;
    if ROC >= ROC_needed
        state = true;
    else
        state = false;
    end
end