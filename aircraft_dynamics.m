function aircraft_dynamics(s)
%AIRCRAFT_KINEMATICS Summary of this function goes here
%   Detailed explanation goes here
    setup(s);
end
function setup(s)
    s.NumDialogPrms = 44;
    s.NumContStates = 12;
    s.NumInputPorts = 1;
    s.NumOutputPorts = 1;
    s.SetPreCompInpPortInfoToDynamic;
    s.SetPreCompOutPortInfoToDynamic;
    s.InputPort(1).Dimensions = [4,1];
    s.OutputPort(1).Dimensions = [12,1];
    s.SampleTimes = [0 0];
    s.SimStateCompliance = 'DefaultSimState';
    s.RegBlockMethod('InitializeConditions',    @InitializeConditions);  
    s.RegBlockMethod('Outputs',                 @Outputs);  
    s.RegBlockMethod('Derivatives',             @Derivatives);
end

function Outputs(s)
    s.OutputPort(1).Data = s.ContStates.Data;
end

function InitializeConditions(s)
    s.ContStates.Data = zeros(12,1);
end

function Derivatives(s)
    delta = s.InputPort(1).Data;
    m = s.DialogPrm(1).Data;
    J = s.DialogPrm(2).Data;
    S = s.DialogPrm(3).Data;
    b = s.DialogPrm(4).Data;
    rho = s.DialogPrm(5).Data;
    C_l_o = s.DialogPrm(6).Data;
    C_l_beta = s.DialogPrm(7).Data;
    C_l_delta_a = s.DialogPrm(8).Data;
    C_l_delta_r = s.DialogPrm(9).Data;
    C_l_p = s.DialogPrm(10).Data;
    C_l_r = s.DialogPrm(11).Data;
    k_Omega = s.DialogPrm(12).Data;
    k_T_p = s.DialogPrm(13).Data;
    c = s.DialogPrm(14).Data;
    C_m_o = s.DialogPrm(15).Data;
    C_m_alpha = s.DialogPrm(16).Data;
    C_m_delta_e = s.DialogPrm(17).Data;
    C_m_q = s.DialogPrm(18).Data;
    C_n_o = s.DialogPrm(19).Data;
    C_n_beta = s.DialogPrm(20).Data;
    C_n_delta_a = s.DialogPrm(21).Data;
    C_n_delta_r = s.DialogPrm(22).Data;
    C_n_p = s.DialogPrm(23).Data;
    C_n_r = s.DialogPrm(24).Data;
    C_L_delta_e= s.DialogPrm(25).Data;
    M= s.DialogPrm(26).Data;
    alpha_0= s.DialogPrm(27).Data;
    C_L_0= s.DialogPrm(28).Data;
    ef= s.DialogPrm(29).Data;
    C_D_q= s.DialogPrm(30).Data;
    C_D_p= s.DialogPrm(31).Data;
    C_D_delta_e= s.DialogPrm(32).Data;
    C_L_q= s.DialogPrm(33).Data;
    C_prop= s.DialogPrm(34).Data;

    S_prop= s.DialogPrm(35).Data;
    k_motor= s.DialogPrm(36).Data;
    g= s.DialogPrm(37).Data;
    C_Y_o= s.DialogPrm(38).Data;
    C_Y_beta= s.DialogPrm(39).Data;
    C_Y_delta_a= s.DialogPrm(40).Data;
    C_Y_delta_r= s.DialogPrm(41).Data;
    C_Y_p= s.DialogPrm(42).Data;
    C_Y_r= s.DialogPrm(43).Data;

    x = s.ContStates.Data
    F = kinetics(...
        x,delta,S,b,rho,C_l_o,C_l_beta,C_l_delta_a,C_l_delta_r,C_l_p,C_l_r,k_Omega,k_T_p,c,C_m_o,C_m_alpha,C_m_delta_e,...
        C_m_q,C_n_o,C_n_beta,C_n_delta_a,C_n_delta_r,C_n_p,C_n_r, C_L_delta_e,M,alpha_0,C_L_0,ef,C_D_q,C_D_p,C_D_delta_e,C_L_q,C_prop,...
        S_prop,k_motor,g,m,C_Y_o,C_Y_beta,C_Y_delta_a,C_Y_delta_r,C_Y_p,C_Y_r...
    );
    s.Derivatives.Data = kinematics(x,F,m,J);
end

function [F] = kinetics(x,delta,S,b,rho,C_l_o,C_l_beta,C_l_delta_a,C_l_delta_r,C_l_p,C_l_r,k_Omega,k_T_p,c,C_m_o,C_m_alpha,C_m_delta_e,...
    C_m_q,C_n_o,C_n_beta,C_n_delta_a,C_n_delta_r,C_n_p,C_n_r, C_L_delta_e,M,alpha_0,C_L_0,ef,C_D_q,C_D_p,C_D_delta_e,C_L_q,C_prop,...
    S_prop,k_motor,g,m,C_Y_o,C_Y_beta,C_Y_delta_a,C_Y_delta_r,C_Y_p,C_Y_r)
    u=x(4);v=x(5);w=x(6);phi=x(7);theta=x(8);psi=x(9);p=x(10);q=x(11);r=x(12);
    Vr = [u;v;w];
    ur=Vr(1);vr=Vr(2);wr=Vr(3);Va=sqrt(ur^2+vr^2+wr^2),alpha=atan(wr/ur),beta=sin(vr/Va),
    delta_a=delta(1);delta_e=delta(2);delta_r=delta(3);delta_t=delta(4);
    F = zeros(6,1);
   

    x
    Va = 1; alpha=0;beta=0;
    F(1:3) = [
        (S*Va^2*rho*sin(alpha)*(C_L_delta_e*delta_e - ((exp(M*(alpha + alpha_0)) + exp(-M*(alpha - alpha_0)) + 1)/((exp(M*(alpha + alpha_0)) + 1)*(exp(-M*(alpha - alpha_0)) + 1)) - 1)*(C_L_0 + (pi*alpha*b^2)/(S*(sqrt(b^4/(4*S^2) + 1) + 1))) + (C_L_q*c*q)/(2*Va) + (2*cos(alpha)*sign(alpha)*sin(alpha)^2*(exp(M*(alpha + alpha_0)) + exp(-M*(alpha - alpha_0)) + 1))/((exp(M*(alpha + alpha_0)) + 1)*(exp(-M*(alpha - alpha_0)) + 1))))/2 - (C_prop*S_prop*rho*(Va^2 - delta_t^2*k_motor^2))/2 - (S*Va^2*rho*cos(alpha)*(C_D_p + C_D_delta_e*delta_e + (C_D_q*c*q)/(2*Va) + (S*(C_L_0 + (pi*alpha*b^2)/(S*(sqrt(b^4/(4*S^2) + 1) + 1)))^2)/(b^2*ef*pi)))/2 - g*m*sin(theta);
        (S*Va*rho*(2*C_Y_o*Va + 2*C_Y_beta*Va*beta + 2*C_Y_delta_a*Va*delta_a + 2*C_Y_delta_r*Va*delta_r + C_Y_p*b*p + C_Y_r*b*r))/4 + g*m*cos(theta)*sin(phi); 
        g*m*cos(phi)*cos(theta) - (S*Va^2*rho*sin(alpha)*(C_D_p + C_D_delta_e*delta_e + (C_D_q*c*q)/(2*Va) + (S*(C_L_0 + (pi*alpha*b^2)/(S*(sqrt(b^4/(4*S^2) + 1) + 1)))^2)/(b^2*ef*pi)))/2 - (S*Va^2*rho*cos(alpha)*(C_L_delta_e*delta_e - ((exp(M*(alpha + alpha_0)) + exp(-M*(alpha - alpha_0)) + 1)/((exp(M*(alpha + alpha_0)) + 1)*(exp(-M*(alpha - alpha_0)) + 1)) - 1)*(C_L_0 + (pi*alpha*b^2)/(S*(sqrt(b^4/(4*S^2) + 1) + 1))) + (C_L_q*c*q)/(2*Va) + (2*cos(alpha)*sign(alpha)*sin(alpha)^2*(exp(M*(alpha + alpha_0)) + exp(-M*(alpha - alpha_0)) + 1))/((exp(M*(alpha + alpha_0)) + 1)*(exp(-M*(alpha - alpha_0)) + 1))))/2
    ];
    F(4:6) = [
        (S*Va*b*rho*(2*C_l_o*Va + 2*C_l_beta*Va*beta + 2*C_l_delta_a*Va*delta_a + 2*C_l_delta_r*Va*delta_r + C_l_p*b*p + C_l_r*b*r))/4 - delta_t^2*k_Omega^2*k_T_p;
        (S*Va^2*c*rho*(C_m_o + C_m_alpha*alpha + C_m_delta_e*delta_e + (C_m_q*c*q)/(2*Va)))/2;
        (S*Va*b*rho*(2*C_n_o*Va + 2*C_n_beta*Va*beta + 2*C_n_delta_a*Va*delta_a + 2*C_n_delta_r*Va*delta_r + C_n_p*b*p + C_n_r*b*r))/4
    ];
    F
end

function [xdot] = kinematics(x,F,m,J)
    u=x(4);v=x(5);w=x(6);phi=x(7);theta=x(8);psi=x(9);p=x(10);q=x(11);r=x(12);
    Fx=F(1);Fy=F(2);Fz=F(3);Ml=F(4);Mm=F(5);Mn=F(6);Jx=J(1);Jy=J(2);Jz=J(3);Jxz=J(4);

    xdot = zeros(12,1);
    xdot(1:3) = [
        w*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - v*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + u*cos(psi)*cos(theta);
        v*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - w*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + u*cos(theta)*sin(psi);
        w*cos(phi)*cos(theta) - u*sin(theta) + v*cos(theta)*sin(phi)
    ];
    xdot(4:6) = [
        r*v - q*w + Fx/m;
        p*w - r*u + Fy/m;
        q*u - p*v + Fz/m
    ];
    xdot(7:9) = [
        p + (r*cos(phi)*sin(theta))/cos(theta) + (q*sin(phi)*sin(theta))/cos(theta);
        q*cos(phi) - r*sin(phi);
        (r*cos(phi))/cos(theta) + (q*sin(phi))/cos(theta)
    ];
    xdot(10:12) = [
        -(Jz*Ml + Jxz*Mn - Jxz^2*q*r - Jz^2*q*r + Jx*Jxz*p*q - Jxz*Jy*p*q + Jxz*Jz*p*q + Jy*Jz*q*r)/(Jxz^2 - Jx*Jz);
        (Mm - Jxz*p^2 + Jxz*r^2 - Jx*p*r + Jz*p*r)/Jy;
        -(Jxz*Ml + Jx*Mn + Jx^2*p*q + Jxz^2*p*q - Jx*Jy*p*q - Jx*Jxz*q*r + Jxz*Jy*q*r - Jxz*Jz*q*r)/(Jxz^2 - Jx*Jz)
    ];
end