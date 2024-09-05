function aircraft_kinematics(s)
%AIRCRAFT_KINEMATICS Summary of this function goes here
%   Detailed explanation goes here
%    switch flag
%        case 0
    setup(s)
end
function setup(s)
    s.NumDialogPrms = 2;

    s.NumContStates = 12

    s.NumInputPorts = 1;
    s.NumOutputPorts = 1;

    s.SetPreCompInpPortInfoToDynamic;
    s.SetPreCompOutPortInfoToDynamic;

    s.InputPort(1).Dimensions = [12,1];
    s.OutputPort(1).Dimensions = [12,1];

    s.SampleTimes = [0 0];

    s.SimStateCompliance = 'DefaultSimState';
end
function outputs(s)
    s.OutputPort(n).Data
    s.OutputPort(1).Data = block.ContStates.Data;
end
function InitConditions(s)
    s.ContStates.Data(1) = zeros(12,1);
end

function Derivative(s)
    F = block.InputPort(1).Data;
    m = block.DialogPrm(1).Data;
    J = block.DialogPrm(2).Data;
    x = block.ContStates.Data;
    s.Derivatives.Data = kinematics(x,F,m,J);
end
%        case 1
%            u
%            sys = kinematics(x,u,m,J);
%        case 3
%            u
%            sys = x;
%        otherwise
%            sys=[];
%    end

%end
%function [F] = kinetics(x,delta_input,Vr,rho)
%    u=x(4);v=x(5);w=x(6);phi=x(7);theta=x(8);psi=x(9);p=x(10);q=x(11);r=x(12);
%    ur=Vr(1);vr=Vr(2);wr=Vr(3);Va=sqrt(ur^2+vr^2+wr^2);alpha=atan(wr/ur);beta=sin(vr/Va);
%    F = zeros(6,1);
%
%    F=1;
%end

function [xdot] = kinematics(x,F,m,J)
    u=x(4);v=x(5);w=x(6);phi=x(7);theta=x(8);psi=x(9);p=x(10);q=x(11);r=1;%x(12);
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
    %xdot(10:12) = [
    %    -(Jz*Ml + Jxz*Mn - Jxz^2*q*r - Jz^2*q*r + Jx*Jxz*p*q - Jxz*Jy*p*q + Jxz*Jz*p*q + Jy*Jz*q*r)/(Jxz^2 - Jx*Jz);
    %    (Mm - Jxz*p^2 + Jxz*r^2 - Jx*p*r + Jz*p*r)/Jy;
    %    -(Jxz*Ml + Jx*Mn + Jx^2*p*q + Jxz^2*p*q - Jx*Jy*p*q - Jx*Jxz*q*r + Jxz*Jy*q*r - Jxz*Jz*q*r)/(Jxz^2 - Jx*Jz)
    %];
end