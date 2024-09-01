function drawAircraft(U)
%ANIMATEPLANE Summary of this function goes here
%   Detailed explanation goes here
    pn       = U(1);       % inertial North position     
    pe       = U(2);       % inertial East position
    pd       = U(3);           
    u        = U(4);       
    v        = U(5);       
    w        = U(6);       
    phi      = U(7);       % roll angle         
    theta    = U(8);       % pitch angle     
    psi      = U(9);       % yaw angle     
    p        = U(10);       % roll rate
    q        = U(11);       % pitch rate     
    r        = U(12);       % yaw rate    
    t        = U(13);       % time

    persistent s_handle
    n = pn + u*t;
    e = pe + v*t;
    d = pd + w*t;
    phi = phi + p*t;
    theta = theta + q*t;
    psi = psi + r*t;

    [V, F] = getVertexFaces();
    if t==0
        figure(1), clf
        grid on 
        axis equal
        xlim([-20 20])
        ylim([-20 20])
        zlim([-20 20])
        view(30,30)  % set the vieew angle for figure
        s_handle = draw(V,F,n,e,d,phi,theta,psi,s_handle,t==0);
        title('Spacecraft')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        hold on
    else
        draw(V,F,n,e,d,phi,theta,psi,s_handle,t==0);
    end

   
end

function handl = draw(V,F,pn,pe,pd,phi,theta,psi,handle,b)
    Rned2xyz = [0 1 0; 1 0 0; 0 0 -1];
    V = rotate(V,phi,theta,psi);
    V = translate(V',pn,pe,pd);
    V = Rned2xyz*V;



       
    if b
        handl = patch('Vertices',V','Faces',F,'FaceColor','red');
    else
        set(handle,'Vertices',V','Faces',F);
        %handle.Vertices = V; handle.Faces = F;
        drawnow        
    end
    
    pause(.2)
end

function XYZ = rotate(XYZ,phi,theta,psi)
    R_roll = [...
          1, 0, 0;...
          0, cos(phi), -sin(phi);...
          0, sin(phi), cos(phi)];
    R_pitch = [...
          cos(theta), 0, sin(theta);...
          0, 1, 0;...
          -sin(theta), 0, cos(theta)];
    R_yaw = [...
          cos(psi), -sin(psi), 0;...
          sin(psi), cos(psi), 0;...
          0, 0, 1];
    R = R_roll*R_pitch*R_yaw;
    XYZ =  XYZ * R;%euler2rotm([phi theta psi], "XYZ");
end

function XYZ = translate(XYZ,pn,pe,pd)
    XYZ = XYZ + repmat([pn;pe;pd],1,size(XYZ,2));
end

function [V, F] = getVertexFaces()
    fuse_l1 = 1; fuse_l2 = 0.5; fuse_l3 = 7; fuse_h = 1; fuse_w = 1;  
    tailwing_w = 4; tailwing_l = 1; wing_l = 2; wing_w = 10; tail_h = 1.5;

    V = [
        fuse_l1, 0, 0;
        fuse_l2,fuse_w/2,fuse_h/2
        fuse_l2,-fuse_w/2,fuse_h/2
        fuse_l2,-fuse_w/2,-fuse_h/2
        fuse_l2,fuse_w/2,-fuse_h/2
        -fuse_l3, 0, 0;
        0,wing_w/2,0;
        -wing_l,wing_w/2,0;
        -wing_l,-wing_w/2,0;
        0,-wing_w/2,0;
        -fuse_l3,tailwing_w/2, 0;
        -fuse_l3+tailwing_l,tailwing_w/2, 0;
        -fuse_l3+tailwing_l,-tailwing_w/2, 0;
        -fuse_l3,-tailwing_w/2, 0;
        -fuse_l3+tailwing_l, 0, 0;
        -fuse_l3, 0, -tail_h;
    ];

    F = [
        1,3,4,1; 1,2,5,1; 1,2,3,1; 1,4,5,1; % nose
        3,4,6,3; 2,5,6,2; 2,3,6,2; 5,4,6,5; % fuselage
        7,8,9,10; % wing
        11,12,13,14; % tail
        15,6,16,15 % rudder section
    ];
end