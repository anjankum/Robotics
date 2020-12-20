function [ v06, w06 ] = puma_velocities( thetas, thetadot )
%PUMA_VELOCITIES The input to the function will be:
%    thetas: The joint angles of the robot in radians - 1x6 matrix
%    thetadot: The rate of change of joint angles of the robot in radians/sec - 1x6 matrix
%    The output has 2 parts:
%    v06 - The linear velocity of frame 6 with respect to frame 0, expressed in frame 0.
%    w06 - The angular velocity of frame 6 with respect to frame 0, expressed in frame 0.
%    They are both 1x3 matrices of the form [x y z] for a vector xi + yj + zk

    %% YOUR CODE GOES HERE
    theta1 = thetas(1); theta2 = thetas(2); theta3 = thetas(3);
    theta4 = thetas(4); theta5 = thetas(5); theta6 = thetas(6);
    a = 13; b = 2.5; c = 8; d = 2.5; e = 8; f = 2.5;

    A0_0 = eye(4);
    A0_1 = A0_0*compute_dh_matrix(0,-pi/2,a,theta1 + pi);
    A0_2 = A0_1*compute_dh_matrix(-c,0,-b,theta2);
    A0_3 = A0_2*compute_dh_matrix(0,-pi/2,-d,theta3 + pi);
    A0_4 = A0_3*compute_dh_matrix(0,-pi/2,e,theta4 + pi);
    A0_5 = A0_4*compute_dh_matrix(0,-pi/2,0,theta5 + pi);
    A0_6 = A0_5*compute_dh_matrix(0,0,f,theta6);

    J1=[cross(A0_0(1:3,3), (A0_6(1:3,4) - A0_0(1:3,4)));
        A0_0(1:3,3)];
    J2=[cross(A0_1(1:3,3), (A0_6(1:3,4) - A0_1(1:3,4)));
        A0_1(1:3,3)];
    J3=[cross(A0_2(1:3,3), (A0_6(1:3,4) - A0_2(1:3,4)));
        A0_2(1:3,3)];    
    J4=[cross(A0_3(1:3,3), (A0_6(1:3,4) - A0_3(1:3,4)));
        A0_3(1:3,3)];
    J5=[cross(A0_4(1:3,3), (A0_6(1:3,4) - A0_4(1:3,4)));
        A0_4(1:3,3)];
    J6=[cross(A0_5(1:3,3), (A0_6(1:3,4) - A0_5(1:3,4)));
        A0_5(1:3,3)];
    
    J = [J1 J2 J3 J4 J5 J6];
     
     vel = J * thetadot';
     
     v06 = vel(1:3)';
     w06 = vel(4:6)';   
end

function A = compute_dh_matrix(r, alpha, d, theta)

    % Your code from part 1 of the assignment goes here
    A = eye(4);
    ctheta = cos(theta);
    stheta = sin(theta);
    calpha = cos(alpha);
    salpha = sin(alpha);

    A(1,1) = ctheta;A(1,2) = -stheta*calpha;A(1,3) = stheta*salpha;A(1,4) = r*ctheta;
    A(2,1) = stheta;A(2,2) = ctheta*calpha;A(2,3) = -ctheta*salpha;A(2,4) = r*stheta;
    A(3,1) = 0;A(3,2) = salpha;A(3,3) = calpha;A(3,4) = d;
    A(4,1) = 0;A(4,2) = 0;A(4,3) = 0;A(4,4) = 1;
end