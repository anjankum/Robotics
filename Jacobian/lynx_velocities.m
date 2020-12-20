function [ v05, w05 ] = lynx_velocities( thetas, thetadot )
%LYNX_VELOCITIES The input to the function will be:
%    thetas: The joint angles of the robot in radians - 1x5 matrix
%    thetadot: The rate of change of joint angles of the robot in radians/sec - 1x5 matrix
%    The output has 2 parts:
%    v05 - The linear velocity of frame 5 with respect to frame 0, expressed in frame 0.
%    w05 - The angular velocity of frame 5 with respect to frame 0, expressed in frame 0.
%    They are both 1x3 matrices of the form [x y z] for a vector xi + yj + zk

    %% YOUR CODE GOES HERE
    theta1 = thetas(1); theta2 = thetas(2); theta3 = thetas(3);
    theta4 = thetas(4); theta5 = thetas(5);
    
    A0_0 = eye(4);
    A0_1 = A0_0*compute_dh_matrix(0,-pi/2,3,theta1);
    A0_2 = A0_1*compute_dh_matrix(5.75,0,0,theta2 - pi/2);
    A0_3 = A0_2*compute_dh_matrix(7.375,0,0,theta3 + pi/2);
    A0_4 = A0_3*compute_dh_matrix(0,-pi/2,0,theta4 - pi/2);
    A0_5 = A0_4*compute_dh_matrix(0,0,4.125,theta5);
    
    J1=[cross(A0_0(1:3,3), (A0_5(1:3,4) - A0_0(1:3,4)));
        A0_0(1:3,3)];
    J2=[cross(A0_1(1:3,3), (A0_5(1:3,4) - A0_1(1:3,4)));
        A0_1(1:3,3)];
    J3=[cross(A0_2(1:3,3), (A0_5(1:3,4) - A0_2(1:3,4)));
        A0_2(1:3,3)];    
    J4=[cross(A0_3(1:3,3), (A0_5(1:3,4) - A0_3(1:3,4)));
        A0_3(1:3,3)];
    J5=[cross(A0_4(1:3,3), (A0_5(1:3,4) - A0_4(1:3,4)));
        A0_4(1:3,3)];
    
    J = [J1 J2 J3 J4 J5];

     v05 = zeros(1, 3);
     w05 = zeros(1, 3);
     
     vel = J * thetadot';
     
     v05 = vel(1:3)';
     w05 = vel(4:6)';
end


function A = compute_dh_matrix(r, alpha, d, theta)

    %% Your code from the first part of this assignment goes here
    %% You can use this function in the lynx_fk function
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