function [ pos ] = lynx_fk( theta1, theta2, theta3, theta4, theta5, g )
%LYNX_FK The input to the function will be the joint
%    angles of the robot in radians, and the distance between the gripper pads in inches.
%    The output must contain 10 positions of various points along the robot arm as specified
%    in the question.

    %% YOUR CODE GOES HERE
    pos = zeros(10, 3);
    
    A0 = eye(4);
    A1 = A0*compute_dh_matrix(0,-pi/2,3,theta1);
    A2 = A1*compute_dh_matrix(5.75,0,0,theta2 - pi/2);
    A3 = A2*compute_dh_matrix(7.375,0,0,theta3 + pi/2);
    A4 = A3*compute_dh_matrix(0,-pi/2,0,theta4 - pi/2);
    A5 = A4*compute_dh_matrix(0,0,4.125,theta5);
    
    A01 = A0(1:3,4)';
    A11 = A1(1:3,4)';
    A21 = A2(1:3,4)';
    A31 = A3(1:3,4)';
    A41 = A4(1:3,4)';
    A51 = A5*[0 0 -1.125 1]';
    A52 = A5*[.5*g 0 -1.125 1]';
    A53 = A5*[-.5*g 0 -1.125 1]';
    A54 = A5*[.5*g 0 0 1]';
    A55 = A5*[-.5*g 0 0 1]';
    
    A511 = A51(1:3,1)';
    A512 = A52(1:3,1)';
    A513 = A53(1:3,1)';
    A514 = A54(1:3,1)';
    A515 = A55(1:3,1)';
    
    pos = [A01;A11;A21;A31;A41;A511;A512;A513;A514;A515];    
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