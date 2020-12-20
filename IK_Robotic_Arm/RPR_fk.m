function [ pos, R ] = RPR_fk( theta1, d2, theta3 )
%RPR_FK Write your code here. The input to the function will be the joint
%    angles of the robot in radians, and the extension of the prismatic joint in inches.
%    The output includes: 
%    1) The position of the end effector and the position of 
%    each of the joints of the robot, as explained in the question.
%    2) The rotation matrix R_03, as explained in the question.

    %% YOUR CODE GOES HERE
    
    
    pos = zeros(4, 3);
    R = eye(3);
    A0 = eye(4);
    A1 = A0*compute_dh_matrix(0,-(3*pi)/4,10,theta1);
    A2 = A1*compute_dh_matrix(0,-pi/2,d2,- pi/2);
    A3 = A2*compute_dh_matrix(0,pi/2,0,theta3 + pi/4);
    A4 = A3*compute_dh_matrix(0,0,5,pi/2);
    
    R = A4(1:3,1:3);
    
    pos = [0 0 0; A1(1,4) A1(2,4) A1(3,4);
        A2(1,4) A2(2,4) A2(3,4);
        A4(1,4) A4(2,4) A4(3,4)];
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