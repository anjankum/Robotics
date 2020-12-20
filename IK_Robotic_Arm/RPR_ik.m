function [ ik_sol ] = RPR_ik( x, y, z, R )
%RPR_IK Write your code here. The input to the function will be the position of
%    the end effector (in inches) in the world frame, and the 
%    Rotation matrix R_30 as described in the question.
%    The output must be the joint angles and extensions of the robot to achieve 
%    the end effector position and orientation.

    %% YOUR CODE GOES HERE
    ik_sol = ones(1,3);
     
%   theta1
    ik_sol(1)= atan2(R(2,1), R(1,1));

%   theta3 (utilize decoupling method)
    s1= sin(ik_sol(1));
    c1 = cos(ik_sol(1));
    v=sin(pi/4);

     R0_2=[s1*v s1*v c1;
         c1*v -c1*v s1;
         v v 0];
     
    R2_4 = R0_2\R;
     
    ik_sol(3)= atan2(-R2_4(2,2), -R2_4(1,2)) - (pi/4);   

%   d2
    ik_sol(2) = sqrt(2)*(10 - z + 5 * sin(ik_sol(3)));  
end
