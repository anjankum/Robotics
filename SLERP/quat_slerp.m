function [ q_int ] = quat_slerp( q0, q1, steps )
%QUAT_SLERP Perform SLERP between two quaternions and return the intermediate quaternions
%   Usage: [ q_int ] = quat_slerp( q0, q1, steps )
%   Inputs:
%       q0 is the quaternion representing the starting orientation, 1x4 matrix
%       q1 is the quaternion representing the final orientation, 1x4 matrix
%       steps is the number of intermediate quaternions required to be returned, integer value
%       The first step is q0, and the last step is q1
%   Output:
%       q_int contains q0, steps-2 intermediate quaternions, q1
%       q_int is a (steps x 4) matrix

    %% Your code goes here
    q_int = zeros(steps, 4);
    
    thet = 2*acos(dot(q0,q1));
    
    if(thet > pi)
        q0=-q0;
        thet = 2*acos(dot(q0,q1));
    end

    a = 1/sin(thet*.5);
    for i = 0:1/(steps-1):1
        if(i==0)
            index = 1;
        elseif (i==1)
            index = steps;
        else
            index = round(i*(steps-1) + 1);
        end

        q_int(index,:) =  a*(sin((1-i)*thet*.5)*q0 + sin(i*thet*.5)*q1);   
    end
end
