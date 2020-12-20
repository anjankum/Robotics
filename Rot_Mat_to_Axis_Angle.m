% In this function, you need to convert the rotation matrix R into axis-angle form

function [axang] = rotm2axang(R)
 
    %% Your code starts here   
 
    vec = [0 0 0];
    %theta = 0;
    theta = acos((trace(R)-1)/2)
        
    if (theta == 0)
        vec = [NaN NaN NaN];
    elseif (theta == pi)
        vecx1 = sqrt((R(1,1)+1)/2);
        vecx2 = - sqrt((R(1,1)+1)/2);
        vecy1 = sqrt((R(2,2)+1)/2);
        vecy2 = - sqrt((R(2,2)+1)/2);
        vecz1 = sqrt((R(3,3)+1)/2);
        vecz2 = -sqrt((R(3,3)+1)/2);
        
        if(vecx1 ~= 0 || vecx2 ~= 0)
            vecy1 = R(1,2)/(2*vecx1);
            vecy2 = R(1,2)/(2*vecx2);
            
            vecz1 = R(1,3)/(2*vecx1);
            vecz2 = R(1,3)/(2*vecx2);
        elseif(vecy1 ~= 0 || vecy2 ~= 0)
            vecx1 = R(2,1)/(2*vecy1);
            vecx2 = R(2,1)/(2*vecy2);
            
            vecz1 = R(2,3)/(2*vecy1);
            vecz2 = R(2,3)/(2*vecy2);
        elseif ( vecz1 ~=0 || vecz2 ~= 0)
            vecx1 = R(3,1)/(2*vecz1);
            vecx2 = R(3,1)/(2*vecz2);
            
            vecy1 = R(3,2)/(2*vecz1);
            vecy2 = R(3,2)/(2*vecz2);
        end
        
        vec = [vecx1 vecy1 vecz1;
               vecx2 vecy2 vecz2];
        theta = [pi;pi];
        
    else      
        const = 1/(2*sin(theta));
        vecx = const*(R(3,2) - R(2,3));
        vecy = const*(R(1,3) - R(3,1));
        vecz = const*(R(2,1) - R(1,2));
        vec = [vecx vecy vecz];
        vec = vec/norm(vec);
    end      
    axang = [vec, theta]
    %% Your code ends here

end