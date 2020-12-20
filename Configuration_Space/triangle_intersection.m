function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise

%%% All of your code should be between the two lines of stars.
% *******************************************************************
A = P1(1,:); B = P1(2,:); C = P1(3,:);
E = P2(1,:); F = P2(2,:); G = P2(3,:);

flag = false;
% 1:- Line to Lie segment intersection Test

if(Line_To_Line_Intersection(A,B,E,F)) 
    flag = true;
    return;
elseif(Line_To_Line_Intersection(A,B,E,G)) 
    flag = true;
    return;
elseif(Line_To_Line_Intersection(A,B,F,G))
    flag = true;
    return;
elseif(Line_To_Line_Intersection(A,C,E,F))
    flag = true;
    return;
elseif(Line_To_Line_Intersection(A,C,E,G))
    flag = true;
    return;
elseif(Line_To_Line_Intersection(A,C,F,G))
    flag = true;
    return;
elseif(Line_To_Line_Intersection(B,C,E,F))
    flag = true;
    return;
elseif(Line_To_Line_Intersection(B,C,E,G))
    flag = true;
    return;
elseif(Line_To_Line_Intersection(B,C,F,G))
    flag = true;
    return;
end
    
% 2:- Test whether point is inside the triangle or not.
if (PointInTriangle(A,B,C,E))
    flag = true;
    return;
elseif(PointInTriangle(A,B,C,F))
    flag = true;
    return;
elseif(PointInTriangle(E,F,G,A))
    flag = true;
    return;
elseif(PointInTriangle(E,F,G,B))
    flag = true;
    return;
elseif(PointInTriangle(E,F,G,C))
    flag = true;
    return; 
end

% *******************************************************************
end

function flag = PointInTriangle(A,B,C,P)

% Compute vectors        
v0 = C - A;
v1 = B - A;
v2 = P - A;

% Compute dot products
dot00 = dot(v0, v0);
dot01 = dot(v0, v1);
dot02 = dot(v0, v2);
dot11 = dot(v1, v1);
dot12 = dot(v1, v2);

% Compute barycentric coordinates
invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
u = (dot11 * dot02 - dot01 * dot12) * invDenom;
v = (dot00 * dot12 - dot01 * dot02) * invDenom;

% Check if point is in triangle
flag = (u >= 0) && (v >= 0) && (u + v < 1);

end


function flag = Line_To_Line_Intersection(P1,P2,P3,P4)

% http://thirdpartyninjas.com/blog/2008/10/07/line-segment-intersection/
flag = false;
eps = 10^-6;

x1=P1(1,1); y1 = P1(1,2); x2 = P2(1,1); y2 = P2(1,2);
x3=P3(1,1); y3 = P3(1,2); x4 = P4(1,1); y4 = P4(1,2);

denom = (y4 - y3)*(x2-x1) - (y2-y1)*(x4-x3);

if(abs(denom) < eps) %parallel lines
    return;
end

numA = (x4-x3)*(y1-y3) - (y4-y3)*(x1-x3);
numB = (x2-x1)*(y1-y3) - (y2-y1)*(x1-x3);

if(abs(numA) < eps && abs(numB) < eps) %coincident lines
    flag = true;
    return;
end

Ua = numA/denom;
Ub = numB/denom;

if( Ua >= eps && Ua <= 1 && Ub >= 0 && Ub <=1)
    flag = true;
    return;
end

end
