function [ value ] = radius_check( A, x,y, radius )
%RADIUS_CHECK Summary of this function goes here
%   Detailed explanation goes here
value = true;
for i = -radius:radius
    for j = -radius:radius
        nx = x+i;
        ny = y+j;
        if nx>0 && nx<=size(A,1) && ny>0 && ny<=size(A,2) && A(nx,ny)==0
            value = false;
            break;
        end
    end
end

end

