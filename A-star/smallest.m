function [x,y] = smallest( F, Open, Set)
%SMALLEST Summary of this function goes here
%   Detailed explanation goes here
min = size(F,1)+size(F,2)+5;
x=-1;
y=-1;
for i=1:size(Set)
    currx = Set(i,1);
    curry = Set(i,2);
    if(Open(currx, curry)==1)
        if min>F(currx, curry)
            x=currx;
            y=curry;
        end
    end
end
end

