function [x,y] = smallest( F, Open, Set)
%SMALLEST Summary of this function goes here
%   Detailed explanation goes here
min = size(F,1)+size(F,2)+5;
x=0;
y=0;
for i=1:size(Set)
    currx = Set(i,1);
    curry = Set(i,2);
    disp 'checking';
    disp ([currx curry]);
    if(Open(currx, curry)==1)
        if min>F(currx, curry)
            x=currx;
            y=curry;
        end
    else
        disp 'not in set';
        disp ([currx curry]);
    end
end
disp 'returning'
disp ([x y]);
end

