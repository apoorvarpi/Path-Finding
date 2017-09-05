function [x,y] = smallest( F, Open, Set)
%SMALLEST Summary of this function goes here
%   Detailed explanation goes here
min = Inf;
x=0;
y=0;
for i=1:size(Set,1)
    currx = Set(i,1);
    curry = Set(i,2);
    %disp 'checking';
    %disp ([i currx curry]);
    if Open(currx, curry)==1
        if min>F(currx, curry)
            x=currx;
            y=curry;
            min = F(currx,curry);
        end
    %else
        %disp 'not in set';
        %disp ([currx curry]);
    end
end
%disp 'returning'
%disp ([x y]);
end

