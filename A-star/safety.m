function [ J ] = safety( I,x )
%LEFTOVERS Summary of this function goes here
%Left and top of the image padded with x
%   Detailed explanation goes here
J = I;
for i=1:size(I,1)
    for j=1:size(I,2)
        if I(i,j) == 0
            for k=max(1,i-floor(x/2)):min(size(I,1),i+floor(x/2))
                for l=max(1,j-floor(x/2)):min(size(I,2), j+floor(x/2))
                    J(k,l)=0;
                end;
            end;
        end;
    end;
end

