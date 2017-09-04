function [ANS] = path(ANS, ParentX, ParentY, nx, ny )
%PATH Summary of this function goes here
%   Detailed explanation goes here
    if nx==0&&ny==0
        return;
    end;
    ANS = path(ANS, ParentX, ParentY, ParentX(nx, ny), ParentY(nx, ny));
    disp ([nx, ny]);
    %set pixel value here
    ANS(ny:ny+1, nx:nx+1, 1) = 255;
    ANS(ny:ny+1, nx:nx+1, 2) = 0;
    ANS(ny:ny+1, nx:nx+1, 3) = 0;
end

