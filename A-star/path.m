function [ANS, sol] = path(ANS, sol, ParentX, ParentY, nx, ny )
%PATH Summary of this function goes here
%   Detailed explanation goes here
    if nx==0&&ny==0
        return;
    end;
    [ANS sol] = path(ANS, sol, ParentX, ParentY, ParentX(nx, ny), ParentY(nx, ny));
    %display
    disp ([nx, ny]);
    %save in a matrix
    sol = [sol; [nx,ny]];
    %set pixel value here
    ANS(nx:nx+1, ny:ny+1, 1) = 255;
    ANS(nx:nx+1, ny:ny+1, 2) = 0;
    ANS(nx:nx+1, ny:ny+1, 3) = 0;
end

