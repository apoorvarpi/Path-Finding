function path( ParentX, ParentY, endx, endy )
%PATH Summary of this function goes here
%   Detailed explanation goes here
    if endx==0&&endy==0
        return;
    end;
    path(ParentX, ParentY, ParentX(endx, endy), ParentY(endx, endy));
    disp ([endx, endy]);
end

