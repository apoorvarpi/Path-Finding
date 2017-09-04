MAP = imread('padded.bmp');
[Height, Width] = size(MAP);
imshow(imread('map.bmp'));
%start position, end position
[x y] = ginput(2);
startx = x(1);
starty = y(1);
endx = x(2);
endy = y(2);
close all;
%F(Esimated), G(Travelled),H(Heuristic) matrices declaration
F = zeros(Height, Width);
G = F;
H = G;
%openset and closed set declaration
Close = [];
Set = [];
Open = F;
%setting parent
ParentX = F;
ParentY = F;
%initialising open, parents, heuristic, gvalues, fvalues
Open(startx, starty)=1;
Set = [Set, [startx, starty]];
ParentX(startx,starty)=-1;
ParentY(startx,starty)=-1;
inf = Height+Width+10;
for i=1:Height
    for j=1:Width
        H(i,j) = abs(i-endx)+abs(j-endy);
        G(i,j) = inf;
        F(i,j) = inf;
    end;
end;
G(startx, starty) = 0; 
F(startx, starty) = H(startx, starty);
count = 1;
%describing the eight neighbours
xs = [-1, -1, -1, 0, 1, 1, 1, 0];
ys = [-1, 0, 1, 1, 1, 0, -1, -1];
while count>0
    [currx, curry] = smallest(F, Open, Set);
     if currx==endx && curry==endy
         disp 'found'
         break;
     end;
     if currx==0&&curry==0
         break;
     end
     Open(currx, curry) = 0;
     count = count-1;
     Close = [Close, [currx, curry]];
     New = Open;
     for i=1:8
         nx = currx+xs(i);
         ny = curry+ys(i);
         if nx>0 && nx<=Height && ny>0 && ny<=Width && MAP(nx,ny)==1
            if ismember([nx, ny], Close)
                continue;
            end;
            if Open(nx,ny)==0
                Set = [Set, [nx,ny]];
                Open(nx,ny)=1;
                count = count+1;
              end;
             g = G(currx, curry) + 1;
             if g >= G(nx,ny)
                continue;
             end;
             ParentX(nx,ny) = currx;
             ParentY(nx,ny) = curry;
             G(nx,ny) = g;
             F(nx,ny) = G(nx,ny)+H(nx,ny);
         end;
     end; 
end;
