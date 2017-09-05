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
Open(starty, starty)=1;
Set = [Set; [startx, starty]];
ParentX(startx,starty)=0;
ParentY(startx,starty)=0;
%inf = Height+Width+10;
for i=1:Height
    for j=1:Width
        H(i,j) = abs(i-endx)+abs(j-endy);
        G(i,j) = Inf;
        F(i,j) = Inf;
    end;
end;
G(startx, starty) = 0; 
F(startx, starty) = H(startx, starty);
count = 1;
%describing the eight neighbours
xs = [-1, -1, -1, 0, 1, 1, 1, 0];
ys = [-1, 0, 1, 1, 1, 0, -1, -1];
flag=1;
%declaring the answer
ANS = imread('colour.jpg');
ANS(starty:starty+1, startx:startx+1, 1) = 0;
ANS(starty:starty+1, startx:startx+1, 2) = 255;
ANS(starty:starty+1, startx:startx+1, 3) = 0;

ANS(endy:endy+1, endx:endx+1, 1) = 0;
ANS(endy:endy+1, endx:endx+1, 2) = 255;
ANS(endy:endy+1, endx:endx+1, 3) = 0;

%loop begins
while count>0
    [currx, curry] = smallest(F, Open, Set);
     if currx==endx && curry==endy
         flag=0;
         ANS = path(imread('colour.jpg'), ParentX, ParentY, endx, endy);
         disp 'FOUND';
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
         if nx>0 && nx<=Height && ny>0 && ny<=Width && MAP(ny,nx)==1
            if ismember([nx, ny], Close)
                continue;
            end;
            if Open(nx,ny)==0
                Set = [Set; [nx,ny]];
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
             ANS(ny, nx, 1) = 0;
             ANS(ny, nx, 2) = 0;
             ANS(ny, nx, 3) = 255;
             imshow(ANS);
         end;
     end; 
end;
if flag==1
    disp 'NOT FOUND';
    
    ANS(starty-2:starty+2, startx-2:startx+2, 1) = 0;
    ANS(starty-2:starty+2, startx-2:startx+2, 2) = 255;
    ANS(starty-2:starty+2, startx-2:startx+2, 3) = 0;

    ANS(endy-2:endy+2, endx-2:endx+2, 1) = 0;
    ANS(endy-2:endy+2, endx-2:endx+2, 2) = 255;
    ANS(endy-2:endy+2, endx-2:endx+2, 3) = 0;
end;
imshow(ANS);
title('Final');