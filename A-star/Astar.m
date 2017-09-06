MAP = imread('padded.bmp');
[Height, Width] = size(MAP);
imshow(imread('map.bmp'));
%start position, end position
[x y] = ginput(2);
startx = y(1);
starty = x(1);
endx = y(2);
endy = x(2);
close all;
%F(Esimated), G(Travelled),H(Heuristic) matrices declaration
G = zeros(Height, Width);
H = single(zeros(Height, Width));
F = single(inf(Height,Width));
%openset and closed set declaration
Close = G;
Set = [];
Close(MAP==0)=1;                  %Adding object-cells to closed matrix
Open = G;
%setting parent
ParentX = zeros(Height, Width);
ParentY = zeros(Height, Width);

%initialising open, parents, heuristic, gvalues, fvalues
Open(startx, starty)=1;
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
ANS(startx:startx+1, starty:starty+1, 1) = 0;
ANS(startx:startx+1, starty:starty+1, 2) = 255;
ANS(startx:startx+1, starty:starty+1, 3) = 0;

ANS(endx:endx+1, endy:endy+1, 1) = 0;
ANS(endx:endx+1, endy:endy+1, 2) = 255;
ANS(endx:endx+1, endy:endy+1, 3) = 0;

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
     Close(currx, curry) = 1;
     New = Open;
     for i=1:8
         nx = currx+xs(i);
         ny = curry+ys(i);
         if nx>0 && nx<=Height && ny>0 && ny<=Width 
            if (Close(nx,ny)==1)
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
             ANS(nx, ny, 1) = 0;
             ANS(nx, ny, 2) = 0;
             ANS(nx, ny, 3) = 255;
             imshow(ANS);
         end;
     end; 
end;
if flag==1
    disp 'NOT FOUND';
    
    ANS(startx-2:startx+2, starty-2:starty+2, 1) = 0;
    ANS(startx-2:startx+2, starty-2:starty+2, 2) = 255;
    ANS(startx-2:startx+2, starty-2:starty+2, 3) = 0;

    ANS(endx-2:endx+2, endy-2:endy+2, 1) = 0;
    ANS(endx-2:endx+2, endy-2:endy+2, 2) = 255;
    ANS(endx-2:endx+2, endy-2:endy+2, 3) = 0;
end;
imshow(ANS);
title('Final');