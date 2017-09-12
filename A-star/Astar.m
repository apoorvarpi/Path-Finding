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
%Adding object-cells to closed matrix
Close(MAP==0)=1;                  
Open = G;
%setting parent
ParentX = zeros(Height, Width);
ParentY = zeros(Height, Width);
%initialising open, parents, heuristic, gvalues, fvalues
Open(startx, starty)=1;
ParentX(startx,starty)=0;
ParentY(startx,starty)=0;
%inf = Height+Width+10;
for i=1:Height
    for j=1:Width
        H(i,j) = sqrt((i-endx)*(i-endx)+(j-endy)*(j-endy));
        G(i,j) = Inf;
        F(i,j) = Inf;
    end;
end;
G(startx, starty) = 0; 
F(startx, starty) = H(startx, starty);
count = 1;
%flag for found
flag=1;
%declaring the answer
ANS = imread('padded.jpg');
ANS(startx:startx+1, starty:starty+1, 1) = 0;
ANS(startx:startx+1, starty:starty+1, 2) = 255;
ANS(startx:startx+1, starty:starty+1, 3) = 0;

ANS(endx:endx+1, endy:endy+1, 1) = 0;
ANS(endx:endx+1, endy:endy+1, 2) = 255;
ANS(endx:endx+1, endy:endy+1, 3) = 0;

%loop begins
while count>0
     Vals = min(min(F));
     [X Y] = find(F==Vals);
     currx = X(1);
     curry = Y(1);
     if currx==endx && curry==endy
         flag=0;
         sol = [];
         [ANS, sol] = path(imread('colour.jpg'), sol, ParentX, ParentY, endx, endy);
         send(sol);
         disp 'FOUND';
         disp (G(endx,endy));
         break;
     end;
     %if currx==0&&curry==0
     %    break;
     %end
     ANS(currx, curry, 1) = 0;
     ANS(currx, curry, 2) = 0;
     ANS(currx, curry, 3) = 255;
     imshow(ANS);
     Open(currx, curry) = 0;
     F(currx, curry) = Inf;
     count = count-1;
     Close(currx, curry) = 1;
     New = Open;
     for i=-4:4
         for j=-4:4
            nx = currx+i;
            ny = curry+j;
            if ~(i==0&&j==0) && nx>0 && nx<=Height && ny>0 && ny<=Width 
                if (Close(nx,ny)==1)
                    continue;
                end;
                if Open(nx,ny)==0
                    Open(nx,ny)=1;
                    count = count+1;
                end;
                g = G(currx, curry) + sqrt(i*i+j*j);
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