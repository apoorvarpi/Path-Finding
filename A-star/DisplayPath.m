I = imread('colour.jpg');
N = size(I,1);
colorsR = [255 0 0 153];
colorsG = [0 255 0 0];
colorsB = [0 0 255 153];
while true
    for i=1:4
        fid=fopen(strcat('Position',num2str(i),'.txt'));
        s = fscanf(fid,'%d %d');
        fclose(fid);
        x1 = s(1);
        y1 = s(2);
        x = round(y1/50);
        y = round(N-(x1/50));
        I(x-2:x+2,y-2:y+2,1) = colorsR(i);
        I(x-2:x+2,y-2:y+2,2) = colorsG(i);
        I(x-2:x+2,y-2:y+2,3) = colorsB(i);
    end
    imshow(I);
end