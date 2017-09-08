function send(A)
%SEND - save in a file named path.txt
%   Detailed explanation goes here
    fileID = fopen('path.txt','w');
    N = size(imread('map.bmp'),1);
    for i=1:size(A,1)
        X = A(i,2);
        Y = N - A(i,1);
        fprintf(fileID,'%d %d\n', X, Y);
    end;
    fclose(fileID);
end

