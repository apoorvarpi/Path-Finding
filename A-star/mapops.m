im = imread('map.bmp');
imwrite(im, 'colour.jpg');
im = im2bw(rgb2gray(im));
im = imcomplement(im);
im1 = imcomplement(im);
x = 18; % choose appropriate value
im1 = safety(im1,x);
imwrite(im1,'padded.bmp');
imwrite(im1, 'padded.jpg');
im1 = uint8(im1);
im1 = 255.*im1;
A = imread('padded.jpg');
for i=1:size(im,1)
    for j=1:size(im,2)
        if im(i,j)==1
            A(i,j,1)=0;
            A(i,j,2)=255;
            A(i,j,3)=0;
        end;
    end;
end;
imshow(A);