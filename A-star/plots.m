robot1 = [100 100];
robot2 = [200 150];
obstacle1 = [15 60];
obstacle2 = [150 150];
obstcale3 = [250 300];
goal = [300 300];

robotR = 5;
obstacleR = 2;
goalR = 7;

[x y] = meshgrid(95:105,95:105);
z = sqrt(25-((x-100).^2)-((y-100).^2));
surf(x,y,z);
% hold on;
% for i=1:2
%     name = strcat('robot',num2str(i));
%     z = (x-name(1)).^2+(y-name(2)).^2-robotR.^2;
%     surf(x,y,z);
% end
% hold off;
% f1 = @(x,y) erf(x)+cos(y);
% fsurf(f1,[-5 0 -5 5])
% hold on
% f2 = @(x,y) sin(x)+cos(y);
% fsurf(f2,[0 5 -5 5])
% hold off