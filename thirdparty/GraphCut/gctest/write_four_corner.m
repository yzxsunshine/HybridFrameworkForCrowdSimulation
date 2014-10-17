minx = -72.194504
minz = -82.920998
maxx = 87.805496
maxz = 77.079002
[X, Y] = meshgrid(0:2:20, 0:2:20);
X2 = X + (maxx-40)*ones(size(X, 1), size(X,2));
Y2 = Y + (minz+20)*ones(size(Y, 1), size(Y, 2));
X1 = X + (minx+20)*ones(size(X, 1), size(X, 2));
Y1 = Y + (minz+20)*ones(size(Y, 1), size(Y, 2));
X3 = X + (maxx-40)*ones(size(X, 1), size(X, 2));
Y3 = Y + (maxz-40)*ones(size(Y, 1), size(Y, 2));
X4 = X + (minx+20)*ones(size(X, 1), size(X, 2));
Y4 = Y + (maxz-40)*ones(size(Y, 1), size(Y, 2));

fid = fopen('four_corner.txt', 'w');
radius = (size(X1, 1) + size(X1, 2))/4;
centerI = size(X1, 1) / 2;
centerJ = size(X1, 2) / 2;
for i=1:size(X1, 1)
for j=1:size(X1, 2)
    if( (i-centerI)^2 + (j-centerJ)^2 <= radius*radius)
        fprintf(fid, '%f %f %f %f %f %f %d\n', X1(i, j), 10.10000, Y1(i, j), X3(i, j), 10.10000, Y3(i, j), 0);
    end
end
end

radius = (size(X2, 1) + size(X2, 2))/4;
centerI = size(X2, 1) / 2;
centerJ = size(X2, 2) / 2;
for i=1:size(X2, 1)
for j=1:size(X2, 2)
    if( (i-centerI)^2 + (j-centerJ)^2 <= radius*radius)
        fprintf(fid, '%f %f %f %f %f %f %d\n', X2(i, j), 10.10000, Y2(i, j), X4(i, j), 10.10000, Y4(i, j), 1);
    end
end
end

radius = (size(X3, 1) + size(X3, 2))/4;
centerI = size(X3, 1) / 2;
centerJ = size(X3, 2) / 2;
for i=1:size(X3, 1)
for j=1:size(X3, 2)
    if( (i-centerI)^2 + (j-centerJ)^2 <= radius*radius)
        fprintf(fid, '%f %f %f %f %f %f %d\n', X3(i, j), 10.10000, Y3(i, j), X1(i, j), 10.10000, Y1(i, j), 2);
    end
end
end

radius = (size(X4, 1) + size(X4, 2))/4;
centerI = size(X4, 1) / 2;
centerJ = size(X4, 2) / 2;
for i=1:size(X4, 1)
for j=1:size(X4, 2)
    if( (i-centerI)^2 + (j-centerJ)^2 <= radius*radius)
        fprintf(fid, '%f %f %f %f %f %f %d\n', X4(i, j), 10.10000, Y4(i, j), X2(i, j), 10.10000, Y2(i, j), 3);
    end
end
end

fclose(fid);