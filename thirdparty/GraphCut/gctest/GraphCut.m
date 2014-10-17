function [ VX, VY ] = GraphCut( X, Y, DX, DY, labelNum, alpha)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
rows = size(X, 1);
cols = size(X, 2);
siteNum = rows * cols;
%labelNum = 60;  % every 6 degree is a label
labels = zeros(labelNum, 2);
deltaAngle = 2*pi/labelNum;
for i=1:labelNum
    labels(i, 1) = sin((i-1)*deltaAngle);
    labels(i, 2) = cos((i-1)*deltaAngle);
end

h = GCO_Create(siteNum, labelNum); 
dataCost = zeros(siteNum, labelNum);
smoothCost = zeros(labelNum, labelNum);
neighbors = zeros(siteNum, siteNum);

for i=1:rows
    for j=1:cols
        cid = (i-1)*cols + j;
        if i > 1
            nid = (i-2)*cols + j;
            neighbors(cid ,nid) = 1;
            neighbors(nid ,cid) = 1;
        end
        if i < rows
            nid = (i)*cols + j;
            neighbors(cid ,nid) = 1;
            neighbors(nid ,cid) = 1;
        end
        if j > 1
            nid = (i-1)*cols + j-1;
            neighbors(cid ,nid) = 1;
            neighbors(nid ,cid) = 1;
        end
        if j < cols
            nid = (i-1)*cols + j + 1;
            neighbors(cid ,nid) = 1;
            neighbors(nid ,cid) = 1;
        end
    end
end

invalidSite = zeros(siteNum, 1);
EPS = 0.000001;
for i=1:siteNum
    r = ceil(i / cols);
    c = mod(i-1, cols) + 1;
    vec = [DX(r, c), DY(r, c)];
    if(abs(vec(1)) < EPS && abs(vec(2)) < EPS)
        invalidSite(i) = 1;
    end
    for j=1:labelNum 
        if(abs(vec(1)) < EPS && abs(vec(2)) < EPS)
            dataCost(i, j) = 0;
            %{
            if r > 1
                nid = (r-2)*cols + c;
                neighbors(i ,nid) = 0;
                neighbors(nid, i) = 0;
            end
            if r < rows
                nid = (r)*cols + c;
                neighbors(i ,nid) = 0;
                neighbors(nid, i) = 0;
            end
            if c > 1
                nid = (r-1)*cols + c-1;
                neighbors(i ,nid) = 0;
                neighbors(nid ,i) = 0;
            end
            if c < cols
                nid = (r-1)*cols + c + 1;
                neighbors(i ,nid) = 0;
                neighbors(nid ,i) = 0;
            end
            %}
        else 
            vec = vec / norm(vec);
            dataCost(i, j) = (1 - dot(vec, labels(j, :))) / 2 * alpha;
        end
    end
end

for i=1:labelNum
    for j=1:labelNum
        smoothCost(i, j) = min(abs(i - j), labelNum - abs(i - j));%(1 - dot(labels(i, :), labels(j, :)))/2;
    end
end

GCO_SetDataCost(h, dataCost');
GCO_SetSmoothCost(h, smoothCost);
GCO_SetNeighbors(h, neighbors);
GCO_Expansion(h); 
minELabel = GCO_GetLabeling(h);
VX = zeros(rows, cols);
VY = zeros(rows, cols);
for i=1:siteNum
    r = ceil(i / cols);
    c = mod(i-1, cols) + 1;     
    if invalidSite(i) > 0
        VX(r, c) = 0;
        VY(r, c) = 0;
        continue;
    end
    normVal = norm([DX(r, c), DY(r, c)]);
    VX(r, c) = labels(minELabel(i), 1)*normVal;
    VY(r, c) = labels(minELabel(i), 2)*normVal;
end
GCO_Delete(h);

figure;
hold on
quiver(X,Y,DX,DY, 0.4)
colormap hsv
hold off
grid on;
axis([-1.5 -0.1 -0.7 0.7]);
set(gca, 'xtick', -1.5:0.2:-0.1);
set(gca, 'ytick', -0.7:0.2:0.7);
axis equal;

figure;
hold on
quiver(X,Y,VX,VY, 0.4)
colormap hsv
hold off
grid on;
axis([-1.5 -0.1 -0.7 0.7]);
set(gca, 'xtick', -1.5:0.2:-0.1);
axis equal;

set(gca, 'ytick', -0.7:0.2:0.7);

end

