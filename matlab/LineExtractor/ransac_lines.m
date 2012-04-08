function [max_inlier, max_pt] = ransac_lines(lines, eps)

% [x1 y1 1; x2 y2 1] * [a;b;c] = [0 0 0]
equ = get_line_equ(lines);

% for i = 1:size(lines, 1)
%     equ(i, :) = null([lines(i, 1), lines(i, 3), 1; lines(i, 2), lines(i, 4), 1])';
%     
%     
%     line(lines(i, 1:2), lines(i, 3:4),'Color', 'r', 'linewidth', 2);
%     tempx = 1:720;
%     for j = 1:720
%         tempy(j) = (-equ(i, 3) - (tempx(j) * equ(i, 1))) / equ(i, 2);
%     end
%     hold on
%     plot(tempx, tempy);
%     hold off 
%     tic;
% end
% 
% axis([0 720 0 480])

max_inlier = zeros(size(equ, 1), 1);
max_pt = [0, 0];

niter = 50000;
for i = 1:niter
    idx = ceil(rand(2, 1) * size(equ, 1));
    
    pt = null(equ(idx, :)); pt = pt ./ pt(3);
    res = equ * pt;
    
    if sum(res.^2 < eps) > sum(max_inlier)
        max_pt = pt(1:2);
        max_inlier = (res.^2 < eps);
    end
%     
%     figure(1);
%     clf;
%     hold on;
%     for j = 1:2
%         drawLine(equ(idx(j), :), [640 480]);
%     end
%     hold off;
%     
%     hold on;
%     scatter(pt(1)/pt(3), pt(2)/pt(3));
%     hold off;
%     
%     tic;
    
end
end


function drawLine(equ, sz)

x = 1:sz(1);

for j = 1:sz(1)
    y(j) = (-equ(3) - (x(j) * equ(1))) / equ(2);
end

plot(x, y);

axis([1 sz(1) 1 sz(2)]);

end