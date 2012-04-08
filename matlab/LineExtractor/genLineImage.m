function im = genLineImage(lines, sz)

for i = 1:size(lines, 1)
    hold on
    line(lines(i, 1:2), lines(i, 3:4),'Color', 'r', 'linewidth', 2);
    hold off
end

return