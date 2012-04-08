function angles = lineangles(lines, im)

for i = 1:size(lines, 1)
    % x1, x2, y1, y2
    dx = lines(i, 2) - lines(i, 1);
    dy = lines(i, 4) - lines(i, 3);
    angles(i) = atan2(dy, dx);
    if(angles(i) < -0.8 * pi)
        imshow(uint8(im)); genLineImage(lines(i, :), size(im));
        keyboard
    end
end

end