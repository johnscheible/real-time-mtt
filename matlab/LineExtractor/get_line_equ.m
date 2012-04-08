function equ = get_line_equ(lines)

equ = zeros(size(lines, 1), 3);
for i = 1:size(lines, 1)
    equ(i, :) = null([lines(i, 1), lines(i, 3), 1; lines(i, 2), lines(i, 4), 1])';
end


equ = equ ./ repmat(sqrt(sum(equ(:, 1:2).^2, 2)), 1, 3);