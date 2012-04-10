function vote = heuristic_vote(lines, sz, step)

vote = zeros(sz(1), sz(2));
sigma = 0.1;

equ = get_line_equ(lines);
for x = 1:step:sz(1)
    for y = 1:step:sz(2)
        pt = [y, x];
        
        for k = 1:size(lines, 1)
            alpha = get_alpha(pt, lines(k, 1:4), equ(k, :));
            assert(alpha >= 0 && alpha < pi / 2);
%             if(alpha > pi / 2)
%                 alpha = pi - alpha;
%             elseif(alpha < 0)
%                 alpha = -alpha;
%             elseif(alpha > pi)
%                 assert(0);
%             end
            l = sqrt((lines(k, 1) - lines(k, 2))^2 + (lines(k, 3) - lines(k, 4))^2);
            vote(x, y) = vote(x, y) + l * exp(-(alpha / (2 * sigma ^ 2)));
        end
    end
end

vote = vote(1:step:end, 1:step:end);

end


function alpha = get_alpha(pt, line, equ)

mpt = [(line(1) + line(2)) / 2, (line(3) + line(4)) / 2];
equ2 = null([pt, 1; mpt, 1])';
alpha = atan2(abs(equ(1) * equ2(2) - equ2(1) * equ(2)), abs(equ(1) * equ2(1) + equ(2) * equ2(2)));

end
