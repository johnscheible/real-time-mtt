function vote = hough_transform(lines, sz, gridsz)

vote = zeros(floor(sz(1) / gridsz(1)), floor(sz(2)/gridsz(2)));

for i = 1:size(lines, 2)
    for xgrid = 1:floor(sz(2)/gridsz(2))
        for ygrid = 1:floor(sz(1)/gridsz(1))
            if (is_overlap(rt, line))
            end
        end
    end
end

end


function map = get_votemap_oneline(line, sz, gridsz)
end

% (lines(i, 4) - lines(i, 3)) / (lines(i, 2) - lines(i, 1)) * (lines(i, 2) - lines(i, 1)) + lines(i, 3);