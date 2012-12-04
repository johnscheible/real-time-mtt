function bbs = filter_bbs(bbs, min_height)
bbs(:, bbs(4, :) - bbs(2, :) < min_height) = [];
end