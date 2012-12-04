function ret = findLAMR(mr, fppi, N, range)

if nargin < 3
	N = 1000;
	range = [-1.6 0];
elseif nargin < 4
	range = [-1.6 0];
end

ret = 0;
samples = logspace(range(1), range(2), N);

for i = 1:length(samples)
	idx = findFPPI(fppi, samples(i));
	ret = ret + mr(idx) / N;
end

end

function idx = findFPPI(fppi, val)

tid = find(fppi < val);
[dummy, idx] = max(fppi(tid));
idx = tid(idx);

end

