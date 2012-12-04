function draw_all_evals(evals, figid)
l = cell(1, length(evals));

cols = 'rgbkcymbrgbkcymb';
pattern = {'-' '-.' '--' '-' '-.' '--' '-' '-.' '--' '-' '-.' '--'};
figure(figid);
hold on;

for i = 1:length(evals)
    draw_fppimr(evals(i).recall, evals(i).fppi, cols(i), pattern{i});
    l{i} = evals(i).name;
end
hold off;
legend(l)
end