runs  = length(opt_cuss);
results = zeros(runs,1);

for i = 1:runs
    opt_cus = opt_cuss{i,1};
    for s = 1:vrp2e.num_sat
        results(i) = results(i) + max((sum(opt_cus==s)-vrp2e.maxFleetPerSat),0);
    end
end
results