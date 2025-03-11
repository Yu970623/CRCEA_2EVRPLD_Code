%% --轮盘赌策略：基于权重val_w选取n个变量并返回对应序号seq_sel--
function seq_w = roulette(val_w,n)
    num_w = numel(val_w);
    val_w = val_w/(sum(val_w));
    seq_w = zeros(1,n); 
    r     = rand(1,n); 
    for i = 1:n     
        sum_w = 0;      
        j     = ceil(num_w*rand); 
        while sum_w < r(i)          
            sum_w = sum_w + val_w(mod(j-1,num_w)+1);         
            j     = j+1;     
        end   
        seq_w(i) = mod(j-2,num_w)+1; 
    end
end