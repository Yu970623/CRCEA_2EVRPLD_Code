%% -----------------种群更新---------------------------------------
function [seq_sat,cap_sat,seq_cus,vrp1,vrp2,vlt1,vlt2] = select(seq_sats,cap_sats,seq_cuss,vrps1,vrps2,vlts1,vlts2,num_pop)
    num_p = size(seq_sats,1);
    [~,loc] = sort((vrps1+vrps2).*((vlts1+vlts2)*10000+1)); %f = f1+CV*1000;
%   lst         = 1:2:num_vrp;
%   lst         = 1:ceil(0.5*num_p);%select half
     lst         = 1:num_pop;
    seq_sat = seq_sats(loc(lst),:);
    cap_sat = cap_sats(loc(lst),:);
    seq_cus = seq_cuss(loc(lst),:);
    vrp1    = vrps1(loc(lst));
    vrp2    = vrps2(loc(lst));
    vlt1    = vlts1(loc(lst));
    vlt2    = vlts2(loc(lst));
end