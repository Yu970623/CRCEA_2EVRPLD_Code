%% -----------------种群更新---------------------------------------
function [seq_sat,cap_sat,seq_cus,vrp1,vrp2,vlt1,vlt2] = select_SR(seq_sats,cap_sats,seq_cuss,vrps1,vrps2,vlts1,vlts2)
num_p = size(seq_sats,1);
% [~,loc] = sort((vrps1+vrps2).*((vlts1+vlts2)*10000+1));
loc=sort_SR(vrps1,vrps2,vlts1,vlts2);

lst     = 1:ceil(0.5*num_p);
seq_sat = seq_sats(loc(lst),:);
cap_sat = cap_sats(loc(lst),:);
seq_cus = seq_cuss(loc(lst),:);
vrp1    = vrps1(loc(lst));
vrp2    = vrps2(loc(lst));
vlt1    = vlts1(loc(lst));
vlt2    = vlts2(loc(lst));
end

function loc=sort_SR(vrps1,vrps2,vlts1,vlts2)
Pf=0.65;
vrps=vrps1+vrps2;
vlts=vlts1+vlts2;
N=length(vrps);
loc=1:N;
    for i=1:N-1%执行N次
        for j=1:N-1
            if  ((vlts(j+1)==0)&&(vlts(j)==0))||(rand<Pf)%if both is feasible or pf<0.55, compare their objecitves
                if vrps(j)>vrps(j+1)
                    t=vrps(j);
                    vrps(j)=vrps(j+1);
                    vrps(j+1)=t;
                    
                    t=loc(j);%index following the sorting
                    loc(j)=loc(j+1);
                    loc(j+1)=t;
                end
            else% if exist infeasible sollution and pf>=0.55,  compare their constraints violations
                if vlts(j)>vlts(j+1)
                    t=vlts(j);
                    vlts(j)=vlts(j+1);
                    vlts(j+1)=t;
                    
                    t=loc(j);
                    loc(j)=loc(j+1);
                    loc(j+1)=t;
                end
            end
        end   
    end
% [~,loc2] = sort(vrps1+vrps2);
end
