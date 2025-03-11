%% ----------------------´æ´¢¼ÆËãµÄ½á¹û-----------------------------
function result(vrp2e,name,opt_vrps,opt_vlts,opt_cuss,opt_sats,opt_caps,vrps_covs,vrtc_covs,cpu_time)
    %----ÅÐ¶ÏÊÇ·ñ´æÔÚvrp_dataÎÄ¼þ¼Ð£¬Ã»ÓÐ¾Í´´½¨
    judge     = exist('vrp_data');
    if judge ~= 7
        system('mkdir vrp_data');
    end
%     file_path = strcat(cd,'\vrp_data');
%     %----´æ´¢opt_vrps----
%     names   = strcat('fvrp_',name,'.txt');
%     save(names,'opt_vrps','-ascii');
%     movefile('fvrp*',file_path);
% 
% 
%     %----´æ´¢opt_vlts----
%     names   = strcat('fvlt_',name,'.txt');
%     save(names,'opt_vlts','-ascii');
% %     fid       = fopen(names,'w');
% %     fprintf(fid,'%6.5f\n',opt_vlts);
% %     fclose(fid);
%     movefile('fvlt*',file_path);
% 
% 
%     %----´æ´¢opt_cuss----
%     names   = strcat('cus_',name,'.txt');
%     save(names,'opt_cuss','-ascii');
% %     fid       = fopen(names,'w');
% %     fprintf(fid,'%d\n',opt_cuss);
% %     fclose(fid);
%     movefile('cus*',file_path);
% 
% 
%     %----´æ´¢opt_sats----
%     names   = strcat('sat_',name,'.txt');
%     save(names,'opt_sats','-ascii');
% %     fid       = fopen(names,'w');
% %     fprintf(fid,'%d\n',opt_sats);
% %     fclose(fid);
%     movefile('sat*',file_path);
% 
% 
%     %----´æ´¢opt_caps----
%     names   = strcat('cap_',name,'.txt');
%     save(names,'opt_caps','-ascii');
% %     fid       = fopen(names,'w');
% %     fprintf(fid,'%d\n',opt_caps);
% %     fclose(fid);
%     movefile('cap*',file_path);
% 
% 
%     %----´æ´¢cpu_time----
%     names   = strcat('cov_',name,'.txt');
%     save(names,'vrps_covs','-ascii');
%     movefile('cov*',file_path);
%     
%     %----´æ´¢cpu_time----
%     names   = strcat('time_',name,'.txt');
%     save(names,'cpu_time','-ascii');
%     movefile('time*',file_path);
    save(['vrp_data/',name,'.mat'],'vrp2e','opt_vrps','opt_vlts','opt_cuss','opt_sats','opt_caps','vrps_covs','vrtc_covs','cpu_time');
end