function [rep_hr,rep_fc,re_rtk] = fun_FootPP(pos_hr,pos_fc,ra,rw,norm_acc,norm_gyr,rtk)

 rep_fc = pos_fc;
 rep_hr = pos_hr;
 re_rtk = rtk;
 h = ra * 2; 
% h= 2.8;
 pos_footPP =[pos_hr(1:end)' pos_fc(1:end)'];
 T_footPP = pos_fc(1:end)' - pos_hr(1:end)';
 N = length(T_footPP);

 L = mean(T_footPP) * 1.5;
 for k = 1:N
     a = norm_acc(pos_footPP(k,1):pos_footPP(k,2));
     g = norm_gyr(pos_footPP(k,1):pos_footPP(k,2));
     pp_acc = max(a);
     if pp_acc < h
         re_rtk(pos_hr(k):pos_fc(k)) = 0;
         if k==1
             rep_fc(k) = rep_fc(k+1);
             rep_hr(k) = rep_hr(k+1);
         else
             rep_fc(k) = rep_fc(k-1);
             rep_hr(k) = rep_hr(k-1);
         end

     end

     
     M = length(a);
     h_a = 0.23;
     h_w = 0.23;
     acc_h = 1.5 * ra; 
     agr_h = 1.5 * rw;

     if M > L           
            ra_finial= fun_accelerationBasedRestSignal(a,h_a,M,acc_h);
            rw_finial= fun_accelerationBasedRestSignal(g,h_w,M,agr_h);
             %% 完全落地到 full contact 融合
            rtk_combined = ra_finial + rw_finial;
            rtk_pos = rtk_combined>1;
            rtk_combined(rtk_pos)=1; 
            re_rtk(pos_hr(k):pos_fc(k)) = fun_peliminatePhases(rtk_combined);
            %%%% time : heel rise and full contact
            re_rtk = fun_peliminatePhases(re_rtk);
            rep_hr =find(diff(re_rtk)==1);
            rep_fc = find(diff(re_rtk)==-1);  %%% 这个是1还是0
     end

 end
%      [rep_fc,~] = unique(rep_fc,'rows','stable');
%      [rep_hr,~] = unique(rep_hr,'rows','stable');
   [rep_fc1,~] = unique(rep_fc','rows','stable');
   [rep_hr1,~] = unique(rep_hr','rows','stable');
   rep_fc = rep_fc1';
   rep_hr = rep_hr1';
end