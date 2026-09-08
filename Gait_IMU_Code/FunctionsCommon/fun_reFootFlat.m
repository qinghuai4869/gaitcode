function [rep_hr,rep_fc,re_rtk] = fun_reFootFlat(pos_hr,pos_fc,ra,rw,norm_acc,norm_gyr,rtk)
 rep_fc = pos_fc;
 rep_hr = pos_hr;
 re_rtk = rtk;
%  h0 = max(norm_acc) * 0.3;
 h0 = ra* 2;
 h = ra * 1.5;
 w0 = rw * 1.8;
%  ra1 = 1.23 * ra;
 pos_footFlat =[pos_fc(1:end-1)' pos_hr(2:end)'];
 T_footFlat =  pos_hr(2:end)' - pos_fc(1:end-1)';
 L = mean(T_footFlat) * 0.3;
 N = length(T_footFlat);
 for k = 1:N
     a = norm_acc(pos_footFlat(k,1):pos_footFlat(k,2));
     g = norm_gyr(pos_footFlat(k,1):pos_footFlat(k,2));
     M = length(a);
     b = zeros(1,M);
     c = 0; 
     Tnum = 0;
     if M < L 
         rep_fc(k) = rep_fc(k+1);
         re_rtk(rep_hr(k):rep_fc(k)) = 1; % 这才是目标
         rep_hr(k+1) = rep_hr(k);
%          [rep_fc,~] = unique(rep_fc,'rows','stable');
%          [rep_hr,~] = unique(rep_hr,'rows','stable');
%          N = N-1;
%          rep_fc(k) = rep_fc(k+1); 这就是个过度
        continue;
     end

     for m = 1:M
        if a(m) > ra || g(m) > rw
         Tnum = Tnum + 1;
         b(m) = 1;
         c = [c,m];
        end
     end

     if c == 0
         continue;
     end
        
     value_ = diff(c(2:end));
     L = length(value_);
     m = 1;
     n = 1;
     b = 0;
     valum_num0 = 0;

     for i = 1:L
         if value_(i)==1 && m==i
             m = m + 1;
             b = 1;
%              n = n + 1;
             valum_num0   = [valum_num0,i];
             value_num{n} = valum_num0;
         else 
             valum_num0 = 0;
%              m= m+1;
             if value_(i) == 1
                 m = i + 1;
                 n = n + b;
                 valum_num0   = [valum_num0,i];
%                  value_num0 = i;

                 value_num{n} = valum_num0;
                 
             end
         
         end
     end

     if n==1 && b==0 && m==1
         continue;
     end

%      [L_prow, L_pcol]= size(value_num);
     [~, L_pcol]= size(value_num);

%      if L_pcol == 0
%          continue;
%      end

     for j =1:L_pcol
         value_num_ = value_num{j};
         pos_c = c(value_num_(2:end)+1:value_num_(end)+2);
         a_ = a(pos_c);
         g_ = g(pos_c);
         PP_value_a = find(a_>h0);
         PP_value_w = find(g_>w0);
         PP_num = length(PP_value_a) + length(PP_value_w);
         sum_num = length(value_num_(1:end)); % 这里计数比实际少1个（2：end)
         d1 = pos_c(1);
         d2 = pos_c(end);

%      sum_num = sum(value_ == 1);
%      if value_ == 1
%          k = k+1;
%           
%      d0 = find(value_ == 1);
%      d1 = min(d0)+1;
%      d2 = max(d0)+2;
%      PP_value = find(a>h0);
% length(PP_value_a) >= 3
%      PP_num  合适吗？
         if (sum_num>11 && isempty(PP_value_a) ~= 1) || (PP_num >= 3 && isempty(PP_value_a) ~= 1)
%              sum_num>11 && isempty(PP_value) ~= 1
             if d1 < d2
                     re_fc = d2;
                     rep_fc(k) = rep_fc(k) + re_fc;
                     re_rtk(rep_hr(k):rep_fc(k)) = 1;
              else
                     re_hr = d1;
                     rep_hr(k+1) = rep_fc(k) + re_hr ;
                     re_rtk(rep_hr(k+1):rep_fc(k+1)) = 1;
             end
         end

     end

%%%%  如果平坦阶段遇到超过阈值连续，则包含这段，如果里面有峰值，则包含它。
%%%%%%  如果不连续，断断续续很多段，只要包含峰值的那段，如果多段包含峰值，都要，这咋办？ 

%      if diff(c(3:end)) == 1 
%          d1 = c(3) - 1;
%          d2 = M - c(end);  
%          d3 = find(a >= h);
%          if  Tnum > 11 || length(d3) > 5
%              if d1 < d2
%                  re_fc = c(end);
%                  rep_fc(k) = rep_fc(k) + re_fc;
%                  re_rtk(rep_hr(k):rep_fc(k)) = 1;
%              else
%                  re_hr = c(3);
%                  rep_hr(k+1) = rep_fc(k) + re_hr ;
%                  re_rtk(rep_hr(k+1):rep_fc(k+1)) = 1;
%              end
%               
%               
%          end
%      else
%           d4 = find(a >= h0);
%           d5 = min(d4) - 5;
%           d6 = max(d4) + 5;
%           if abs(d5-d6) >= 16 % 18 实际是6个长度
%               if d5 < M-d6
%                   re_fc = d6;
%                   rep_fc(k) = rep_fc(k) + re_fc;
%                   re_rtk(rep_hr(k):rep_fc(k)) = 1;
%               else
%                   re_hr = d5;
%                   rep_hr(k+1) = rep_hr(k+1) - re_hr + 1;
%                   re_rtk(rep_hr(k+1):rep_fc(k+1)) = 1;
%               end
%               
%               
%           end
%      end
   
 
 clear c
 clear value_num
 end
   [rep_fc1,~] = unique(rep_fc','rows','stable');
   [rep_hr1,~] = unique(rep_hr','rows','stable');
   rep_fc = rep_fc1';
   rep_hr = rep_hr1';
end

