%%%%%% acquire data from all people
%% IMU Txt
% simdata.numFiles = 62; % 多少个文件，改这里

% Path to the folder where the IMU data file of right foot that should be processed is
% located.
% 可以自己定义导入数据的方法，与文章核心内容无关


%% walkFreedom 以前的+2*6
pathSubjectWalkFreedomInit = {'XXX'};
pathSubjectWalkFreedom = {'XXX\MT_01200CE8_'};

SubjectWalkFreedomFullName(1,:) ={'XXX'};
SubjectWalkFreedomFullName(2,:) = {};% 试次总数
SubjectWalkFreedomFullName(3,:) = {};% 去掉的试次
SubjectWalkFreedomFullName(4,:) = {};% 起始

%% 固定步长行走
pathSubjectLinearWalkInit = {['XXX']};

pathSubjectLinearWalk = {'XXX\MT_01200CE8_'};

SubjectLinearWalkFullName(1,:) ={'XXX'};
SubjectLinearWalkFullName(2,:) = {};% 试次总数
SubjectLinearWalkFullName(3,:) = {};% 记录失误的数据
SubjectLinearWalkFullName(4,:) = {};% 数据记录的开始

% end



