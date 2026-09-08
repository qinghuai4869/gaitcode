%四元数旋转法，输入theta是旋转角度（右手系），v是旋转轴矢量，u是原始点位置矢量，输出U是旋转后的点位置矢量
function U=fun_QuaternionRotate(theta,v,u)
    v=v./sqrt(sum(v.*v));%归一化
    q=[cos(theta/2),v(1)*sin(theta/2),v(2)*sin(theta/2),v(3)*sin(theta/2)];
    qt=quatconj(q);%取共轭
    w=[0,u];%纯四元数
    W=quatmultiply(quatmultiply(q,w),qt);%调用四元数乘法计算公式
    U=W(2:4);%取后三位
end
%%%%%%%%%%%%%%%%
% 版权声明：本文为CSDN博主「非 常 道」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
% 原文链接：https://blog.csdn.net/weixin_42845306/article/details/118221029

