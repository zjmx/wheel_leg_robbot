 % 本程序用于求解LQR反馈矩阵lqr_k(L0)
% 对于每一个腿长L0，求解一次系统状态空间方程，然后求得反馈矩阵K
% 对于不同的K，对L0进行拟合，得到lqr_k

clear;

L0s=0.125:0.01:0.439; % L0变化范围最小到最大
Ks=zeros(2,6,length(L0s)); % 存放不同L0对应的K

for step=1:length(L0s)
    
    % 所需符号量
    % bata1为左侧电机角度
    syms theta theta1 theta2 ; 
    syms x x1 x2;
    syms phi phi1 phi2; 
    syms bata1 bata2;
    syms T Tp N P Nm Pm Nf t;
    
    % 机器人结构参数(国际单位制)
    % l1小腿长度  l2大腿长度 X0关节电机中心距离 R驱动轮半径 l机体中心到机体旋转轴距离 mw驱动轮质量 mp腿质量 M机体质量 r机体电机质量到机体重心距离
    l1=0.16; l2=0.32;  x0=0.198; 
    R=0.113;  l=0.10; mw=0.5+1.13; mp=2*(0.214+0.035+0.358); M=10.031; r=sqrt((x0/2)^2+l^2);


    %eqs
    eqs=l1*sin(bata1)+l2*sin(bata2)-L0s(step)==0;
    eqs1=l1*cos(bata1)+l2*cos(bata2)+x0==0;
    [bata1,bata2]=solve(eqs,eqs1,bata1,bata2);
    bata1= max(bata1);
    bata2=min(bata2);


    %LM摆杆重心到机体转轴距离
    %                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
    Lm=0.5*(2*l1*l2*sin(bata1)+l1^2*sin(bata1)+l2^2*sin(bata2))/(l1+l2);
    %L摆杆重心到驱动轮轴心距离
    L=L0s(step)-Lm;


    %Iw轮子转动惯量
    Iw=0.5*mw*R*R; 
    %Ip腿转动惯量
    Ip=l1*mp*((Lm-0.5*l1*sin(bata1)^2)+(x0-0.5*l1*cos(bata1))^2)/(l1+l2)+l2*((L-0.5*l2*sin(bata2))^2+(l2*cos(bata2)))/(l1+l2);      
    %机体转动惯量
    Im=M*r*r;




    g=9.8;
    % 进行物理计算
    Nm=M*(x2+(L+Lm)*(theta2*cos(theta)-theta1^2*sin(theta))-l*(phi2*cos(phi)-phi1^2*sin(phi)));
    Pm=M*g+M*((L+Lm)*(-theta1^2*cos(theta)-theta2*sin(theta))-l*(phi1^2*cos(phi)+phi2*sin(phi)));
    N=Nm+mp*(x2+L*(theta2*cos(theta)-theta1^2*sin(theta)));
    P=Pm+mp*g+mp*L*(-theta1^2*cos(theta)-theta2*sin(theta));

    %将NM PM N P中间变量约掉
    equ1=x2-(T-N*R)/(Iw/R+mw*R);
    equ2=(P*L+Pm*Lm)*sin(theta)-(N*L+Nm*Lm)*cos(theta)-T+Tp-Ip*theta2;
    equ3=Tp+Nm*l*cos(phi)+Pm*l*sin(phi)-Im*phi2;
    [x2,theta2,phi2]=solve(equ1,equ2,equ3,x2,theta2,phi2);
    
    % 求得雅克比矩阵，然后得到状态空间方程
    Ja=jacobian([theta1;theta2;x1;x2;phi1;phi2],[theta theta1 x x1 phi phi1]);
    Jb=jacobian([theta1;theta2;x1;x2;phi1;phi2],[T Tp]);
%     vpa(Ja)
%     vpa(Jb)
    A=vpa(subs(Ja,[theta theta1 x x1  phi phi1],[0 0 0 0 0 0]));
    B=vpa(subs(Jb,[theta theta1 x x1  phi phi1],[0 0 0 0 0 0]));
%     vpa(A)
%     vpa(B)

    % 离散化
    [G,H]=c2d(eval(A),eval(B),0.005);
    
    % 定义权重矩阵Q, R
    Q=diag([4 0.1 300 50 400 0.1]);
    R=diag([1 1]);

    % 求解反馈矩阵K
    Ks(:,:,step)=dlqr(G,H,Q,R);

end

% 对K的每个元素关于L0进行拟合
K=sym('K',[2 6]);
syms L0;
for x=1:2
    for y=1:6
        p=polyfit(L0s,reshape(Ks(x,y,:),1,length(L0s)),3);
        K(x,y)=p(1)*L0^3+p(2)*L0^2+p(3)*L0+p(4);
        vpa(p(1))
        vpa(p(2))
        vpa(p(3))
        vpa(p(4))
    end
end

% 输出到m函数
matlabFunction(K,'File','LQR_K');

% 代入L0=0.2打印矩阵K，启动位置的初始K
%  vpa(subs(K,L0,0.2))
% 
% 
% disp("LQR模型函数生成完毕");