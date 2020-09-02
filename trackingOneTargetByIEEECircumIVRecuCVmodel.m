
%%%Localization and Circumnavigation of a Slowly Moving Target Using Bearing Measurements
%%%仅按照改文章进行仿真
clear all;
close all;
clc;
targetRealPos(:,1)=[2;4];
targetRealVel(:,1)=[0.1;0];

targetEstPos(:,1)=[4;3];
targetEstVel(:,1)=[0;0];

obsPos(:,1)=[15;15];
obsVel(:,1)=[0;0];
constantVel=5;
trackingR=2;


sampleT=0.1;
% axis([-10 10 -10 10]) ;
grid on;
kEst=4;

clv=0.1*pi/180;
T_O_del=[0;0];%targetRealPos(:,1)-obsPos(:,1);
T_O_RealDist(1,1)=sqrt(T_O_del(1)^2+T_O_del(2)^2);%%目标—观测器的真实斜距离
obsFai(1,1)=atan2(T_O_del(2),T_O_del(1))+0.1*pi/180*randn(1);%%仿真观测角度
uniObsVec=[cos(obsFai(1,1));sin(obsFai(1,1))];%%单位观测向量
targetEstPos(:,1)=[4;3];%targetRealPos(:,1);%%%位置估计
T_O_delEst=targetEstPos(:,1)-obsPos(:,1);%%%估计出的目标—观测器向量
T_O_DisEst(1,1)=sqrt(T_O_delEst(1)^2+T_O_delEst(2)^2);%%估计出的目标—观测器斜距离
targetEstVel(:,1)=kEst*(eye(2)-uniObsVec*uniObsVec')*(-T_O_delEst);%%%当前估计出的目标速度估计
% targetEstVel(:,1)=[0;0];
rotateAng=90*pi/180;%acos(sampleT*constantVel/2/T_O_DisEst(1,1));
rotateM=[cos(rotateAng) sin(rotateAng);-sin(rotateAng) cos(rotateAng)];
uniObsVecBar=rotateM*uniObsVec;%%观测器切向运动向量
obsVel(:,1)=constantVel*uniObsVecBar;%%%当前观测器运动速度

    targetRealVel(1,1)=1;%%目标当真实速度
    targetRealVel(2,1)=1+10*0.1;
    T_O_delEst=targetEstPos(:,1)-obsPos(:,1);%%%估计出的目标—观测器向量
%     remDeltaEstPt(:,1)=T_O_delEst;
%%观测器的运动方向

EstX_IV(:,1)=[30 0 30 0]';
EstP_IV=100*eye(4);
fai1=[1 sampleT; 0 1];
fai=[fai1 zeros(2,2);zeros(2,2) fai1];
B1=[sampleT^2/2;sampleT];
B=[B1 zeros(2,1);zeros(2,1) B1];
Mm=[1 0 0 0 ; 0 0 1 0];
q=0.2;
theta=pi/2;
for count=2:300
    
%     targetRealPos(:,count)=targetRealPos(:,count-1)+sampleT*targetRealVel(:,count-1);%%目标当前真实位置
%     targetRealVel(:,count)=targetRealVel(:,count-1);%%目标当真实速度

    targetRealPos(1,count)=targetRealPos(1,1)+1*sampleT*count+0*sin(0.1*sampleT*count);%%目标当前真实位置
    targetRealPos(2,count)=targetRealPos(2,1)+1*sampleT*count+10*sin(0.1*sampleT*count);%%目标当前真实位置
    targetRealVel(1,count)=1;%%目标当真实速度
    targetRealVel(2,count)=1+10*0.1*cos(0.1*sampleT*count);%%目标当真实速度
    
%     CWVx=2*cos(theta);
%     CWVy=2*sin(theta);
%     
%     targetRealVel(1,count)=CWVx;%%目标当真实速度
%     targetRealVel(2,count)=CWVy;%%目标当真实速度
%     targetRealPos(1,count)=targetRealPos(1,count-1)+CWVx*sampleT;%+1*sampleT*count+0*sin(0.1*sampleT*count);%%目标当前真实位置
%     targetRealPos(2,count)=targetRealPos(2,count-1)+CWVy*sampleT;%+1*sampleT*count+10*sin(0.1*sampleT*count);%%目标当前真实位置
%     theta=theta+3*pi/180*sampleT;
    
    
    obsPos(:,count)=obsPos(:,count-1)+sampleT*obsVel(:,count-1);%%观测器当前位置
    targetEstPos(:,count)=targetEstPos(:,count-1)+sampleT*targetEstVel(:,count-1);%%%估计出的目标当前位置
    if count>10
        yuceState=fai*EstX_IV(:,count-1);
        targetEstPos(:,count)=[yuceState(1,1),;yuceState(3,1)];
    end
    T_O_del=targetRealPos(:,count)-obsPos(:,count);
    T_O_RealDist(count,1)=sqrt(T_O_del(1)^2+T_O_del(2)^2);%%目标—观测器的真实斜距离
    obsFai(count,1)=atan2(T_O_del(2),T_O_del(1))+clv*randn(1);%%仿真观测角度    
    uniObsVec=[cos(obsFai(count,1));sin(obsFai(count,1))];%%单位观测向量
    T_O_delEst=targetEstPos(:,count)-obsPos(:,count);%%%估计出的目标—观测器向量
%     remDeltaEstPt(:,count)=T_O_delEst;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    targetEstVel(:,count)=kEst*(eye(2)-uniObsVec*uniObsVec')*(-T_O_delEst);%%%当前估计出的目标速度估计
    
    T_O_DisEst(count,1)=sqrt(T_O_delEst(1)^2+T_O_delEst(2)^2);%%估计出的目标—观测器斜距离
    checkDisErr=T_O_DisEst(count,1)-trackingR;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    rotateAng=acos(sampleT*constantVel/2/T_O_DisEst(count,1));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     rotateAng=90*pi/180;
%     rotateM=[cos(rotateAng) sin(rotateAng);-sin(rotateAng) cos(rotateAng)];
%     uniObsVecBar=rotateM*uniObsVec;%%观测器切向运动单位向量
%     if checkDisErr>10
%         checkDisErr=10;
%     end
%     obsVel(:,count)=checkDisErr*uniObsVec+5*uniObsVecBar;%%%当前观测器运动速度
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     if checkDisErr>0.1&&checkDisErr<=0.5
        rotateAng=80*pi/180;%atan2(4,sqrt(25-16));
    elseif   checkDisErr>0.1 
        rotateAng=30*pi/180;%atan2(4,sqrt(25-16));
    elseif checkDisErr<-0.1&&checkDisErr>=-0.5
        rotateAng=pi-80*pi/180;%atan2(4,sqrt(25-16));
    elseif checkDisErr<-0.5
        rotateAng=pi-30*pi/180;%atan2(4,sqrt(25-16));
    else
%         rotateAng=atan2(sqrt(constantVel^2-checkDisErr^2),checkDisErr);
        if checkDisErr<0
            rotateAng=90*pi/180;
        else
%             curV=sqrt(obsVel(1,count-1)^2+obsVel(1,count-1)^2);
%             rotateAng=acos(sampleT*curV/2/T_O_DisEst(count,1));
            rotateAng=acos(sampleT*constantVel/2/T_O_DisEst(count,1));
        end
    end
    rotateM=[cos(rotateAng) sin(rotateAng);-sin(rotateAng) cos(rotateAng)];
    uniObsVecBar=rotateM*uniObsVec;%%观测器切向运动向量
%     uniObsVecBar=[sin(obsFai(count,1));-cos(obsFai(count,1))];
    obsVel(:,count)=constantVel*uniObsVecBar;%%%当前观测器运动速度
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if count==2
        EstX_IV(:,count-1)=[targetEstPos(1,count) targetEstVel(1,count) targetEstPos(2,count) targetEstVel(2,count)]';
    end
    %%%%%predict state
    yuceX=fai*EstX_IV(:,count-1);
    %%%%%Covariance matrix of predicted state
    yuceP=fai*EstP_IV*fai'+B*q^2*B';
    %%%Kalman gain matrix:
    uk=[sin(obsFai(count,1));-cos(obsFai(count,1))];
    Hk=uk'*Mm;
    miu=clv;
    disEstVec=Mm*yuceX-obsPos(:,count);
    disEstScale=sqrt(disEstVec(1,1)^2+disEstVec(2,1)^2);
    clfc=disEstScale^2*miu^2;
    Kk=yuceP*Hk'*inv(clfc+Hk*yuceP*Hk');
    %%%Updated state
    zk=uk'*obsPos(:,count);
    X_kal=yuceX+Kk*(zk-Hk*yuceX);
%     EstX_IV(:,i)=X_kal;
    %%%%Covariance matrix of updated state
    P_kal=yuceP-Kk*Hk*yuceP;
    EstP_IV=P_kal;
    %%%%%Bias compensation
    X_bc=X_kal+P_kal*1/clfc*miu^2*Mm'*(Mm*X_kal-obsPos(:,count));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    EstX_IV(:,count)=X_bc;
    %%%%%%%%%
%     if count>70
        posE_bc=Mm*X_bc;
        theta_BC=atan2(posE_bc(2,1)-obsPos(2,count),posE_bc(1,1)-obsPos(1,count));
%         (theta_BC-RelaThetaMeas(i))*180/pi
        %%IV estimation
        Gk=[sin(theta_BC) -cos(theta_BC)]*Mm;
%         disEstVec=[posE_bc(1,1);posE_bc(2,1)]-obsPos(:,i);
%         disEstScale=sqrt(disEstVec(1,1)^2+disEstVec(2,1)^2);
%         clfc=disEstScale^2*miu^2;
        Kk_IV=yuceP*Gk'*inv(clfc+Hk*yuceP*Gk');
        EstX_IV(:,count)=yuceX+Kk_IV*(zk-Hk*yuceX);
        EstP_IV=yuceP-Kk_IV*Hk*yuceP;
%     end

    if count>10
        obsVel(:,count)=obsVel(:,count)+[EstX_IV(2,count);EstX_IV(4,count)];
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    line(EstX_IV(1,count),EstX_IV(3,count),'color','k','marker','>','markersize',5);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%绘制目标点、观测点、观测线
    line(targetRealPos(1,count),targetRealPos(2,count),'color','r','marker','o','markersize',5);
    line(targetEstPos(1,count),targetEstPos(2,count),'color','b','marker','>','markersize',5);
    line([obsPos(1,count-1),obsPos(1,count)],[obsPos(2,count-1),obsPos(2,count)],'color','c','marker','<','markersize',5);
    
    pause(0.01); 
    
end
PosEstErr=targetRealPos-[EstX_IV(1,:);EstX_IV(3,:)];
figure(2);
plot((1:count)*sampleT,PosEstErr(1,:),'r.-',(1:count)*sampleT,PosEstErr(2,:),'b.-');
title('x、y通道位置估计误差');
xlabel('t/s');
ylabel('Estimation error/m');
legend('e_x','e_y');
grid on;
% 
% figure(3);
% plot((1:count-LSN+1)*sampleT,VelEstErr(1,:),'r.-',(1:count-LSN+1)*sampleT,VelEstErr(2,:),'b.-');
% title('x、y通道速度估计误差');
% xlabel('t/s');
% ylabel('Estimation error/m/s');
% legend('e_vx','e_vy');
% grid on;

figure(3);
plot(1:count,T_O_RealDist(:,1),'r.-');
title('观测与目标真实位置间距离');
grid on;
% 
% checkRealDistErr=T_O_RealDist(:,1)-trackingR;
% figure(4);
% plot((1:count)*sampleT,T_O_RealDist(:,1),'b.-',(1:count)*sampleT,checkRealDistErr(:,1),'r.-');
% title('距离控制误差');
% xlabel('t/s');
% ylabel('Estimation error/m');
% legend('D(t)','D(t)-D_d');
% grid on;
% 
% figure(5);
% plot(1:count,targetEstVel(1,:),'b.-',1:count,targetEstVel(2,:),'r.-');
% title('速度估计值');
% grid on;
% 
% figure(6);
% plot((1:count)*sampleT,VelEstErr(1,:),'r.-',(1:count)*sampleT,VelEstErr(2,:),'b.-');
% title('Velocity estimaiton error of x,y channel');
% xlabel('t/s');
% ylabel('Velocity estimation error(m/s)');
% legend('e_vx','e_vy');
% grid on;
figure(1);
plot(targetRealPos(1,:),targetRealPos(2,:),'c-',obsPos(1,:),obsPos(2,:),'r-');
grid on;
% hold on;