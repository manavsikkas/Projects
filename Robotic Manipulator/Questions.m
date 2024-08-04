%% Q6 Kinematic Model 
robot.DH.theta= '[q(1)+pi/2 pi/2 pi/2 pi q(5)+pi/2 q(6)]';
robot.DH.d='[q(2) 0 q(3) q(4) 0 0.100]';
robot.DH.a='[0 0 0 0.200 0 0]';
robot.DH.alpha= '[0 pi/2 pi/2 pi/2 pi/2 0]';
q=[pi/2 1.2 2.5 0 0 0]
Theta=eval(robot.DH.theta);
d=eval(robot.DH.d);
a=eval(robot.DH.a);
alpha=eval(robot.DH.alpha);
 T01=dh(Theta(1),d(1),a(1),alpha(1));
 T12=dh(Theta(2),d(2),a(2),alpha(2));
 T23=dh(Theta(3),d(3),a(3),alpha(3));
 T34=dh(Theta(4),d(4),a(4),alpha(4));
 T45=dh(Theta(5),d(5),a(5),alpha(5));
 T56=dh(Theta(6),d(6),a(6),alpha(6));
 T06=T01*T12*T23*T34*T45*T56;
 robot=load_robot('','Group_19');
 T=directkinematic(robot,q)
%% Q7 Jacobian matrix when picking an M3 screw 
q=[pi 0.5 1 0.25 0 0];
J=compute_jacobian(robot,q)
%% Q8 Tooltip position and orientation for four joint sets 
q1 = [-pi/2 0.37 1.5 -0.27 0 pi/2];
q2 = [pi 0.5 2 0.25 0 0];
q3 = [pi/2 0.2 1.64 -0.5 90 0];
q4 = [pi/2 0.585 1.5 -0.17 -pi/2 pi/2];
drawrobot3d(robot,q4)
%% Q9 Picking and Depositing M3 screw (directly from your code)
J_screw_pick= compute_jacobian(robot,move(27,:))
J_screw_drop= compute_jacobian(robot,move(29,:))
drawrobot3d(robot,move(27,:))
drawrobot3d(robot,move(40,:))
drawrobot3d(robot,move(41,:))
drawrobot3d(robot,move(70,:))
%% Q10 Velocity of joint when handeling M3 screw
% Home
w0 = [pi/2, 1.2, 2.3, 0, 0, 0];
% Pick M3
w1 = [pi, 0.5, 1.5, 0.25, 0, 0];
% Drop M3
w2 = [pi/2, 0.5, 0.765, -0.515, 0, 0];

deltaT = 0.5; T1 = 1; T2 = 1;
deltaw2 = w2 - w1;
deltaw1 = w1 - w0;
a = (T1*deltaw2-T2*deltaw1)/(2*T1*T2*deltaT)
Ts = 0.01;

tau1 = 0:Ts:T1-deltaT;
tau2 = T1-deltaT:Ts:T1+deltaT;
tau3 = T1+deltaT:Ts:T1+T2;

wp1 = zeros(6,length(tau1));
vp1 = zeros(6,length(tau1));
wp2 = zeros(6,length(tau2));
vp1 = zeros(6,length(tau2));
wp4 = zeros(6,length(tau3));
vp1 = zeros(6,length(tau3));
for index = 1:length(tau1)

    wp1(:,index)=w0+tau1(index)*deltaw1/T1;
    vp1(:,index)+deltaw1/T1;
end
for index=1:length(tau2)
    wp2(:,index)=(a/2)*(tau2(index)+(deltaT-T1))^2+(deltaw1/T1)*(tau2(index)-T1)+w1;
    vp2(:,index)=a*(tau2(index)+(deltaT-T1))+deltaw1/T1;
end
for index=1:length(tau3)
    wp3(:,index)=w1+(tau3(index)-1)*deltaw2/T2;
    vp3(:,index)=deltaw2/T2;
end

t1=tau1';
t2=tau2';
t3=tau3';
t = [t1;t2;t3];
wp1 = wp1';
wp2 = wp2';
wp3 = wp3';
wp = [wp1;wp2;wp3];
vp1 = vp1';
vp2 = vp2';
vp3 = vp3';
vp = [vp1;vp2;vp3];

figure(1)
subplot(6,1,1)
plot(t,wp(:,1));
title('Position along X axis (W1)');
ylabel('x- [m]'); 
xlabel('Time [sec]')
grid
subplot (6,1,2)
plot(t,wp(:,2));
title('Position along Y axis (W2)');
ylabel('T [m]');
xlabel('Time [sec]')
grid
subplot(6,1,3)
plot(t,vp(:,3));
title('Position along Z axis (W3)');
ylabel('Z [m]');
xlabel('Time [sec]')
grid 
subplot(6,1,4)
plot(t,vp(:,4));
title('Angular velocity about X axis');
ylabel('Omegax [radian/sec]');
xlabel('Time [sec]')
grid
subplot(6,1,5)
plot(t,vp(:,5));
title('Angular Velocity about y axis');
ylabel('Omegay [radian/sec]');
xlabel('Time [sec]')
grid
subplot(6,1,6)
plot(t,vp(:,6));
title('Angular rotation about Z axis');
ylabel('Omegaz [radian/sec]');
xlabel('Time [sec]')
grid