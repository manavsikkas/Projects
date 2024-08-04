function questions = ques()

%Kinematics

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
 %drawrobot3d(robot,q)
 %qinv=inversekinematic(robot,T);
 J=compute_jacobian(robot,q)
 RD_tool0=[1,[[0,0,0],[1,0,0,0]],[0,[0,0,0],[1,0,0,0],0,0,0]];

%Baseplate
move=[%Base plate 
    0 0.9 0.27 0 0 0;
    0 0.8 0.27 0 0 0;
    -pi/2 0.4 1.5 -0.27 0 pi/2;
    -pi/2 0.5 1.5 -0.27 0 pi/2;
    %Spacer
    pi 0.8 0.75 0 0 0;
    pi 0.7 0.75 0 0 0;
    pi/2 0.44 1.285 -0.015 0 0;
    pi/2 0.41 1.85 -0.015 0 0;
    pi 0.8 0.75 0 0 0;
    pi 0.7 0.75 0 0 0;
    pi/2 0.44 1.25 -0.515 0 0;
    pi/2 0.41 1.285 -0.515 0 0;
    pi 0.8 0.75 0 0 0;
    pi 0.7 0.75 0 0 0;
    pi/2 0.44 0.765 -0.015 0 0;
    pi/2 0.41 0.765 -0.015 0 0;
    pi 0.8 0.75 0 0 0;
    pi 0.7 0.75 0 0 0;
    pi/2 0.44 0.765 -0.515 0 0;
    pi/2 0.41 0.765 -0.515 0 0;
    pi 0.8 0.75 0 0 0;
    pi 0.7 0.75 0 0 0;

    %Top Plate
    0 0.9 1.03 0 0 0;
    0 0.8 1.03 0 0 0;
    -pi/2 0.37 1.5 -0.27 0 pi/2;
    -pi/2 0.47 1.5 -0.27 0 pi/2;

    %Screws

    pi 0.5 1 0.25 0 0;
    pi 0.6 1 0.25 0 0;
    pi/2 0.5 1.285 -0.015 0 0;
    pi/2 0.47 1.285 -0.015 0 0;
    pi 0.5 1 0.25 0 0;
    pi 0.6 1 0.25 0 0;
    pi/2 0.5 1.285 -0.515 0 0;
    pi/2 0.47 1.285 -0.515 0 0;
    pi 0.5 1 0.25 0 0;
    pi 0.6 1 0.25 0 0;
    pi/2 0.5 0.765 -0.015 0 0;
    pi/2 0.47 0.765 -0.015 0 0;
    pi 0.5 1 0.25 0 0;
    pi 0.6 1 0.25 0 0;
    pi/2 0.5 0.765 -0.515 0 0;
    pi/2 0.47 0.765 -0.515 0 0;
    pi 0.5 1 0.25 0 0;
    pi 0.6 1 0.25 0 0;

    %Flip
    pi/2 0.585 1.5 -0.17 -pi/2 pi/2;
    pi/2 .585 1.5 -0.27 -pi/2 pi/2;
    pi/2 0 1.5 -0.27 -pi/2 pi/2;
    pi/2 0 1.5 -0.27 -pi/2 -pi/2;
    pi/2 0.585 1.5 -0.27 -pi/2 -pi/2;
    pi/2 0.585 1.5 -0.17 -pi/2 -pi/2;

    %Nut

    pi 0.5 2 0.25 0 0;
    pi 0.6 2 0.25 0 0;
    pi/2 0.4 1.285 -0.015 0 0;
    pi/2 0.5 1.285 -0.015 0 0;
    pi/2 0.5 1.285 -0.015 0 4*pi;
    pi 0.5 2 0.25 0 0;
    pi 0.6 2 0.25 0 0;
    pi/2 0.4 1.285  -0.515 0 0;
    pi/2 0.5 1.285 -0.515 0 0;
    pi/2 0.5 1.285 -0.015 0 4*pi;
     pi 0.5 2 0.25 0 0;
    pi 0.6 2 0.25 0 0;
    pi/2 0.4 0.765 -0.015 0 0;
    pi/2 0.5 0.765 -0.015 0 0;
    pi/2 0.5 0.765 -0.015 0 pi*4;
     pi 0.5 2 0.25 0 0;
    pi 0.6 2 0.25 0 0;
    pi/2 0.4 0.765 -0.515 0 0;
    pi/2 0.5 0.765 -0.515 0 0;
    pi/2 0.5 0.7650 -0.515 0 4*pi;

    %FLip again
    pi/2 0.585 1.5 -0.17 -pi/2 pi/2;
    pi/2 0 1.5 -0.27 -pi/2 pi/2;
    pi/2 0.585 1.5 -0.27 -pi/2 -pi/2;
    pi/2 0.585 1.5 -0.17 -pi/2 -pi/2;


    %wheels
    pi 0.5 1.86 0 0 0;
    pi 0.4 1.86 0 0 0;
    pi/2 0.6 1.64 -0.46 pi/2 0;
    pi/2 0.2 1.64 -0.5 pi/2 0;
    pi/2 0.6 1.64 -0.5 pi/2 0;
    pi 0.5 1.86 0 0 0;
    pi 0.4 1.86 0 0 0;
    pi/2 0.6 1.64 -0.4 -pi/2 0;
    pi/2 0.2 1.64 0 -pi/2 0;
    pi 0.5 1.86 0 0 0;
    pi 0.4 1.86 0 0 0;
    pi/2 0.6 1.14 -0.46 pi/2 0;
    pi/2 0.2 1.14 -0.5 pi/2 0;
    pi/2 0.6 1.14 -0.5 pi/2 0;
    pi 0.5 1.86 0 0 0;
    pi 0.4 1.86 0 0 0;
    pi/2 0.6 1.14 -0.4 -pi/2 0;
    pi/2 0.6 1.14 -0 -pi/2 0;
    
    ];
%Jacobian when picking up and dropping the M3 screw
J_screw_pick= compute_jacobian(robot,move(27,:))
J_screw_drop= compute_jacobian(robot,move(29,:))
drawrobot3d(robot,move(27,:))
drawrobot3d(robot,move(40,:))
drawrobot3d(robot,move(41,:))
drawrobot3d(robot,move(70,:))

for i=1:91
    a=move(i,:)
    MoveAbsJ(move(i,:), 'vmax','fine',RD_tool0,'wobj0')
end



