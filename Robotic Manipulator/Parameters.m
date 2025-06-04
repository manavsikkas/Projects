
function robot = parameters()

robot.name= 'gr19';
robot.DH.theta= '[q(1)+pi/2 pi/2 pi/2 pi q(5)+pi/2 q(6)]';
robot.DH.d='[1.2-q(2) 0 2.5-q(3) q(4) 0 0.100]';
robot.DH.a='[0 0 0 0.100 0 0]';
robot.DH.alpha= '[0 pi/2 pi/2 pi/2 pi/2 0]';
robot.J=[];

%q=[0 1.200 2.500 0.460q
% 0 0];
robot.inversekinematic_fn = 'inversekinematic_gr19(robot, T)';
robot.directkinematic_fn = 'directkinematic(robot, q)';


%number of degrees of freedom
robot.DOF = 6;

%rotational: 0, translational: 1
robot.kind=['R' 'T' 'T' 'T' 'R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[-pi pi; %Axis 1, minimum, maximum
                -100 100; %Axis 2, minimum, maximum
                -100 100; %Axis 3
                -100 100; %Axis 4
                deg2rad(-90) deg2rad(900); %Axis 5
                deg2rad(-400) deg2rad(400)]; 

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(200); %Axis 1, rad/s
                deg2rad(200); %Axis 2, rad/s
                deg2rad(260); %Axis 3, rad/s
                deg2rad(360); %Axis 4, rad/s
                deg2rad(360); %Axis 5, rad/s
                deg2rad(450)];%Axis 6, rad/s
    
% robot.velmax = [deg2rad(200); %Axis 1, rad/s
%                 deg2rad(200); %Axis 2, rad/s
%                 deg2rad(200); %Axis 3, rad/s
%                 deg2rad(400); %Axis 4, rad/s
%                 deg2rad(400); %Axis 5, rad/s
%                 deg2rad(400)];%Axis 6, rad/s
            
            
robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time
            
% end effectors maximum velocity
robot.linear_velmax = 1; %m/s



%base reference system
robot.T0 = eye(4);
% robot.T0 = [1 0 0 0;q=q
%             0 -1 0 0;
%             0 0 -1 1;
%             0   0   0   1];


%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);
robot.path = pwd;


% GRAPHICS
robot.graphical.has_graphics=0;
robot.graphical.color = [255 102 51]./255;
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=0.3;
%adjust for a default view of the robot
robot.axis=[-0.5 0.75 -0.75 0.75 0 1.1];
%read graphics files
robot = read_graphics(robot);