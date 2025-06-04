clear; clc; clf;


y0=[0;0;10;0];
h=0.15;
tspan=[0 30];
parameters.ks1=2000;
parameters.ks2=1300;
parameters.m1=50;
parameters.m2=10;
F=200;
%model.c2=sqrt(2*model.m2*model.ks2)*0.1;
parameters.c2=200;

%%
[t,y]= ode45(@(t,y) modeldynamics(t,y,F,parameters), tspan, y0);
%% 

subplot (2,1,1); hold on;
plot(t, y(:,1),'-b');
plot(t, y(:,2),'-g');
xlabel('time(s)');
ylabel('displacement(x)');
grid on;

subplot (2,1,2); hold on;
plot(t, y(:,3),'-b');
plot(t, y(:,4),'-g');
xlabel('time(s)');
ylabel('velocities(v)');
grid on;



%% 

function dydt= modeldynamics(t,y,F,parameters)
dydt=zeros(4,1);
dydt(1)=y(3);
dydt(2)= y(4);
dydt(3)= (F+parameters.ks2*y(2)-parameters.c2*(y(3)-y(4))-(parameters.ks1+parameters.ks2)*y(1))/parameters.m1;
dydt(4)=(parameters.c2*(y(3)-y(4))+parameters.ks2*(y(1)-y(2)))/parameters.m2;
end
