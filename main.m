%Once an obstacle is detected. Vehicle takes its position and its no longer an obstacle as we already detected it once.
clc;
clear;
close all;
t=1:0.01:100; %time [1 100] with step of 0.01sec
j= length(t);
k=1:j;
%Random Signal Generation [0--red 1--green]
rand_signal=randi([0,1],1);
for i=1:0.01:100
    if i>=0 && i<=20
        x(k(1,(1:1901)))=rand_signal;
    else if i>=20 && i<=40
            x(k(1,(1901:3901)))=1-rand_signal;
        else if i>=40 && i<=60
                x(k(1,(3901:5901)))=rand_signal;
            else if i>=60 && i<=80
                    x(k(1,(5901:7901)))=1-rand_signal;
                else if i>=80 && i<=100
                        x(k(1,(7901:9901)))=rand_signal;
                    end
                end
            end
        end
    end
end
figure(1)
plot(t(1,:),x(1,:));
title('Random Signal vs Time');
xlabel('time in sec');
ylabel('0=red or 1=green');
rand_obstacle=randi([0 1],1,1);
rand_time= round(1+79*rand(1,1),2); %%Obstacles generated at random times within the range[1,80]
distance=1:0.01:200;
rand_distance= round(1+199*rand(1,1),2);
%rand_timeinterval=round(10+10*rand(1,1),2); %%random time interval till obstacle stays[10 20]
rand_distanceinterval=75-rand_distance;
a=zeros(1,find(t==100));
b=zeros(1,find(distance==200));
%c=rand_time+rand_timeinterval;
d=find(abs(t-rand_time)<1e-8);
%e=find(t==c);
f=find(abs(distance-rand_distance)<1e-8);
a(1,(d:find(t==100)))=rand_obstacle;
b(1,(f:find(distance==200)))=rand_obstacle;
figure(2);
plot(t(1,:),a(1,:));
title('Obstacle Presence vs Time');
xlabel('time in sec');
ylabel('0=Obstacle or 1=No Obstacle')
figure(3);
plot(distance(1,:),b(1,:));
title('Obstacle Presence vs Distance');
xlabel('Distance in metres');
ylabel('0=Obstacle or 1=No Obstacle')
fid= fopen('Braking distance vs speed.txt','r');
ii=1;
while ~feof(fid)
    line=fgetl(fid);
    comp=strcmp(line,'Speed	Reaction distance	Braking distance	Total stopping distance');
    if comp==1
        break;
    end
    split=strsplit(line);
    z(ii,:)=str2double(split);
    ii=ii+1;
end
while ~feof(fid)
    line=fgetl(fid);
    split=strsplit(line);
    z(ii,:)=str2double(split);
    ii=ii+1;
end
speed_dry=z((1:8),1);
speed_wet=z((9:16),1);
distance_dry=z((1:8),4);
distance_wet=z((9:16),4);
p=fitlm(speed_dry,distance_dry,'quadratic');
pp=predict(p,speed_dry);
q=fitlm(speed_wet,distance_wet,'quadratic');
qq=predict(q,speed_wet);
figure(4)
subplot(2,1,1)
plot(speed_dry,distance_dry,'b o')
hold on
plot(speed_dry,pp,'r');
title('Curve fitting for Safe distance vs Speed under Dry Conditions');
xlabel('Speed');
ylabel('Safe Distance');
subplot(2,1,2)
plot(speed_wet,distance_wet,'g o')
hold on
plot(speed_wet,qq,'m');
title('Curve fitting for Safe distance vs Speed under Wet Conditions');
xlabel('Speed');
ylabel('Safe Distance');
input_obstcalespeed=round(10+10*rand(1,1),2);
input_speed=round(10+20*rand(1,1),2);
input_position=round(1+199*rand(1,1),2);
input_weather=randi([0 1],1,1); % Random weather conditions [0--Wet 1--Dry]
if input_weather==1 %dry conditions
    safe_distance=predict(p,input_speed);
    uk= 0.6;
else
    safe_distance=predict(q,input_speed);
    uk=0.3;
end
mass_car=1302;
mass_wheel=1302/4;
outer_tyrerad=0.35;
outer_rotorrad=0.165;
inner_rotorrad=0.154;
mean_rotorrad=(outer_rotorrad+inner_rotorrad)/2;
bore_dia=0.0384;
area_pedal=5.06e-4;
mean_radius=(outer_rotorrad+inner_rotorrad)/2;
tt=find(abs(t-rand_time)<1e-8);
xx=x(tt);%if its green or red
dd=find(abs(distance-rand_distance)<1e-8);
bb=b(dd); %obstacle presence
if bb==1
    if xx==1% green light
        if input_position>rand_distance
            if rand_distance<=safe_distance
                final_speed=round(15+5*rand(1,1),2);
                accl=abs((final_speed*final_speed-input_speed*input_speed)/(2*(input_position-rand_distance)));
                forc=mass_wheel*accl;
                torq=forc*outer_tyrerad;
                Press=4*torq/(uk*pi*bore_dia*bore_dia*mean_rotorrad*6);
                force_pedal=Press*area_pedal;
                my_time=rand_distance/final_speed; %time from traffic pole
                time_reach=(final_speed-input_speed)/accl;
                timeforchange=t(tt)+time_reach;
                rem_time=20-mod(timeforchange,20);
                if my_time<=rem_time
                    message= sprintf('Its a green light.\nThere is an obstacle and its in the safe radius.\nVehicle takes its position and crosses the line with %f m/s',final_speed);
                else
                    final_speed=0;
                    accl=abs((final_speed*final_speed-input_speed*input_speed)/(2*rand_distance));
                    forc=mass_wheel*accl;
                    torq=forc*outer_tyrerad;
                    Press=4*torq/(uk*pi*bore_dia*bore_dia*mean_rotorrad*6);
                    force_pedal=Press*area_pedal;
                    message= sprintf('Green shifed to Red.\nObstacle is no more in the safe radius.\nStops near the line');
                end
            else
                my_time=input_position/input_speed; %time from traffic pole
                rem_time=20-mod(my_time,20);
                if my_time<=rem_time
                    message= sprintf('No Obstacle detected within the safe radius.\nIts a Green.\nCrosses the line with %f m/s',input_speed);
                else
                    final_speed=0;
                    accl=abs((final_speed*final_speed-input_speed*input_speed)/(2*input_position));
                    forc=mass_wheel*accl;
                    torq=forc*outer_tyrerad;
                    Press=4*torq/(uk*pi*bore_dia*bore_dia*mean_rotorrad*6);
                    force_pedal=Press*area_pedal;
                    message= sprintf('No Obstacle within the safe radius.\nIts a Red.\nStops near the line');
                end
            end
        else
            my_time=input_position/input_speed; %time from traffic pole
            rem_time=20-mod(my_time,20);
            if my_time<=rem_time
                message= sprintf('Position of obstacle is either behind the vehicle or beside the vehicle.\nIts a Green.\nCrosses the line with %f m/s',input_speed);
            else
                final_speed=0;
                accl=abs((final_speed*final_speed-input_speed*input_speed)/(2*input_position));
                forc=mass_wheel*accl;
                torq=forc*outer_tyrerad;
                Press=4*torq/(uk*pi*bore_dia*bore_dia*mean_rotorrad*6);
                force_pedal=Press*area_pedal;
                message= sprintf('Position of obstacle is either behind the vehicle or beside the vehicle.\nGreen shifted to Red.\nStops at the line');
            end
        end
    else
        if input_position>rand_distance
            if rand_distance<=safe_distance
                final_speed=round(5+5*rand(1,1),2);
                accl=abs((final_speed*final_speed-input_speed*input_speed)/(2*(input_position-rand_distance)));
                forc=mass_wheel*accl;
                torq=forc*outer_tyrerad;
                Press=4*torq/(uk*pi*bore_dia*bore_dia*mean_rotorrad*6);
                force_pedal=Press*area_pedal;
                my_time=rand_distance/final_speed; %time from traffic pole
                time_reach=(final_speed-input_speed)/accl; % time to reach the obstacle position
                timeforchange=t(tt)+time_reach; % final time from the obstacle position
                rem_time=20-mod(timeforchange,20);
                if my_time<=rem_time
                    final_speed=0;
                    accl=abs((final_speed*final_speed-input_speed*input_speed)/(2*rand_distance));
                    forc=mass_wheel*accl;
                    torq=forc*outer_tyrerad;
                    Press=4*torq/(uk*pi*bore_dia*bore_dia*mean_rotorrad*6);
                    force_pedal=Press*area_pedal;
                    message= sprintf('Its a Red light.\nThere is an obstacle and its in the safe radius.\nVehicle takes its position\nStops near the line');
                else
                    message= sprintf('Red shifed to Green.\nObstacle is no more in the safe radius.\nCrosses the line with %f m/s',final_speed);
                end
            else
                my_time=input_position/input_speed; %time from traffic pole
                rem_time=20-mod(my_time,20);
                if my_time<=rem_time
                    final_speed=0;
                    accl=abs((final_speed*final_speed-input_speed*input_speed)/(2*input_position));
                    forc=mass_wheel*accl;
                    torq=forc*outer_tyrerad;
                    Press=4*torq/(uk*pi*bore_dia*bore_dia*mean_rotorrad*6);
                    force_pedal=Press*area_pedal;
                    message= sprintf('No Obstacle within the safe radius.\nIts a Red.\nStops near the line');
                else
                    message= sprintf('No Obstacle detected within the safe radius.\nRed shifted to Green.\nCrosses the line with %f m/s',input_speed);
                end
            end
        else
            my_time=input_position/input_speed; %time from traffic pole
            rem_time=20-mod(my_time,20);
            if my_time>=rem_time
                message= sprintf('Position of obstacle is either behind the vehicle or beside the vehicle.\nRed shifted to Green.\nCrosses the line with %f m/s',input_speed);
            else
                final_speed=0;
                accl=abs((final_speed*final_speed-input_speed*input_speed)/(2*input_position));
                forc=mass_wheel*accl;
                torq=forc*outer_tyrerad;
                Press=4*torq/(uk*pi*bore_dia*bore_dia*mean_rotorrad*6);
                force_pedal=Press*area_pedal;
                message= sprintf('Position of obstacle is either behind the vehicle or beside the vehicle.\n Its a Red.\nStops at the line');
            end
        end
    end
else
    if xx==1
        my_time=input_position/input_speed; %time from traffic pole
        rem_time=20-mod(my_time,20);
        if my_time<=rem_time
            message= sprintf('Theres no obstacle.\nIts a Green.\nCrosses the line with %f m/s',input_speed);
        else
            final_speed=0;
            accl=abs((final_speed*final_speed-input_speed*input_speed)/(2*input_position));
            forc=mass_wheel*accl;
            torq=forc*outer_tyrerad;
            Press=4*torq/(uk*pi*bore_dia*bore_dia*mean_rotorrad*6);
            force_pedal=Press*area_pedal;
            message= sprintf('Thers no obstacle.\nGreen shifted to Red\n.Stops at the line');
        end
    else
        my_time=input_position/input_speed; %time from traffic pole
        rem_time=20-mod(my_time,20);
        if my_time>=rem_time
            message= sprintf('Theres no obstacle.\nRed shifted to Green.\nCross the line with %f m/s',input_speed);
        else
            final_speed=0;
            accl=abs((final_speed*final_speed-input_speed*input_speed)/(2*input_position));
            forc=mass_wheel*accl;
            torq=forc*outer_tyrerad;
            Press=4*torq/(uk*pi*bore_dia*bore_dia*mean_rotorrad*6);
            force_pedal=Press*area_pedal;
            message= sprintf('Thers no obstacle.\nIts Red.\nStops at the line');
        end
    end
end
uiwait(helpdlg(message));






















































