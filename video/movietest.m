
%%
t_start = 12.0;
t_end = 40;

real_state = loadStateEstimate('real car/opendlv.logic.legacy.StateEstimate-0.csv');
sim1_state = loadStateEstimate('sim car1/opendlv.logic.legacy.StateEstimate-0.csv');
sim2_state = loadStateEstimate('sim car2/opendlv.logic.legacy.StateEstimate-0.csv');
sim3_state = loadStateEstimate('sim car3/opendlv.logic.legacy.StateEstimate-0.csv');
sim4_state = loadStateEstimate('sim car4/opendlv.logic.legacy.StateEstimate-0.csv');

real_tunerState = loadTunerState('real car/opendlv.logic.legacy.VelocityTunerState-0.csv');
sim1_tunerState = loadTunerState('sim car1/opendlv.logic.legacy.VelocityTunerState-0.csv');
sim2_tunerState = loadTunerState('sim car2/opendlv.logic.legacy.VelocityTunerState-0.csv');
sim3_tunerState = loadTunerState('sim car3/opendlv.logic.legacy.VelocityTunerState-0.csv');
sim4_tunerState = loadTunerState('sim car4/opendlv.logic.legacy.VelocityTunerState-0.csv');

t_ref = sim1_state.SampleTimeStamp(1);

real_speed = loadGroundSpeed('real car/opendlv.proxy.GroundSpeedReading-0.csv');

real_timeSlot = loadTimeSlot('real car/opendlv.logic.legacy.TimeSlot-0.csv');
idx = find(real_timeSlot.vehicleID == 113,1,'first');
real_entryTime = real_timeSlot.entryTimeseconds(idx)...
	+ real_timeSlot.entryTimemicroseconds(idx)*1e-6...
    - real_speed.SampleTimeStamp(1);

sim_timeSlot = loadTimeSlot('sim scheduler/opendlv.logic.legacy.TimeSlot-0.csv');
idx = find(sim_timeSlot.vehicleID == 120,1,'first');
sim1_entryTime = sim_timeSlot.entryTimeseconds(idx)...
    + sim_timeSlot.entryTimemicroseconds(idx)*1e-6...
    - t_ref;
sim1_exitTime = sim_timeSlot.exitTimeseconds(idx)...
    + sim_timeSlot.entryTimemicroseconds(idx)*1e-6...
    - t_ref;
idx = find(sim_timeSlot.vehicleID == 121,1,'first');
sim2_entryTime = sim_timeSlot.entryTimeseconds(idx)...
    + sim_timeSlot.entryTimemicroseconds(idx)*1e-6...
    - t_ref;
sim2_exitTime = sim_timeSlot.exitTimeseconds(idx)...
    + sim_timeSlot.entryTimemicroseconds(idx)*1e-6...
    - t_ref;

real_offset = sim1_entryTime-real_entryTime-real_state.SampleTimeStamp(1);


% manually found time when the car cross the intersection
t_real_cross = 26.34;
t_sim2_cross = 31.24;
t_sim3_cross = 26.35;
t_sim4_cross = 31.07;

x_real_cross = interp1(real_state.SampleTimeStamp+real_offset, real_state.positionX, t_real_cross);
x_sim2_cross = interp1(sim2_state.SampleTimeStamp-t_ref, sim2_state.positionX, t_sim2_cross);
x_sim3_cross = interp1(sim3_state.SampleTimeStamp-t_ref, sim3_state.positionX, t_sim3_cross);
x_sim4_cross = interp1(sim4_state.SampleTimeStamp-t_ref, sim4_state.positionX, t_sim4_cross);

y_real_cross = interp1(real_state.SampleTimeStamp+real_offset, real_state.positionY, t_real_cross);
y_sim2_cross = interp1(sim2_state.SampleTimeStamp-t_ref, sim2_state.positionY, t_sim2_cross);
y_sim3_cross = interp1(sim3_state.SampleTimeStamp-t_ref, sim3_state.positionY, t_sim3_cross);
y_sim4_cross = interp1(sim4_state.SampleTimeStamp-t_ref, sim4_state.positionY, t_sim4_cross);

%% figure;
% plot(real_speed.SampleTimeStamp+real_offset,real_speed.groundSpeed*3.6,'b-')
% hold on
% % plot(sim1_state.SampleTimeStamp-sim1_state.SampleTimeStamp(1),sim1_state.velocityX*3.6)
% plot(sim2_state.SampleTimeStamp-t_ref,sim2_state.velocityX*3.6,'r-')
% plot(sim3_state.SampleTimeStamp-t_ref,sim3_state.velocityX*3.6,'b--')
% plot(sim4_state.SampleTimeStamp-t_ref,sim4_state.velocityX*3.6,'r--')
% 
% % plot(real_entryTime+real_offset,40,'x')
% rectangle('Position',[sim1_entryTime,-10,sim1_exitTime-sim1_entryTime,100],'FaceColor',[0 0 1 0.1],'EdgeColor','k','LineWidth',1)
% rectangle('Position',[sim2_entryTime,-10,sim2_exitTime-sim2_entryTime,100],'FaceColor',[1 0 0 0.1],'EdgeColor','k','LineWidth',1)
% 
% xlim([t_start t_end])
% ylim([0 50])
% % 
% figure;

intersection_halfwidth = 8;
lane_halfwidth = 2.5;


x1 = real_state.positionX-x_real_cross;
y1 = real_state.positionY-y_real_cross;
ang1 = atan2(y_real_cross-y1(1),x_real_cross-x1(1));
xo1 = -intersection_halfwidth;
yo1 = -lane_halfwidth;

x2 = sim2_state.positionX-x_sim2_cross;
y2 = sim2_state.positionY-y_sim2_cross;
ang2 = atan2(y_sim2_cross-y2(1),x_sim2_cross-x2(1))+pi/2;
xo2 = -lane_halfwidth;
yo2 = intersection_halfwidth;

x3 = sim3_state.positionX-x_sim3_cross;
y3 = sim3_state.positionY-y_sim3_cross;
ang3 = atan2(y_sim3_cross-y3(1),x_sim3_cross-x3(1))+pi;
xo3 = intersection_halfwidth;
yo3 = lane_halfwidth;

x4 = sim4_state.positionX-x_sim4_cross;
y4 = sim4_state.positionY-y_sim4_cross;
ang4 = atan2(y_sim4_cross-y4(1),x_sim4_cross-x4(1))+3*pi/2;
xo4 = lane_halfwidth;
yo4 = -intersection_halfwidth;

rectangle('Position',[-150,-lane_halfwidth*2,300,lane_halfwidth*4],'FaceColor',0.5*[1 1 1],'EdgeColor','none')
hold on
rectangle('Position',[-lane_halfwidth*2,-150,lane_halfwidth*4,300],'FaceColor',0.5*[1 1 1],'EdgeColor','none')

plot([-intersection_halfwidth -150], [0 0], 'w--','LineWidth',1)
plot([intersection_halfwidth 150], [0 0], 'w--','LineWidth',1)
plot([0 0], [intersection_halfwidth 150], 'w--','LineWidth',1)
plot([0 0], [-intersection_halfwidth -150], 'w--','LineWidth',1)

plot(x1*cos(ang1)+y1*sin(ang1)+xo1, -x1*sin(ang1)+y1*cos(ang1)+yo1)
plot(x2*cos(ang2)+y2*sin(ang2)+xo2, -x2*sin(ang2)+y2*cos(ang2)+yo2)
plot(x3*cos(ang3)+y3*sin(ang3)+xo3, -x3*sin(ang3)+y3*cos(ang3)+yo3)
plot(x4*cos(ang4)+y4*sin(ang4)+xo4, -x4*sin(ang4)+y4*cos(ang4)+yo4)

xlim([-50 50])
ylim([-50 50])
axis equal

% 
% figure;
% plot(real_state.SampleTimeStamp+real_offset, real_state.positionY)
% hold on
% % plot(sim1_state.positionX, sim1_state.positionY)
% plot(sim2_state.SampleTimeStamp-t_ref, sim2_state.positionY)
% plot(sim3_state.SampleTimeStamp-t_ref, sim3_state.positionY)
% plot(sim4_state.SampleTimeStamp-t_ref, sim4_state.positionY)

 
%% Set up the movie.
writerObj = VideoWriter('velocities.avi'); % Name it.
writerObj.FrameRate = 25; % How many frames per second.
open(writerObj); 

fh1 = figure('units','pixels','position',[0 0 1920 1080]);
 
for t = t_start:(1/writerObj.FrameRate):t_end
    
    t1 = real_speed.SampleTimeStamp+real_offset;
    ii1 = t1 >= t_start & t1 <= t;
    t2 = sim2_state.SampleTimeStamp-sim1_state.SampleTimeStamp(1);
    ii2 = t2 >= t_start & t2 <= t;
    t3 = sim3_state.SampleTimeStamp-sim1_state.SampleTimeStamp(1);
    ii3 = t3 >= t_start & t3 <= t;
    t4 = sim4_state.SampleTimeStamp-sim1_state.SampleTimeStamp(1);
    ii4 = t4 >= t_start & t4 <= t;
    
    figure(fh1)
    plot(t1(ii1)-t,real_speed.groundSpeed(ii1)*3.6,'b-')
    hold on
    plot(t2(ii2)-t,sim2_state.velocityX(ii2)*3.6,'r-')
    plot(t3(ii3)-t,sim3_state.velocityX(ii3)*3.6,'b--')
    plot(t4(ii4)-t,sim4_state.velocityX(ii4)*3.6,'r--')
    xlim([t_start-t t_end-t])
    ylim([0 50])
    set(findall(gca, 'Type', 'Line'),'LineWidth',2);
    xlabel('Time [s]')
    ylabel('Velocity [m/s]')
    
    % Timeslot
    rectangle('Position',[sim1_entryTime-t,-10,sim1_exitTime-sim1_entryTime,100],'FaceColor',[0 0 1 0.1],'EdgeColor','k','LineWidth',1)
    rectangle('Position',[sim2_entryTime-t,-10,sim2_exitTime-sim2_entryTime,100],'FaceColor',[1 0 0 0.1],'EdgeColor','k','LineWidth',1)
    
    hold off
    
    frame = getframe(gcf);
    writeVideo(writerObj, frame);
 
end
hold off
close(writerObj); % Saves the movie.


%%

%% Set up the movie.
writerObj = VideoWriter('topview.avi'); % Name it.
writerObj.FrameRate = 25; % How many frames per second.
open(writerObj); 

fh2 = figure('units','pixels','position',[0 0 1080 1080]);
 
for t = t_start:(1/writerObj.FrameRate):t_end
% for t = 32.2
    
    t1 = real_state.SampleTimeStamp+real_offset;
    t2 = sim2_state.SampleTimeStamp-sim1_state.SampleTimeStamp(1);
    t3 = sim3_state.SampleTimeStamp-sim1_state.SampleTimeStamp(1);
    t4 = sim4_state.SampleTimeStamp-sim1_state.SampleTimeStamp(1);
    
    figure(fh2)

    intersection_halfwidth = 10;
    lane_halfwidth = 3;

    x1 = real_state.positionX-x_real_cross;
    y1 = real_state.positionY-y_real_cross;
    ang1 = atan2(y_real_cross-y1(1),x_real_cross-x1(1));
    xo1 = -intersection_halfwidth;
    yo1 = -lane_halfwidth;
    xx1n = interp1(t1,x1*cos(ang1)+y1*sin(ang1)+xo1,t);
    yy1n = interp1(t1,-x1*sin(ang1)+y1*cos(ang1)+yo1,t);
    if ~isnan(xx1n)
       xx1 = xx1n; 
       yy1 = yy1n;
    end

    x2 = sim2_state.positionX-x_sim2_cross;
    y2 = sim2_state.positionY-y_sim2_cross;
    ang2 = atan2(y_sim2_cross-y2(1),x_sim2_cross-x2(1))+pi/2;
    xo2 = -lane_halfwidth;
    yo2 = intersection_halfwidth;
    xx2=interp1(t2,x2*cos(ang2)+y2*sin(ang2)+xo2,t);
    yy2=interp1(t2,-x2*sin(ang2)+y2*cos(ang2)+yo2,t);

    x3 = sim3_state.positionX-x_sim3_cross;
    y3 = sim3_state.positionY-y_sim3_cross;
    ang3 = atan2(y_sim3_cross-y3(1),x_sim3_cross-x3(1))+pi;
    xo3 = intersection_halfwidth;
    yo3 = lane_halfwidth;
    xx3=interp1(t3,x3*cos(ang3)+y3*sin(ang3)+xo3,t);
    yy3=interp1(t3,-x3*sin(ang3)+y3*cos(ang3)+yo3,t);

    x4 = sim4_state.positionX-x_sim4_cross;
    y4 = sim4_state.positionY-y_sim4_cross;
    ang4 = atan2(y_sim4_cross-y4(1),x_sim4_cross-x4(1))+3*pi/2;
    xo4 = lane_halfwidth;
    yo4 = -intersection_halfwidth;
    xx4=interp1(t4,x4*cos(ang4)+y4*sin(ang4)+xo4,t);
    yy4=interp1(t4,-x4*sin(ang4)+y4*cos(ang4)+yo4,t);
    
    rectangle('Position',[-150,-lane_halfwidth*2,300,lane_halfwidth*4],'FaceColor',0.5*[1 1 1],'EdgeColor','none')
    hold on
    rectangle('Position',[-lane_halfwidth*2,-150,lane_halfwidth*4,300],'FaceColor',0.5*[1 1 1],'EdgeColor','none')

    plot([-intersection_halfwidth -150], [0 0], 'w--','LineWidth',1)
    plot([intersection_halfwidth 150], [0 0], 'w--','LineWidth',1)
    plot([0 0], [intersection_halfwidth 150], 'w--','LineWidth',1)
    plot([0 0], [-intersection_halfwidth -150], 'w--','LineWidth',1)

    L = 4.8;
    W = 1.9;
    
    if t > sim1_entryTime && t < sim1_exitTime
        color1 = 'green';
    else
        color1 = 'red';
    end
    if t > sim2_entryTime && t < sim2_exitTime
        color2 = 'green';
    else
        color2 = 'red';
    end
    rectangle('Position',[xx1-L,yy1-W/2,L, W],'LineWidth',2,'FaceColor',color1)
    rectangle('Position',[xx2-W/2,yy2, W,L]  ,'LineWidth',2,'FaceColor',color2)
    rectangle('Position',[xx3+L,yy3-W/2,L, W],'LineWidth',2,'FaceColor',color1)
    rectangle('Position',[xx4-W/2,yy4-L, W,L],'LineWidth',2,'FaceColor',color2)
    ylim([-50 50])
    axis equal
    xlim([-60 40])
    axis off
    
    hold off
    
    frame = getframe(gcf);
    writeVideo(writerObj, frame);
 
end
hold off
close(writerObj); % Saves the movie.