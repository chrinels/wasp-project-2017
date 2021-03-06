
%%
t_start = 12.0;
t_end = 40;

real_locOnPath = loadLocOnPath('real car/opendlv.logic.legacy.LocationOnPathToIntersection-0.csv');

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

% Time instant where the vehicles are scheduled
dummy_offset = 5; % For nice graphics
sim1_schedulingTime = sim_timeSlot.SentTimeStamp(find(sim_timeSlot.vehicleID == 120,1,'first')) - t_ref + dummy_offset;
sim2_schedulingTime = sim_timeSlot.SentTimeStamp(find(sim_timeSlot.vehicleID == 121,1,'first')) - t_ref + dummy_offset;
sim3_schedulingTime = sim_timeSlot.SentTimeStamp(find(sim_timeSlot.vehicleID == 122,1,'first')) - t_ref + dummy_offset;
sim4_schedulingTime = sim_timeSlot.SentTimeStamp(find(sim_timeSlot.vehicleID == 123,1,'first')) - t_ref + dummy_offset;

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
    plot(t1(ii1),real_speed.groundSpeed(ii1)*3.6,'b-')
    hold on
    plot(t2(ii2),sim2_state.velocityX(ii2)*3.6,'r-')
    plot(t3(ii3),sim3_state.velocityX(ii3)*3.6,'b--')
    plot(t4(ii4),sim4_state.velocityX(ii4)*3.6,'r--')
    xlim([t_start t_end])
    ylim([0 50])
    set(findall(gca, 'Type', 'Line'),'LineWidth',2);
    xlabel('Time [s]')
    ylabel('Velocity [km/h]')
    
    % Timeslot
    rectangle('Position',[sim1_entryTime,-10,sim1_exitTime-sim1_entryTime,100],'FaceColor',[0 0 1 0.1],'EdgeColor','k','LineWidth',1)
    rectangle('Position',[sim2_entryTime,-10,sim2_exitTime-sim2_entryTime,100],'FaceColor',[1 0 0 0.1],'EdgeColor','k','LineWidth',1)
    
    hold off
    
%     frame = getframe(gcf);
%     writeVideo(writerObj, frame);
    set(fh1,'GraphicsSmoothing','on')
    print(fh1,'video_image.png','-dpng')
    img = imread('video_image.png');
    writeVideo(writerObj, img);
 
end
hold off
close(writerObj); % Saves the movie.


%%

%% Set up the movie.
writerObj = VideoWriter('topview.avi'); % Name it.
writerObj.FrameRate = 25; % How many frames per second.
open(writerObj); 

fh2 = figure('units','pixels','position',[0 0 1920 1080]);
 
% Define trajectory vectors
NS.start = [-intersection_halfwidth,-lane_halfwidth];
NS.traj =  [intersection_halfwidth*2 0];
SS.start = [intersection_halfwidth,lane_halfwidth];
SS.traj =  [-intersection_halfwidth*2 0];
ES.start = [lane_halfwidth,-intersection_halfwidth];
ES.traj =  [0 intersection_halfwidth*2];
WS.start = [-lane_halfwidth,intersection_halfwidth];
WS.traj =  [0 -intersection_halfwidth*2];
        
scale_factor = 0.5;
for t = t_start:(1/writerObj.FrameRate):t_end
% for t = 31.5
    clf(fh2)
    t
    
    t1 = real_state.SampleTimeStamp+real_offset;
    t2 = sim2_state.SampleTimeStamp-sim1_state.SampleTimeStamp(1);
    t3 = sim3_state.SampleTimeStamp-sim1_state.SampleTimeStamp(1);
    t4 = sim4_state.SampleTimeStamp-sim1_state.SampleTimeStamp(1);
    
%     figure(fh2)

    intersection_halfwidth = 8;
    lane_halfwidth = 3;

    x1 = real_state.positionX-x_real_cross;
    y1 = real_state.positionY-y_real_cross;
    ang1 = atan2(y_real_cross-y1(1),x_real_cross-x1(1));
    xo1 = -intersection_halfwidth/scale_factor;
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
    yo2 = intersection_halfwidth/scale_factor;
    xx2=interp1(t2,x2*cos(ang2)+y2*sin(ang2)+xo2,t);
    yy2=interp1(t2,-x2*sin(ang2)+y2*cos(ang2)+yo2,t);

    x3 = sim3_state.positionX-x_sim3_cross;
    y3 = sim3_state.positionY-y_sim3_cross;
    ang3 = atan2(y_sim3_cross-y3(1),x_sim3_cross-x3(1))+pi;
    xo3 = intersection_halfwidth/scale_factor;
    yo3 = lane_halfwidth;
    xx3=interp1(t3,x3*cos(ang3)+y3*sin(ang3)+xo3,t);
    yy3=interp1(t3,-x3*sin(ang3)+y3*cos(ang3)+yo3,t);

    x4 = sim4_state.positionX-x_sim4_cross;
    y4 = sim4_state.positionY-y_sim4_cross;
    ang4 = atan2(y_sim4_cross-y4(1),x_sim4_cross-x4(1))+3*pi/2;
    xo4 = lane_halfwidth;
    yo4 = -intersection_halfwidth/scale_factor;
    xx4=interp1(t4,x4*cos(ang4)+y4*sin(ang4)+xo4,t);
    yy4=interp1(t4,-x4*sin(ang4)+y4*cos(ang4)+yo4,t);
    
    rectangle('Position',[-150,-lane_halfwidth*2,300,lane_halfwidth*4],'FaceColor',0.5*[1 1 1],'EdgeColor','none')
    hold on
    rectangle('Position',[-lane_halfwidth*2,-150,lane_halfwidth*4,300],'FaceColor',0.5*[1 1 1],'EdgeColor','none')

    plot([-intersection_halfwidth -150], [0 0], '--','LineWidth',1.5,'Color',[.99 .99 .99])
    plot([intersection_halfwidth 150], [0 0], '--','LineWidth',1.5,'Color',[.99 .99 .99])
    plot([0 0], [intersection_halfwidth 150], '--','LineWidth',1.5,'Color',[.99 .99 .99])
    plot([0 0], [-intersection_halfwidth -150], '--','LineWidth',1.5,'Color',[.99 .99 .99])

    
    % Draw scheduled trajectories
    if t > sim1_schedulingTime && t < sim1_exitTime
        ns_color = 'green';
    else
        ns_color = 0.5*[1 1 1];
    end
    
    if t > sim2_schedulingTime && t < sim2_exitTime
        ws_color = 'yellow';
    else
        ws_color = 0.5*[1 1 1];
    end
    
    if t > sim3_schedulingTime && t < sim1_exitTime
        ss_color = 'green';
    else
        ss_color = 0.5*[1 1 1];
    end
    
    if t > sim4_schedulingTime && t < sim2_exitTime
        es_color = 'yellow';
    else
        es_color = 0.5*[1 1 1];
    end
        
    quiver(NS.start(1), NS.start(2), NS.traj(1), NS.traj(2), 0, 'Color', ns_color, 'LineWidth', 2, 'MaxHeadSize', 0.5)
    quiver(SS.start(1), SS.start(2), SS.traj(1), SS.traj(2), 0, 'Color', ss_color, 'LineWidth', 2, 'MaxHeadSize', 0.5)
    quiver(WS.start(1), WS.start(2), WS.traj(1), WS.traj(2), 0, 'Color', ws_color, 'LineWidth', 2, 'MaxHeadSize', 0.5)
    quiver(ES.start(1), ES.start(2), ES.traj(1), ES.traj(2), 0, 'Color', es_color, 'LineWidth', 2, 'MaxHeadSize', 0.5)
    
    % Draw vehicles
    % Red:    Unscheduled
    % Yellow: Scheduled in a future timeslot
    % Green:  Scheduled in current timeslot    
    L = 4.8;
    W = 1.9;

    if t > sim1_schedulingTime && t < sim1_exitTime
        color1 = 'green';
    else
        color1 = 'red';
    end
    
    if t > sim2_schedulingTime && t < sim2_exitTime
        color2 = 'yellow';
    else
        color2 = 'red';
    end
    
    if t > sim3_schedulingTime && t < sim1_exitTime
        color3 = 'green';
    else
        color3 = 'red';
    end
    
    if t > sim4_schedulingTime && t < sim2_exitTime
        color4 = 'yellow';
    else
        color4 = 'red';
    end

    rectangle('Position',[xx1*scale_factor-L,yy1-W/2,L, W],'LineWidth',2,'FaceColor',color1)
    rectangle('Position',[xx2-W/2,yy2*scale_factor, W,L]  ,'LineWidth',2,'FaceColor',color2)
    rectangle('Position',[xx3*scale_factor+L,yy3-W/2,L, W],'LineWidth',2,'FaceColor',color3)
    rectangle('Position',[xx4-W/2,yy4*scale_factor-L, W,L],'LineWidth',2,'FaceColor',color4)

    % Legend and annotations
    delete(findall(gcf,'type','annotation'))
    
    annotation('rectangle',[.654 .885 .008 .008],'FaceColor','red')
    annotation('rectangle',[.654 .868 .008 .008],'FaceColor','green')
    annotation('rectangle',[.654 .851 .008 .008],'FaceColor','yellow')
    dim = [.65 .6 .9 .3];
    annotation('textbox',dim,'String',{'    Unscheduled';'    Scheduled in SLOT1';'    Scheduled in SLOT2'},'FitBoxToText','on', 'FontSize', 11);
    
    t_str = sprintf('Time = %0.2f s',t);
    annotation('textbox',[0.2 0.6 0.3 0.3],'String',t_str,'FitBoxToText','on', 'FontSize', 11);
    
    if t > sim1_entryTime && t < sim1_exitTime
        annotation('textbox',[0.2 0.55 0.3 0.3],'String','SLOT 1 ACTIVE','FitBoxToText','on', 'FontSize', 11, 'BackgroundColor', 'green');
    elseif t > sim2_entryTime && t < sim2_exitTime
        annotation('textbox',[0.2 0.55 0.3 0.3],'String','SLOT 2 ACTIVE','FitBoxToText','on', 'FontSize', 11, 'BackgroundColor', 'yellow');
    end
    
    ylim([-50 50])
    axis equal
    xlim([-70 70])
    axis off
    
    hold off
    
%     frame = getframe(gcf);
    set(fh2,'GraphicsSmoothing','on')
    print(fh2,'video_image.png','-dpng')
    img = imread('video_image.png');
    writeVideo(writerObj, img);
 
end
hold off
close(writerObj); % Saves the movie.

%% Set up the movie.
writerObj = VideoWriter('VT.avi'); % Name it.
writerObj.FrameRate = 25; % How many frames per second.
open(writerObj); 

fh1 = figure('units','pixels','position',[0 0 1920 1080]);
 
flag = false;
 
for t = (t_start+0.2):(1/writerObj.FrameRate):(t_end-7) 
    
    t1 = real_speed.SampleTimeStamp+real_offset;
    ii1 = t1 >= t_start & t1 <= t;
    t1_temp = t1(ii1);
    t1_last = t1_temp(end);
    ii1_vt = find(real_tunerState.SampleTimeStamp+real_offset<=t1_last,1,'last');
    if real_tunerState.t1(ii1_vt)~=0
        flag = true;
        vecT = real_tunerState.SampleTimeStamp(ii1_vt) + real_offset +...
            [0, real_tunerState.t1(ii1_vt),real_tunerState.t2(ii1_vt),real_tunerState.t3(ii1_vt),sim2_entryTime];
        vecV = [real_tunerState.v1(ii1_vt),real_tunerState.v2(ii1_vt),real_tunerState.v3(ii1_vt),real_tunerState.v4(ii1_vt),real_tunerState.v4(ii1_vt)];
    end
    t2 = sim2_state.SampleTimeStamp-sim1_state.SampleTimeStamp(1);
    ii2 = t2 >= t_start & t2 <= t;
    t3 = sim3_state.SampleTimeStamp-sim1_state.SampleTimeStamp(1);
    ii3 = t3 >= t_start & t3 <= t;
    t4 = sim4_state.SampleTimeStamp-sim1_state.SampleTimeStamp(1);
    ii4 = t4 >= t_start & t4 <= t;
    
    figure(fh1)
    plot(t1(ii1),real_speed.groundSpeed(ii1)*3.6,'b-')
    hold on
    if flag == true
        plot(vecT,vecV*3.6,'or--')
    end
%     plot(t2(ii2)-t,sim2_state.velocityX(ii2)*3.6,'r-')
%     plot(t3(ii3)-t,sim3_state.velocityX(ii3)*3.6,'b--')
%     plot(t4(ii4)-t,sim4_state.velocityX(ii4)*3.6,'r--')
    xlim([t_start t_end])
    ylim([0 50])
    set(findall(gca, 'Type', 'Line'),'LineWidth',2);
    xlabel('Time [s]')
    ylabel('Velocity [km/h]')
    delete(findall(gcf,'type','annotation'))
    iil = find(real_locOnPath.SampleTimeStamp+real_offset<=t1_last,1,'last');
    t_str = sprintf('Distance to the intersection, S = %0.2f m',real_locOnPath.intersectionLocation(iil)-real_locOnPath.currentLocation(iil));
    annotation('textbox',[0.2 0.6 0.3 0.3],'String',t_str,'FitBoxToText','on', 'FontSize', 11);
    
    % Timeslot
    rectangle('Position',[sim1_entryTime,-10,sim1_exitTime-sim1_entryTime,100],'FaceColor',[0 0 1 0.1],'EdgeColor','k','LineWidth',1)
    rectangle('Position',[sim2_entryTime,-10,sim2_exitTime-sim2_entryTime,100],'FaceColor',[1 0 0 0.1],'EdgeColor','k','LineWidth',1)
    
    hold off
%     
    set(fh1,'GraphicsSmoothing','on')
    print(fh1,'video_image.png','-dpng')
    img = imread('video_image.png');
    writeVideo(writerObj, img);
%     frame = getframe(gcf);
%     writeVideo(writerObj, frame);
 
end
hold off
close(writerObj); % Saves the movie.
