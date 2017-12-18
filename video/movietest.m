
%%
t_start = 12.0;
t_end = 40;

real_state = loadStateEstimate('real car/opendlv.logic.legacy.StateEstimate-0.csv');
sim1_state = loadStateEstimate('sim car1/opendlv.logic.legacy.StateEstimate-0.csv');
sim2_state = loadStateEstimate('sim car2/opendlv.logic.legacy.StateEstimate-0.csv');
sim3_state = loadStateEstimate('sim car3/opendlv.logic.legacy.StateEstimate-0.csv');
sim4_state = loadStateEstimate('sim car4/opendlv.logic.legacy.StateEstimate-0.csv');

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

figure;
plot(real_speed.SampleTimeStamp+real_offset,real_speed.groundSpeed*3.6,'b-')
hold on
% plot(sim1_state.SampleTimeStamp-sim1_state.SampleTimeStamp(1),sim1_state.velocityX*3.6)
plot(sim2_state.SampleTimeStamp-t_ref,sim2_state.velocityX*3.6,'r-')
plot(sim3_state.SampleTimeStamp-t_ref,sim3_state.velocityX*3.6,'b--')
plot(sim4_state.SampleTimeStamp-t_ref,sim4_state.velocityX*3.6,'r--')

% plot(real_entryTime+real_offset,40,'x')
rectangle('Position',[sim1_entryTime,-10,sim1_exitTime-sim1_entryTime,100],'FaceColor',[0 0 1 0.1],'EdgeColor','k','LineWidth',1)
rectangle('Position',[sim2_entryTime,-10,sim2_exitTime-sim2_entryTime,100],'FaceColor',[1 0 0 0.1],'EdgeColor','k','LineWidth',1)

xlim([t_start t_end])
ylim([0 50])

figure;
plot(real_state.positionX, real_state.positionY)
hold on
% plot(sim1_state.positionX, sim1_state.positionY)
plot(sim2_state.positionX, sim2_state.positionY)
plot(sim3_state.positionX, sim3_state.positionY)
plot(sim4_state.positionX, sim4_state.positionY)


%% Movie Test.

fid = figure;
 
%% Set up the movie.
writerObj = VideoWriter('out.avi'); % Name it.
writerObj.FrameRate = 25; % How many frames per second.
open(writerObj); 
 
for t = t_start:(1/writerObj.FrameRate):t_end
    
    t1 = real_speed.SampleTimeStamp+real_offset;
    ii1 = t1 >= t_start & t1 <= t;
    t2 = sim2_state.SampleTimeStamp-sim1_state.SampleTimeStamp(1);
    ii2 = t2 >= t_start & t2 <= t;
    t3 = sim3_state.SampleTimeStamp-sim1_state.SampleTimeStamp(1);
    ii3 = t3 >= t_start & t3 <= t;
    t4 = sim4_state.SampleTimeStamp-sim1_state.SampleTimeStamp(1);
    ii4 = t4 >= t_start & t4 <= t;
    
    plot(t1(ii1),real_speed.groundSpeed(ii1)*3.6,'b-')
    hold on
    plot(t2(ii2),sim2_state.velocityX(ii2)*3.6,'r-')
    plot(t3(ii3),sim3_state.velocityX(ii3)*3.6,'b--')
    plot(t4(ii4),sim4_state.velocityX(ii4)*3.6,'r--')
    xlim([t_start t_end])
    ylim([0 50])
    
    % Timeslot
    rectangle('Position',[sim1_entryTime,-10,sim1_exitTime-sim1_entryTime,100],'FaceColor',[0 0 1 0.1],'EdgeColor','k','LineWidth',1)
    rectangle('Position',[sim2_entryTime,-10,sim2_exitTime-sim2_entryTime,100],'FaceColor',[1 0 0 0.1],'EdgeColor','k','LineWidth',1)
    
    hold off
    
%     plot(stateEstimate.positionX(ii),stateEstimate.positionY(ii),'o')
%     axis([-200 200 -200 200])
    
    frame = getframe(gcf);
    writeVideo(writerObj, frame);
 
end
hold off
close(writerObj); % Saves the movie.