filename = '/home/fs/wasp-project-2017/video/car1/opendlv.logic.legacy.StateEstimate-0.csv';
delimiter = ';';
startRow = 2;
formatSpec = '%f%f%f%f%f%f%f%f%f%[^\n\r]';
fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
fclose(fileID);
stateEstimate = table(dataArray{1:end-1}, 'VariableNames', {'SampleTimeStamp','SentTimeStamp','ReceivedTimeStamp','positionX','positionY','velocityX','velocityY','orientation','yawRate'});
clearvars filename delimiter startRow formatSpec fileID dataArray ans;

%% Movie Test.

fid = figure;
 
%% Set up the movie.
writerObj = VideoWriter('out.avi'); % Name it.
writerObj.FrameRate = 10; % How many frames per second.
open(writerObj); 
 
for ii=10:size(stateEstimate.SampleTimeStamp)      
    
%     plot(stateEstimate.po-stateEstimate.SampleTimeStamp(1),stateEstimate.velocityX(1:ii));
    plot(stateEstimate.positionX(ii),stateEstimate.positionY(ii),'o')
    axis([-200 200 -200 200])
    
    frame = getframe(gcf);
    writeVideo(writerObj, frame);
 
end
hold off
close(writerObj); % Saves the movie.