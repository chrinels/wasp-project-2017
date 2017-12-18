function stateEstimate = loadStateEstimate( filename )

    delimiter = ';';
    startRow = 2;
    formatSpec = '%f%f%f%f%f%f%f%f%f%[^\n\r]';
    fileID = fopen(filename,'r');
    dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
    fclose(fileID);
    stateEstimate = table(dataArray{1:end-1}, 'VariableNames', {'SampleTimeStamp','SentTimeStamp','ReceivedTimeStamp','positionX','positionY','velocityX','velocityY','orientation','yawRate'});


end

