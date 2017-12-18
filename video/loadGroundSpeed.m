function data = loadGroundSpeed( filename )

delimiter = ';';
startRow = 2;
formatSpec = '%f%f%f%f%[^\n\r]';
fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType', 'string', 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
fclose(fileID);
data = table(dataArray{1:end-1}, 'VariableNames', {'SampleTimeStamp','SentTimeStamp','ReceivedTimeStamp','groundSpeed'});