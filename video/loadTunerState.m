function data = loadTunerState( filename )

delimiter = ';';
startRow = 2;
formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';
fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType', 'string', 'HeaderLines', startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
fclose(fileID);
data = table(dataArray{1:end-1}, 'VariableNames', {'SampleTimeStamp','SentTimeStamp','ReceivedTimeStamp','t1','t2','t3','t4','v1','v2','v3','v4','t','s','scal','commandAcc','commandAcc2','commandV','commandV2','acctime','mindistance','maxdistance','averagedistance','desireda','mm'});