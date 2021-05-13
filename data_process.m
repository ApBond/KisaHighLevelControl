clear,clc
s=readtable("coordinates_frame1.csv");
testdata = table2array(s);
testdata(:,1)=(testdata(:,1)-testdata(1,1))/1000;
subplot(2,2,1);
plot(testdata(:,4)/100,testdata(:,5)/100)
title('posE(posN),m')
grid on
subplot(2,2,2);
plot(testdata(:,2),testdata(:,3))
title('lon(lat),o')
grid on
subplot(2,2,4);
plot(testdata(:,1),testdata(:,8)/1000)
title('Vg(t),m/s')
grid on
subplot(2,2,3);
plot(testdata(:,1),testdata(:,9)/100000)
title('course(t),o')
grid on
%% Setup the Import Options
opts = spreadsheetImportOptions("NumVariables", 7);

% Specify sheet and range
opts.Sheet = "L1";
opts.DataRange = "A2:G67";

% Specify column names and types
opts.VariableNames = ["Index", "UTC", "relPosN", "relPosE", "VN", "VE", "CoG"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double"];

% Import the data
testdata = readtable("D:\Program Files (x86)\u-blox\u-center_v20.10\test_data(1).xlsx", opts, "UseExcel", false);

% Convert to output type
testdata = table2array(testdata);

% Clear temporary variables
clear  opts

% Show graphics
subplot(2,2,1);
plot(testdata(:,3),testdata(:,4))
title('posE(posN),m')
grid on
subplot(2,2,2);
plot(testdata(:,1)/4,testdata(:,5))
title('VN(t),m/s')
grid on
subplot(2,2,4);
plot(testdata(:,1)/4,testdata(:,6))
title('VE(t),m/s')
grid on
subplot(2,2,3);
plot(testdata(:,1)/4,testdata(:,7))
title('course(t),o')
grid on