% This script collects the data sent from the Arduino Mega.  It collects 
% the control loop frequency (Freq) in Hz, the duration of the test (Time) 
% in seconds, the reference input vector (R) in units dependent on the type
% of test, the output vector (Y) in radians and current (I) in amps. 

% Setup comport
comport = input('COM port that Mechsense is attached [i.e. COM1, COM2, ... etc.]: ','s');
arduino = serial(comport,'BaudRate',115200); 
fopen(arduino);

% Read in inital values
Freq = fscanf(arduino,'%f');      %Read in Control Loop Frequency in Hz
Time = fscanf(arduino,'%f');      %Read in Duration of Test in seconds
I_Gain = fscanf(arduino,'%f');    %Read in Input Filter gain
cnt_max = round(Freq*Time);       %Calculate total # of time steps
T = linspace(0,Time,cnt_max);     %Setup Time Vector

%Read in loop values
for i=1:cnt_max
   
  R(i)=fscanf(arduino,'%f');      %Read in reference input
  Y(i)=fscanf(arduino,'%f');      %Read in output in encoder counts
  I(i)=fscanf(arduino,'%f');      %Read in current in ADC counts (10-bit)
  
end
   
% Close comport
fclose(arduino);

disp('Data capture complete.')

% Convert I from ADC counts to Amps where 5 is the voltage range of the A 
% to D converter, 1023 is the max ADC value and 0.525 is the number of 
% Volts per Amp, as per the controller spec sheet. 
I = I*5/(1023*0.525);

% Convert Y from encoder counts to radians where I_Gain is the Input Filter 
% gain in encoder counts per radian.
Y = Y/I_Gain;

% Plot captured data

figure;
plot(T,R)
grid on
title('Reference Input vs. Time');
ylabel('Reference Input');
xlabel('Time (seconds)');

figure;
plot(T,Y)
grid on
title('Output vs. Time');
ylabel('Angular Position (radians)');
xlabel('Time (seconds)');

figure;
plot(T,I)
grid on
title('Current vs. Time');
ylabel('Current (amperes)');
xlabel('Time (seconds)');

disp('Plotting complete.')



