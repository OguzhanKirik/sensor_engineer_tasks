%Beat frequency → difference between transmitted and received frequencies.
%Beat sweep → how beat frequency changes during one chirp or across multiple chirps.
%The radar maximum range = 300m
rMaxRange = 300;
%The range resolution = 1m
rangeRes = 1;
%The speed of light c = 3*10^8
c = 3e8;


%beat Sweep
bSweep = c / (2 * rangeRes);

% the chirp time based on the Radar's Max Range​
timeChirp = (5.5 * 2 * rMaxRange) / c;


beat_freq = [0, 1.1e6, 13e6, 24e6]; % converting MHz to Hz

% the frequency shifts 
ranges = (c * timeChirp .* beat_freq) / (2 * bSweep);


disp(ranges)
 