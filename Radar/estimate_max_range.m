%Operating frequency (Hz)
fc = 77.0e9;

%Transmitted power (W)
Ps = 3e-3;

%Antenna Gain (linear)
G =  10000;

%Minimum Detectable Power
Pe = 1e-10;

%RCS of a car
RCS = 100;

%Speed of light
c = 3*10^8;

%TODO: Calculate the wavelength
w = c / fc;

%TODO : Measure the Maximum Range a Radar can see. 
range = ((Ps * G^2 * w^2 * RCS) / ((4*pi)^3 * Pe))^(1/4);

disp(range)
