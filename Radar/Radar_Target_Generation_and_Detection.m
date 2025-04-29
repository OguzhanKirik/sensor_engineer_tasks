clear all
clc;

%% Radar Specifications 
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Frequency of operation = 77GHz
% Max Range = 200m
% Range Resolution = 1 m
% Max Velocity = 100 m/s
%%%%%%%%%%%%%%%%%%%%%%%%%%%
freqOP = 77e9;
rangeMAx =200;
rangeRes = 1;
velMax = 70;
velRes = 3;
c = 3e8;
%speed of light = 3e8
%% User Defined Range and Velocity of target
% *%TODO* :
% define the target's initial position and velocity. Note : Velocity
% remains contant
targetRange = 167;
targetVel = -57;


%% FMCW Waveform Generation

% *%TODO* :
%Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of the FMCW
% chirp using the requirements above.
B_sweep = c / (2 * rangeRes);  % freq range in one chirp
T_Chirp = (5.5 * 2 * rangeMAx) / c; % time for 1 chirp
slope = B_sweep / T_Chirp;

%Operating carrier frequency of Radar 
freq_c= 77e9;             %carrier freq

                                                          
%The number of chirps in one sequence. Its ideal to have 2^ value for the ease of running the FFT
%for Doppler Estimation. 
Nd=128;                   % #of doppler cells OR #of sent periods % number of chirps

%The number of samples on each chirp. 
Nr=1024;                  %for length of time OR # of range cells

% Timestamp for running the displacement scenario for every sample on each
% chirp
t=linspace(0,Nd*T_Chirp,Nr*Nd); %total time for samples


%Creating the vectors for Tx, Rx and Mix based on the total samples input.
Tx=zeros(1,length(t)); %transmitted signal
Rx=zeros(1,length(t)); %received signal
Mix = zeros(1,length(t)); %beat signal

%Similar vectors for range_covered and time delay.
r_t=zeros(1,length(t));
td=zeros(1,length(t));


%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 

for i=1:length(t)         
    

    t_current = t(i); %current time
    % *%TODO* :
    %For each time stamp update the Range of the Target for constant velocity. 
    targetRange = targetRange + (T_Chirp/Nr) * targetVel;
    
    r_t(i) = targetRange;

    % *%TODO* :  
    %For each time sample we need update the transmitted and
    %received signal. 
    Tx(i) = cos(2*pi*(freq_c*t_current + (slope * t_current^2) / 2));
    
    timeTravel = 2 * targetRange / c;

    td(i) = timeTravel;
    
    Rx(i) = cos(2*pi*((freq_c*(t_current-timeTravel)) + ((slope * (t_current-timeTravel)^2) / 2)));
    
    % *%TODO* :
    %Now by mixing the Transmit and Receive generate the beat signal
    %This is done by element wise matrix multiplication of Transmit and
    %Receiver Signal
    %Mix(i) = cos(2*pi((2 * slope * R(i) / c) * t + (2 * freq_c * v * n/ c) * t));
    Mix(i) = Tx(i) .* Rx(i);
end

%% RANGE MEASUREMENT


 % *%TODO* :
%reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of
%Range and Doppler FFT respectively.
reshaped_Mix = reshape(Mix, [Nr, Nd]);



 % *%TODO* :
%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.
sig_fft1 = fft(reshaped_Mix, Nr)./Nr;


 % *%TODO* :
% Take the absolute value of FFT output
signal_fft = abs(sig_fft1);

 % *%TODO* :
% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.
%sampling_freq = 1000;
%f = sampling_freq*(0:Nr/2) / Nr;
P1  = signal_fft(1:Nr/2 +1);

%plotting the range
figure ('Name','Range from First FFT')
%subplot(2,1,1)

 % *%TODO* :
% plot FFT output 
plot(P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')
axis ([0 200 0 1]);



%% RANGE DOPPLER RESPONSE
% The 2D FFT implementation is already provided here. This will run a 2DFFT
% on the mixed signal (beat signal) output and generate a range doppler
% map.You will implement CFAR on the generated RDM


% Range Doppler Map Generation.

% The output of the 2D FFT is an image that has reponse in the range and
% doppler FFT bins. So, it is important to convert the axis from bin sizes
% to range and doppler based on their Max values.

Mix=reshape(Mix,[Nr,Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix,Nr,Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM) ;

%use the surf function to plot the output of 2DFFT and to show axis in both
%dimensions
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);
figure,surf(doppler_axis,range_axis,RDM);

%% CFAR implementation

%Slide Window through the complete Range Doppler Map

% *%TODO* :
%Select the number of Training Cells in both the dimensions.
Tr=8; % training cells in range dimension
Td=8; % training cells in doppler dimension

% *%TODO* :
%Select the number of Guard Cells in both dimensions around the Cell under 
%test (CUT) for accurate estimation
Gr=2;
Gd=2;

% *%TODO* :
% offset the threshold by SNR value in dB
offset = 5;
% *%TODO* :
%Create a vector to store noise_level for each iteration on training cells
noise_level = zeros(1,1);


% *%TODO* :
%design a loop such that it slides the CUT across range doppler map by
%giving margins at the edges for Training and Guard Cells.
%For every iteration sum the signal level within all the training
%cells. To sum convert the value from logarithmic to linear using db2pow
%function. Average the summed values for all of the training
%cells used. After averaging convert it back to logarithimic using pow2db.
%Further add the offset to it to determine the threshold. Next, compare the
%signal under CUT with this threshold. If the CUT level > threshold assign
%it a value of 1, else equate it to 0.


   % Use RDM[x,y] as the matrix from the output of 2D FFT for implementing
   % CFAR





% *%TODO* :
% The process above will generate a thresholded block, which is smaller 
%than the Range Doppler Map as the CUT cannot be located at the edges of
%matrix. Hence,few cells will not be thresholded. To keep the map size same
% set those values to 0. 
 % initilize an empty map for displaying
filtered_map = zeros(Nr/2, Nd);

% Loop through relevant cells (avoiding edges)
for i = (Tr+Gr+1) : (Nr/2 -Tr -Gr)
    for j = (Td+Gd+1) :  (Nd-Td-Gd)
        
        
        % Initialize noise level and training cell counter
         noise_level = 0;
         num_training_cells = 0;

        % Loop over the window around the CUT
         for r = (i - Tr-Gr) : (i+Gr+Tr)
             for c = (j-Td-Gd) : (j+Td+Gd)
                % Exclude guard cells and the CUT itself
                 if(abs(i-r) > Gr || abs(c-j) > Gd)
                    % Estimate average noise level (in power), apply offset
                     noise_level = noise_level + db2pow(RDM(r,c));
                     num_training_cells = num_training_cells + 1;

                 end

             end
         end
    
            

        %estimate averate of running noise level
        avg_noise_level = offset * (noise_level / num_training_cells);
        noise_threshold = pow2db(avg_noise_level);

        % Compare CUT against threshold
        signal_CUT = RDM(i,j);
        if(signal_CUT > noise_threshold)
            filtered_map(i,j) =1;
        end
    end
end




% *%TODO* :
%display the CFAR output using the Surf function like we did for Range
%Doppler Response output.
%figure,surf(doppler_axis,range_axis,'replace this with output');
%colorbar;

subplot(1,2, 2)
surf(doppler_axis,range_axis, filtered_map);
title("2D CFAR Filtering Appliedf");
xlabel("Doppler Axis [m/s]");
ylabel("Range Axis [m]");
zlabel("Intensity (Normalized)");
 
 