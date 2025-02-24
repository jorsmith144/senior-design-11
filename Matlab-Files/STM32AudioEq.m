clearvars;
close all;
clc;


% Load Audio Signal
[x, Fs] = audioread('MH3U.mp3');        % Add your file in audioread()
x = x(:, 1); 
Ts = 1/(Fs);                            % Sample Time

% Display Basic Information
disp(['Sampling frequency: ', num2str(Fs), ' Hz']);
disp(['Signal length: ', num2str(length(x)), ' samples']);


%------------------- USER CONTROL VARIABLES -----------------------

% High Cut / Low Pass Filter Settings
f0_LPF = 5000;                          % Center / Corner Frequency
Q_LPF = 1;                              % Quality Factor
BW_LPF = 1;                             % Bandwidth in Octaves

% Low Cut / High Pass Filter Settings
f0_HPF = 20;                            % Center / Corner Frequency
Q_HPF = 1;                              % Quality Factor
BW_HPF = 1;                             % Bandwidth in Octaves

% Notch Filter Settings
f0_NCH1 = 100;                          % Center / Corner Frequency
Q_NCH1 = 1;                             % Quality Factor
BW_NCH1 = 1;                            % Bandwidth in Octaves

% Peaking Filter
f0_PKG = 100;                           % Center / Corner Frequency
Q_PKG = 1;                              % Quality Factor
BW_PKG = 1;                             % Bandwidth in Octaves
dB_gain_PKG = 1;                        % Gain in dB

% REVERB SETTINGS
wet = 0.5;                               % 0.0 -> 1.0
delay = 0.50;                            % 0.0 -> 1.0
delay_scale = 2;                         % Increases Max Delay Integers Only


%------------------- USER CONTROL VARIABLES END ------------------



% LPF Coefficients and Parameters

w0_LPF = 2 * pi * (f0_LPF / Fs);    % Cutoff Freq in radians
cos_w0_LPF = cos(w0_LPF);           % cosine omega
sin_w0_LPF = sin(w0_LPF);           % sine omega
alpha_LPF = sin_w0_LPF / (2 * Q_LPF);       


b0_LPF = (1 - cos_w0_LPF) / 2;
b1_LPF = (1 - cos_w0_LPF);
b2_LPF = (1 - cos_w0_LPF) / 2;
a0_LPF = 1 + alpha_LPF;
a1_LPF = -2 * cos_w0_LPF;
a2_LPF = 1 - alpha_LPF;

norm_b0_LPF = b0_LPF / a0_LPF;
norm_b1_LPF = b1_LPF / a0_LPF;
norm_b2_LPF = b2_LPF / a0_LPF;
norm_a1_LPF = a1_LPF / a0_LPF;
norm_a2_LPF = a2_LPF / a0_LPF;



% HPF Coefficients and Parameters

w0_HPF = 2 * pi * (f0_HPF / Fs);        % Cutoff Freq in radians
cos_w0_HPF = cos(w0_HPF);               % cosine omega
sin_w0_HPF = sin(w0_HPF);               % sine omega
alpha_HPF = sin_w0_HPF / (2 * Q_HPF);

b0_HPF = (1 + cos_w0_HPF) / 2;
b1_HPF = -(1 + cos_w0_HPF);
b2_HPF = (1 + cos_w0_HPF) / 2;
a0_HPF = 1 + alpha_HPF;
a1_HPF = -2 * cos_w0_HPF;
a2_HPF = 1 - alpha_HPF;

norm_b0_HPF = b0_HPF / a0_HPF;
norm_b1_HPF = b1_HPF / a0_HPF;
norm_b2_HPF = b2_HPF / a0_HPF;
norm_a1_HPF = a1_HPF / a0_HPF;
norm_a2_HPF = a2_HPF / a0_HPF;



% Notch Coefficients and Parameters

w0_NCH1 = 2 * pi * (f0_NCH1 / Fs);      % Center Freq in radians
cos_w0_NCH1 = cos(w0_NCH1);             % cosine omega
sin_w0_NCH1 = sin(w0_NCH1);             % sine omega
alpha_NCH1 = sin_w0_NCH1 / (2 * Q_NCH1);

b0_NCH1 = 1;
b1_NCH1 = -2 * cos_w0_NCH1;
b2_NCH1 = 1;
a0_NCH1 = 1 + alpha_NCH1;
a1_NCH1 = -2 * cos_w0_NCH1;
a2_NCH1 = 1 - alpha_NCH1;

norm_b0_NCH1 = b0_NCH1 / a0_NCH1;
norm_b1_NCH1 = b1_NCH1 / a0_NCH1;
norm_b2_NCH1 = b2_NCH1 / a0_NCH1;
norm_a1_NCH1 = a1_NCH1 / a0_NCH1;
norm_a2_NCH1 = a2_NCH1 / a0_NCH1;



% Peaking EQ Coefficients and Parameters

w0_PKG = 2 * pi * (f0_PKG / Fs);    % Center Freq in radians
A_PKG = 10^(dB_gain_PKG / 40);          % Linear Gain from dB
cos_w0_PKG = cos(w0_PKG);      % cosine omega
sin_w0_PKG = sin(w0_PKG);      % sine omega
alpha_PKG = sin_w0_PKG / (2 * Q_PKG);

% Case BW
%alpha_PKG = sin(w0_PKG) * sinh((log(2)/2) * BW_PKG * (w0_PKG / sin(w0_PKG)));


b0_PKG = 1 + alpha_PKG * A_PKG;
b1_PKG = -2 * cos_w0_PKG;
b2_PKG = 1 - alpha_PKG * A_PKG;
a0_PKG = 1 + ( alpha_PKG / A_PKG );
a1_PKG = -2 * cos_w0_PKG;
a2_PKG = 1 - ( alpha_PKG / A_PKG );

norm_b0_PKG = b0_PKG / a0_PKG;
norm_b1_PKG = b1_PKG / a0_PKG;
norm_b2_PKG = b2_PKG / a0_PKG;
norm_a1_PKG = a1_PKG / a0_PKG;
norm_a2_PKG = a2_PKG / a0_PKG;





% Workspace Setup

time = length(x);
t = 0:time - 1;   % Time Vector
t = t * Ts;

% output audio files
h1 = zeros(1, length(t));    % Filtered Output
h2 = zeros(1, length(t));    % Filtered Output
h3 = zeros(1, length(t));    % Filtered Output

% Reverb Output
y = zeros(1, length(t));

% EQ Output
z = zeros(1, length(t));

% Comb Output File
h = zeros(1, length(t));



% DIGITAL AUDIO EQ FILTERS -------------------------------------------------

for n =3 : length(x)

% Biquad Low Pass Filter
    h1(n) =  norm_b0_LPF * x(n) + norm_b1_LPF * x(n-1) + norm_b2_LPF * x(n-2) - norm_a1_LPF * h1(n-1) - norm_a2_LPF * h1(n-2);

% Biquad Notch Filter
    h2(n) =  norm_b0_NCH1 * h1(n) + norm_b1_NCH1 * h1(n-1) + norm_b2_NCH1 * h1(n-2) - norm_a1_NCH1 * h2(n-1) - norm_a2_NCH1 * h2(n-2);


% Biquad Peak Filter 
    h3(n) =  norm_b0_PKG * h2(n) + norm_b1_PKG * h2(n-1) + norm_b2_PKG * h2(n-2) - norm_a1_PKG * h3(n-1) - norm_a2_PKG * h3(n-2);


% Biquad High Pass Filter
    z(n) =  norm_b0_HPF * h3(n) + norm_b1_HPF * h3(n-1) + norm_b2_HPF * h3(n-2) - norm_a1_HPF * z(n-1) - norm_a2_HPF * z(n-2);

end



% DIGITAL AUDIO EQ FILTERS END ---------------------------------------------

% PLOT TRANSFER FUNCTION


%b0 = norm_b0_PKG;
%b1 = norm_b1_PKG;
%b2 = norm_b2_PKG;
%a1 = norm_a1_PKG;
%a2 = norm_a2_PKG;

%figure;
%tz = tf('z', Ts);
%PKG = (b0 + b1 * tz^-1 + b2 * tz^-2) / (1 + a1 * tz^-1 + a2 * tz^-2);
%opts = bodeoptions;
%opts.FreqUnits = 'Hz'; % Set frequency units to Hertz
%bodeplot(PKG, opts);







% Reverb Filter Setup

% Comb Filters
% Gains must be less than 1, 
% delay in milliseconds




% Delay Calculated: Delay / Sample Time = Sample Buffer Length

Cdelay_1 = 1590 * delay_scale;
Cgain_1 = 0.805;

Cdelay_2 = 1372 * delay_scale;
Cgain_2 = 0.827;

Cdelay_3 = 1782 * delay_scale; 
Cgain_3 = 0.783;

Cdelay_4 = 1980 * delay_scale;
Cgain_4 = 0.764;

Cgain_sum = 0.25;

% All Pass Filter Parameters

Adelay_1 = 220;
Again_1 = 0.7;

Adelay_2 = 74;
Again_2 = 0.7;

Adelay_3 = 21;
Again_3 = 0.7;


% Initialize Filter Buffers

Cbuff_1 = zeros(1, Cdelay_1); 
Cbuff_2 = zeros(1, Cdelay_2); 
Cbuff_3 = zeros(1,Cdelay_3); 
Cbuff_4 = zeros(1, Cdelay_4); 

Abuff_1 = zeros(1, Adelay_1); 
Abuff_2 = zeros(1, Adelay_2); 
Abuff_3 = zeros(1, Adelay_3); 

% intermediate filter outputs;

Cout_1 = 0;
Cout_2 = 0;
Cout_3 = 0;
Cout_4 = 0;

Csum = 0;

Aout_1 = 0;
Aout_2 = 0;
Aout_3 = 0;

Clim_1 = round(Cdelay_1 * delay);
Clim_2 = Cdelay_2 * delay; 
Clim_3 = Cdelay_3 * delay;
Clim_4 = Cdelay_4 * delay;

Alim_1 = Adelay_1 * delay;
Alim_2 = Adelay_2 * delay;
Alim_3 = Adelay_3 * delay;


CIndex_1 = 1;
CIndex_2 = 1;
CIndex_3 = 1;
CIndex_4 = 1;

AIndex_1 = 1;
AIndex_2 = 1;
AIndex_3 = 1;

% Reverb Signal Path
for n = 3 : length(z)

    % Comb Filter 1

    Cout_1 = Cbuff_1(CIndex_1) ; % Index Indicates out Sample, Output then send end
    Cbuff_1(CIndex_1) = z(n) + Cout_1 * Cgain_1; % Delay Input
    CIndex_1 = CIndex_1 + 1; 
    if CIndex_1 > Cdelay_1
        CIndex_1 = 1;
    end

    % Comb Filter 2

    Cout_2 = Cbuff_2(CIndex_2) ; % Index Indicates out Sample, Output then send end
    Cbuff_2(CIndex_2) = z(n) + Cout_2 * Cgain_2; % Delay Input
    CIndex_2 = CIndex_2 + 1; 
    if CIndex_2 > Cdelay_2
        CIndex_2 = 1;
    end

    % Comb Filter 3

    Cout_3 = Cbuff_3(CIndex_3) ; % Index Indicates out Sample, Output then send end
    Cbuff_3(CIndex_3) = z(n) + Cout_3 * Cgain_3; % Delay Input
    CIndex_3 = CIndex_3 + 1; 
    if CIndex_3 > Cdelay_3
        CIndex_3 = 1;
    end
    
    % Comb Filter 4

    Cout_4 = Cbuff_4(CIndex_4) ; % Index Indicates out Sample, Output then send end
    Cbuff_4(CIndex_4) = z(n) + Cout_4 * Cgain_4; % Delay Input
    CIndex_4 = CIndex_4 + 1; 
    if CIndex_4 > Cdelay_4
        CIndex_4 = 1;
    end


    % Input to the all pass Filter
    Csum = (Cout_1 + Cout_2 + Cout_3 + Cout_4) * 0.25;

    h(n) = Csum;
    
    % All-Pass Filter 1
    % Csum is the Input

    Aout_1 = Abuff_1(AIndex_1) - Csum * Again_1;
    Abuff_1 = Csum + Aout_1 * Again_1; 
    if AIndex_1> Adelay_1
        AIndex_1 = 1;
    end


    % All-Pass Filter 2
    % Aout_1 is the Input

    Aout_2 = Abuff_2(AIndex_2) - Aout_1 * Again_2;
    Abuff_2 = Aout_1 + Aout_2 * Again_2; 
    if AIndex_2 > Adelay_2
        AIndex_2 = 1;
    end

    % All-Pass Filter 3
    % Aout_1 is the Input

    Aout_3 = Abuff_3(AIndex_3) - Aout_2 * Again_3;
    Abuff_3 = Aout_2 + Aout_3 * Again_3; 
    if AIndex_3 > Adelay_3
        AIndex_3 = 1;
    end


    

    % Reverb Output

    y(n) = z(n) * (1 - wet) + Aout_3 * wet;
    

end 



figure;
subplot(3, 1, 1);
plot(t, x, 'blue');
title('Input Signal')
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

subplot(3, 1, 2);
plot(t, z, 'blue');
title('Filtered Signal')
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

subplot(3, 1, 3);
plot(t, y, 'blue');
title('Filtered Signal')
xlabel('Time (s)');
ylabel('Amplitude');
grid on;



% Playback the Original signal
%sound(x, Fs);
%pause(length(x) / Fs + 1);

% Playback the Filtered signal
sound(z, Fs);
pause(length(z) / Fs + 1);


% Playback the Reverb signal
%sound(y, Fs);
%pause(length(y) / Fs + 1);


