% Comb Filters in Parallel
% Scale By 0.25
% 3 all pass filters
% wet + original
% audio out

clearvars;
close all;
clc;


% REVERB SETTINGS

wet = 0.0; % 0.0 -> 1.0
delay = 1.0; % 0.0 -> 1.0
delay_scale = 1; %increases ram usage with higher max delay

% Load the audio signal
[x, fs] = audioread('MH3U.mp3'); % 44100 KHZ AUDIO SAMPLE mp3 or wav
x = x(:, 1); 

% Workspace Setup

fs_Hz = 44100; % Sample frequency Hz
T_s = 1/(fs_Hz); % Sample Time

time = length(x);
t = 0:time - 1;   % Time Vector
t = t * T_s;



% output audio file
y = 0:length(x); 
y = zeros(1, length(t));    % Filtered Output

% Comb Output File
h = 0:length(x); 
h = zeros(1, length(t));    % Filtered Output

% Display basic information
disp(['Sampling frequency: ', num2str(fs), ' Hz']);
disp(['Signal length: ', num2str(length(x)), ' samples']);


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

Clim_1 = Cdelay_1 * delay;
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
for n = 3 : length(x)

    % Comb Filter 1

    Cout_1 = Cbuff_1(CIndex_1) ; % Index Indicates out Sample, Output then send end
    Cbuff_1(CIndex_1) = x(n) + Cout_1 * Cgain_1; % Delay Input
    CIndex_1 = CIndex_1 + 1; 
    if CIndex_1 > Cdelay_1
        CIndex_1 = 1;
    end

    % Comb Filter 2

    Cout_2 = Cbuff_2(CIndex_2) ; % Index Indicates out Sample, Output then send end
    Cbuff_2(CIndex_2) = x(n) + Cout_2 * Cgain_2; % Delay Input
    CIndex_2 = CIndex_2 + 1; 
    if CIndex_2 > Cdelay_2
        CIndex_2 = 1;
    end

    % Comb Filter 3

    Cout_3 = Cbuff_3(CIndex_3) ; % Index Indicates out Sample, Output then send end
    Cbuff_3(CIndex_3) = x(n) + Cout_3 * Cgain_3; % Delay Input
    CIndex_3 = CIndex_3 + 1; 
    if CIndex_3 > Cdelay_3
        CIndex_3 = 1;
    end
    
    % Comb Filter 4

    Cout_4 = Cbuff_4(CIndex_4) ; % Index Indicates out Sample, Output then send end
    Cbuff_4(CIndex_4) = x(n) + Cout_4 * Cgain_4; % Delay Input
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

    y(n) = x(n) * (1 - wet) + Aout_3 * wet;
    

end 

figure;

subplot(3,1,1);
plot(t, x);
title('Input Signal');
xlabel('Time (s)');
ylabel('Amplitude');

subplot(3,1,2);
plot(t, h);
title('Comb Filter Output');
xlabel('Time (s)');
ylabel('Amplitude');

subplot(3,1,3);
plot(t, y);
title('Reverb Output');
xlabel('Time (s)');
ylabel('Amplitude');


% Playback the original signal
%sound(x, fs);
%pause(length(x) / fs + 1);

% Playback the Reverb signal
sound(y, fs);
pause(length(y) / fs + 1);