close all
clear all

WaveData = load('WaveData');

figure
plot(WaveData(:,1),WaveData(:,2));
grid on;
hold on;
plot(WaveData(:,1),WaveData(:,3));
plot(WaveData(:,1),WaveData(:,4));
plot(WaveData(:,1),WaveData(:,5));
plot(WaveData(:,1),WaveData(:,6));

legend('eta','min','max','mean','var');
