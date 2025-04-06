%% Setup

clear;
close all;
clc;

% Load INS data
ins = load('ins.txt');
time_ins = ins(:,1);
theta = ins(:,2);
phi = ins(:,3);

% Load GPS data
gps = load('gps.txt');
time_gps = gps(:,1);
x_gps = gps(:,2);
y_gps = gps(:,3);

% Initialize constants
g = 9.81;
dt = 0.05;
n = length(time_ins);