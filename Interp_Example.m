% Author: Brian Roth
% Date: Sept 27, 2023
% Course: AE 420 - Aircraft Preliminary Design
% Purpose: Provide an example of 2-D interpolation for use with 
%   aerodynamic and propulsion data
%
% Matlab command:
%    Vq = interp2(X,Y,V,Xq,Yq) returns interpolated values of a function of 
%    two variables at specific query points using linear interpolation. 
%    The results always pass through the original sampling of the function. 
%    X and Y contain the coordinates of the sample points. V contains the 
%    corresponding function values at each sample point. Xq and Yq contain 
%    the coordinates of the query points.

close all
clear all
clc

CL = [-0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];  % lift coefficient
RE = [80, 100, 120];  % Reynolds number (in thousands) 
CDp = [0.01896, 0.01950, 0.01993, 0.02005, 0.01990, 0.01959, 0.01919, 0.01927, 0.01952, 0.02034, 0.02105, 0.02250;
       0.01697, 0.01733, 0.01730, 0.01701, 0.01649, 0.01581, 0.01555, 0.01581, 0.01647, 0.01734, 0.01837, 0.02007;
       0.01551, 0.01529, 0.01535, 0.01511, 0.01432, 0.01362, 0.01345, 0.01387, 0.01464, 0.01559, 0.01668, 0.01853];

% Suppose we are interested in drag when CL = 0.15 and RE = 110k
CLq = 0.15;
REq = 110;
CDq = interp2(CL,RE,CDp,CLq,REq)