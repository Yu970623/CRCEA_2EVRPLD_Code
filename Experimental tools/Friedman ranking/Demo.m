%% drawNemenyi demo
% Farshid Sepehrband - Jeiran Choupan
% nemenyi.m function is borrowed from: http://kourentzes.com/forecasting/2013/04/19/nemenyi-test/

% Let's say we have a matrix of MxN, as the input:
% M measurements x N techniques

% First we define a cell of strings, including names of techniques. 
% for example: 
% Names = {'tech #1','tech #2',...,'tech #n'};

% Then, we define our output folder and output name and run the code.

% Here is an example, assuming you are comparing 4 techniques:
results =  xlsread('D:\OneDrive-CSU\OneDrive - csu.edu.cn\2E-VRP\Experiment\drawNemenyi-master\Data\Cus100.xlsx','Sheet1');
Names = {'LNS-2E', 'ALNS', 'EHG-HA', 'CRCEA'}; % names

OutputFolder  = 'D:\OneDrive-CSU\OneDrive - csu.edu.cn\2E-VRP\Experiment\drawNemenyi-master\Data';                       % Output folder
Outname = 'NemenyiResults';                        % Output name
drawNemenyi(results, Names, OutputFolder, Outname);

% Now you should see a "NemeyiResults.tif" on yoru Desktop folder.
% You can change the print options (see %%print section of drawNemenyi.m)
% Enjoy!

%  [1 2 3 4;
%   1 3 2 4;
%   2 2 3 4;
%   1 3 3 4;
%   1 2 2 3;
%   1 4 2 4]

