function [ settlingIndex ] = settlingIndexAnalysis( data, settlingPercent,  startingValue, endingValue)
%SETTLINGINDEXANALYSIS Find the point at which all future points are below
%the given value
%   Find the index of the point where all future datapoints are less than
%   a threshold value determined by the settling percentage and the
%   starting and ending value.
%
% Input:
%   data - Vector containing the dataset to analyze
%   settlingPercent - Decimal representation of the settling percent
%   startingValue - Starting value of the dataset
%   endingValue - Final value of the dataset
%
% Outputs:
%   settlingIndex - Index for when the dataset first stays below the
%                   threshold
%
%
% Created by: Ian McInerney
% Created on: September 21, 2016
% Version: 1.0
% Last Modified: September 21, 2013
%
% Revision History:
%   1.0 - Initial Release

%% Compute the settling threshold
thresholdValue = abs(startingValue - endingValue)*settlingPercent;

dataLen = length(data);

%% The following is not a really efficient search method, but it will handle
% the case of underdamped oscillations better than a binary search will

mStop = 0;
startIndex = 1;

% Loop over every datapoint, starting from the first one and see when the
% maximum of the data eventuall drops below the threshold
while (~mStop && (startIndex < dataLen) )
    [maximum, index] = max( data(startIndex:end) );
    
    if maximum < thresholdValue
        % If the largest value in the set is smaller than the threshold,
        % settling time is the starting index of the set
        mStop = 1;
        settlingIndex = startIndex;
    else
        % If the largest value in the set is larger than the threshold,
        % remove the next datapoint and run again
        startIndex = startIndex + 1;
    end
end

end

