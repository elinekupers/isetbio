function [interpFilters, meanCur] = computeCurrent(obj, varargin)
%COMPUTECURRENT Convert absorptions to photocurrent using the os model.
%   [interpFilters, meanCur] = COMPUTECURRENT(obj, varargin) 
%
%   Input:
%   obj - A coneMosaic object.
%
%   Output:
%   interpFilters - The linear filters are computed from the biophys model.
%       They are interpolated to the time samples of the cone mosaic.  The
%       interpolated filters are provided here.  To get the 1 ms values,
%       use osLinear.linearFilters.
%   meanCur - Sometimes we need the mean current as well.
%
% See also CONEMOSAIC, COMPUTE, COMPUTEFOROISEQUENCE.

% HJ ISETBIO Team 2016

%% parse inputs
p = inputParser;
p.addParameter('absorptionsInXWFormat', [], @isnumeric);
p.addParameter('bgR',[],@isnumeric);
p.KeepUnmatched = true;
p.parse(varargin{:});
bgR = p.Results.bgR;

% Check that absorption time series has been computed
if (isempty(obj.absorptions)  || size(obj.absorptions,3) == 1) && (isempty(p.Results.absorptionsInXWFormat))
    error('You must compute isomerizations (absorptions) time series prior to the current.');
end

% This is the background absorption rate.  We pass it in to 'warm up' the
% biophysical model to reach steady state faster.  It is also used by the
% linear os model to obtain the needed filters.

% if background absorption rate is already computed and defined as input
% variable, use that bgR. If not, compute it on the spot..
if isempty(bgR)
    bgR = coneMeanIsomerizations(obj, 'absorptionsInXWFormat', p.Results.absorptionsInXWFormat);
end
%% Call the appropriate outer segment photocurrent computation
if isa(obj.os,'osLinear')
    [obj.current, interpFilters, meanCur] = obj.os.osCompute(obj,'bgR',mean(bgR),varargin{:});
elseif isa(obj.os,'osBioPhys')
    obj.current = obj.os.osCompute(obj,'bgR',bgR); % bgR is already a mean, or just one cone class so warm up the biophys model.
    interpFilters = [];
    meanCur       = [];
else
    error('Attempting to computer current with unsupported os class');
end


end