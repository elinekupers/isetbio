function [rgcL, nTrialsSpikes] = compute(rgcL, bp, varargin)
% @RGCLAYER.COMPUTE - Computes the rgc mosaic responses to an input
%
%   ir = @rgcLayer.compute(ir, bipolar, varargin)
%
% Required inputs
%  'rgcL' -    rgc layer object 
%  'bipolar' - bipolar mosaic object
%
% Optional inputs
%   'nTrialsSpikes' -
%
% Computes the continuous (linear) and spike responses for each of the
% mosaics within the inner retina object.  (Note: There are no spikes
% for the rgcLinear class).
% 
% For each mosaic, a space-time separable linear response is computed. This
% stage of the computation is stored in 'responseLinear'.  This is managed
% in irComputeLinearSTSeparable.  There is no noise added in the linear
% part.
%
% At present, the temporal response is set to an impulse because of the way
% the bipolar tIR is set.  We need to deal with this.  Also, the 'dt' of
% the RGC should be inherited form the dt of the bipolar calculation.  I
% don't see that anywhere (BW).  Should be in the
% irComputeLinearSTSeparable routine.
%
% The spikes are computed irComputeSpikes routine. The spiking can have a
% random element.  So, we may run the conversion from linear to spikes
% multiple times, effectively producing spike rasters.
%
% Outputs:
%  ir: the inner retina object with responses attached to each mosaic
%  nTrialsSpikes:  (trials x xPos x yPos x Time)
%     Binary matrix indicating the spike times for all the trials.
%     The last one is stored in the mosaics of the inner retina object in
%     the responseSpikes slot.
%
% Example:
%   rgcL.compute(bipolarMosaic??);
%
% See also: rgcMosaic
%
% BW (c) isetbio team

%% Parse inputs
p = inputParser;
p.CaseSensitive = false;

p.addRequired('rgcL',@(x) ~isempty(validatestring(class(x),{'rgcLayer'})));
vFunc = @(x) (isequal(class(x),'bipolarMosaic')||isequal(class(x{1}),'bipolarMosaic'));
p.addRequired('bp',vFunc);

p.addParameter('coupling',false,@islogical);
p.addParameter('bipolarTrials',  [], @(x) isnumeric(x)||iscell(x));  % Multiple bipolar trials
p.addParameter('bipolarScale',50,@isnumeric);
p.addParameter('bipolarContrast',1,@isnumeric);

p.parse(rgcL,bp,varargin{:});
coupling = p.Results.coupling;
bipolarTrials = p.Results.bipolarTrials;

% See notes below
bipolarScale    = p.Results.bipolarScale; 
bipolarContrast = p.Results.bipolarContrast; 

%% Linear stage of the computation

% We set the bipolar scale factor in order to produce a bipolar model that
% generates reasonable RGC spikes for typical viewing conditions.  The sad
% truth is that we don't have a biophysically accurate model of the bipolar
% cells.  Consequently, we have no match for the bipolar current with real
% units.  This is a fudge factor that produces attractive RGC spike rates.
% When we get more information about the bipolar models, we hope to do
% better.
% Similarly, the internal calculation converts bipolar current to a
% contrast with a max value of 1.  We can control the max contrast here.
% The code is not set up vary these parameters yet.  We will expose them
% some day.
if ~isempty(bipolarTrials)  
    [rgcL,nTrialsLinearResponse] = rgcL.computeSeparable(bp, ...
        'bipolarScale', bipolarScale,...
        'bipolarContrast',bipolarContrast,...
        'bipolarTrials',bipolarTrials);
else
    rgcL = rgcL.computeSeparable(bp, ...
        'bipolarContrast',bipolarContrast,...
        'bipolarScale', bipolarScale);
end
% rgcL.plot('response linear');

%% Compute spikes from linear response; possibly for multiple trials

% This should be for ii=1:length(ir.mosaic)
switch class(rgcL.mosaic{1})
    case {'rgcLinear'}
        % No linear response implemented yet.
        disp('No spikes computed for linear RGC mosaic');   
    otherwise
        % Runs for rgcLNP, rgcGLM
        % Send the coupling field to decide on the coupling parameter
        if ~isempty(bipolarTrials) 
            % Multiple trial case
            [rgcL, nTrialsSpikes] = rgcL.computeSpikes('coupling',coupling, ...
                'nTrialsLinearResponse',nTrialsLinearResponse);
        else
            % Single trial case
            [rgcL, nTrialsSpikes] = rgcL.ComputeSpikes('coupling',coupling);
        end
end

end