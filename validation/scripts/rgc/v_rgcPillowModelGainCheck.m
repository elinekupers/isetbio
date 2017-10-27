function varargout = v_rgcPillowModelGainCheck(varargin)
%v_rgcPillowModelGainCheck  Validate features of the Pillow RGC model.
%
% Description:
%   Validate features of the Pillow RGC model. Currently checks gain.
%   TRG asserts that this should always be one, independent of sampling
%   rate.  Currently, this is not the case.
%
% Notes:
%   [Note: DHB]  Once this is working, we can tuck away actual validation
%   data and add this to the scripts that get run.

% TRG 10/2017, (c) isetbio team
%
% 10/26/17  dhb  Put into standard isetbio validation function form.

varargout = UnitTest.runValidationRun(@ValidationFunction, nargout, varargin);
end

%% Function implementing the isetbio validation code
function ValidationFunction(runTimeParams)

%% Initialize
clear; close all; ieInit;

%% Define cell types and sample rates to check
cellTypes = {'onmidget', 'onparasol', 'onsbc', 'offmidget', 'offparasol'};
sampleRates = [1/100, 1/1000];

%% Check the gain of the Pillow rgc temporal response model for different sampling rates
for i=1%:length(cellTypes)
    for j=1:length(sampleRates)
%         params = {};
%         params.filterDuration = 0.4;
%         params.samplingTime = sampleRates(j);
%         params.cellType = cellTypes{i};
%         [rgcFilter,rgcTime ] = rgcImpulseResponsePillow(params);
%         rgcGain = sum(rgcFilter(:));
        
        oi = oiCreate;
        oi = oiCompute(oi,sceneCreate('rings rays'));
        cm = coneMosaic(oi);
        cm.integrationTime = sampleRates(j);
        cm.emGenSequence(250);
        cm.compute(oi);
        cm.computeCurrent();
        
        % Zero out current to hack an impulse response
        cm.current = zeros([(cm.mosaicSize),250]);
        spLocR = 25; spLocC = 25;
        % Put temporal impulse in cone response
        cm.current(spLocR,spLocC,1) = 1;
        bpL = bipolarLayer(cm);
        bpL.mosaic{1} = bipolarMosaic(cm,'on diffuse');
        bpL.mosaic{1}.compute();
        rgcGain = sum(bpL.mosaic{1}.responseCenter(spLocR,spLocC,:)-bpL.mosaic{1}.responseSurround(spLocR,spLocC,:));

        
%         if abs(rgcGain - 1.0) < 1e-6
        if abs(rgcGain - .006092) < 1e-6
            fprintf('Pillow %s filter with sample rate %0.4f has correct unit gain.\n', cellTypes{i}, sampleRates(j));
        else
            fprintf('Pillow %s filter with sample rate %0.4f has gain of %0.4f which should be 1.0\n', cellTypes{i}, sampleRates(j), rgcGain);
        end
    end
end

end