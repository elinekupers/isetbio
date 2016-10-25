% t_oiSequence
%
% Demonstrate usage of oiSequence
%
%
% NPC, ISETBIO TEAM, 2016
%

% Generate a uniform scene
meanLuminance = 30;
uniformScene = sceneCreate('uniform equal photon', 128);
% square scene with desired FOV
FOV = 2.0;
uniformScene = sceneSet(uniformScene, 'wAngular', FOV);
% 1 meter away
uniformScene = sceneSet(uniformScene, 'distance', 1.0);
uniformScene = sceneAdjustLuminance(uniformScene, meanLuminance);

% Generate a gabor scene
gaborParams = struct(...
    'freq', 4, ...
    'contrast', 0.4, ...
    'ph', 0, ...
	'ang',  0, ...
    'row', 128, ...
    'col', 128, ...
	'GaborFlag', true);
gaborScene = sceneCreate('harmonic', gaborParams);
gaborScene = sceneSet(gaborScene, 'wAngular', FOV);
gaborScene = sceneSet(gaborScene, 'distance', 1.0);
gaborScene = sceneAdjustLuminance(gaborScene, meanLuminance);

% generating stimulus modulation functions
stimulusSamplingInterval = 60/1000;
oiTimeAxis = -0.6:stimulusSamplingInterval:0.6;
stimulusRampTau = 0.165;
% monophsaic modulation function
modulationFunction = exp(-0.5*(oiTimeAxis/stimulusRampTau).^2);
% biphasic modulation function
modulationFunction2 = exp(-0.5*((oiTimeAxis-0.2)/stimulusRampTau).^2) - exp(-0.5*((oiTimeAxis+0.2)/stimulusRampTau).^2);

% Default human optics
oi = oiCreate('human');
oi = oiCompute(oi, uniformScene);

% Compute the background and the modulated optical images
oiBackground = oiCompute(oi, uniformScene);
oiModulated  = oiBackground;
  
% Instantiate an oiSequence object for computing a sequence of full field ois
theOIsequence = oiSequence(oiBackground, oiModulated, modulationFunction);

% Instantiate another oiSequence object for computing a sequence of windowed (radius = 250 microns)ois
modulationRegion.radiusInMicrons = 250;
theOIsequence2 = oiSequence(oiBackground, oiModulated, modulationFunction, 'modulationRegion', modulationRegion);

% Instantiate another oiSequence object for computing windowed (radius = 250 microns) ois with biphasic modulation
modulationRegion.radiusInMicrons = 250;
theOIsequence3 = oiSequence(oiBackground, oiModulated, modulationFunction2, 'modulationRegion', modulationRegion);

% Instantiate another oiSequence object for computing windowed (radius = 250 microns) ois with biphasic modulation 
% and a modulated oi corresponding to a gabor scene
oiModulated = oiCompute(oi, gaborScene);
modulationRegion.radiusInMicrons = 250;
theOIsequence4 = oiSequence(oiBackground, oiModulated, modulationFunction2, 'modulationRegion', modulationRegion, 'oiModulatedReplacesBackground', true);


% Plot the oisequences
hFig = figure(1); clf;
set(hFig, 'Color', [1 1 1], 'Position', [10 10 1500 950]);
for oiIndex = 1:theOIsequence.length
    
    % Plot the modulation function
    subplot(8,round(theOIsequence.length/2)+1, 1);
    plot(1:theOIsequence.length, modulationFunction, 'ks-', 'LineWidth', 1.5);
    set(gca, 'XLim', [1 theOIsequence.length]);
    xlabel('frame index');
    ylabel('modulation');
    
    % Ask theOIsequence to return the oiIndex-th frame
    currentOI = theOIsequence.frameAtIndex(oiIndex);
    
    % plot it
    subplot(8,round(theOIsequence.length/2)+1, 1+oiIndex);
    rgbImage = xyz2rgb(oiGet(currentOI, 'xyz'));
    imagesc(rgbImage, [0 1]);
    title(sprintf('frame %d', oiIndex));
    axis 'image'
    set(gca, 'XTick', [], 'YTick', []);
    
    
    % Plot the modulation function
    subplot(8,round(theOIsequence.length/2)+1, 1 + 2*(1+round(theOIsequence.length/2)));
    plot(1:theOIsequence.length, modulationFunction, 'ks-', 'LineWidth', 1.5);
    set(gca, 'XLim', [1 theOIsequence.length]);
    xlabel('frame index');
    ylabel('modulation');
    
    % Ask theOIsequence2 to return the oiIndex-th frame
    currentOI = theOIsequence2.frameAtIndex(oiIndex);
    
    % plot it
    subplot(8,round(theOIsequence.length/2)+1, 1+oiIndex+2*(1+round(theOIsequence.length/2)));
    rgbImage = xyz2rgb(oiGet(currentOI, 'xyz'));
    imagesc(rgbImage, [0 1]);
    title(sprintf('frame %d', oiIndex));
    axis 'image'
    set(gca, 'XTick', [], 'YTick', []);
    
    
    % Plot the modulation function
    subplot(8,round(theOIsequence.length/2)+1, 1 + 4*(1+round(theOIsequence.length/2)));
    plot(1:theOIsequence.length, modulationFunction2, 'ks-', 'LineWidth', 1.5);
    set(gca, 'XLim', [1 theOIsequence.length]);
    xlabel('frame index');
    ylabel('modulation');
    
    % Ask theOIsequence3 to return the oiIndex-th frame
    currentOI = theOIsequence3.frameAtIndex(oiIndex);
    
    % plot it
    subplot(8,round(theOIsequence.length/2)+1, 1+oiIndex+4*(1+round(theOIsequence.length/2)));
    rgbImage = xyz2rgb(oiGet(currentOI, 'xyz'));
    imagesc(rgbImage, [0 1]);
    title(sprintf('frame %d', oiIndex));
    axis 'image'
    set(gca, 'XTick', [], 'YTick', []);
    
    
    % Plot the modulation function
    subplot(8,round(theOIsequence.length/2)+1, 1 + 6*(1+round(theOIsequence.length/2)));
    plot(1:theOIsequence.length, modulationFunction2, 'ks-', 'LineWidth', 1.5);
    set(gca, 'XLim', [1 theOIsequence.length]);
    xlabel('frame index');
    ylabel('modulation');
    
    % Ask theOIsequence4 to return the oiIndex-th frame
    currentOI = theOIsequence4.frameAtIndex(oiIndex);
    
    % plot it
    subplot(8,round(theOIsequence.length/2)+1, 1+oiIndex+6*(1+round(theOIsequence.length/2)));
    rgbImage = xyz2rgb(oiGet(currentOI, 'xyz'));
    imagesc(rgbImage, [0 1]);
    title(sprintf('frame %d', oiIndex));
    axis 'image'
    set(gca, 'XTick', [], 'YTick', []);
    
    
end
