function irReconstruct(innerRetina, varargin)
% Reconstructs the stimulus according to the inner retina representation by
% retinal ganglion cell mosaics.
% 
% As of now, this executes only a dumb linear reconstruction where the STRF
% of each cell is added to the stimulus when a spike occurs.
% 
% Inputs: an inner retina object and the method of reconstruction.
% 
% Outputs: a movie showing the reconstruced stimulus from the RGC
% representation.
% 
% Examples:
%   irReconstruct(innerRetina);
%   irReconstruct(innerRetina,'model','linear');
% 
% 2/2016 JRG (c) isetbio team
%
%% Parse input
p = inputParser;
p.addRequired('innerRetina');
p.addOptional('model', 'linear', @ischar);

p.parse(innerRetina, varargin{:});

innerRetina = p.Results.innerRetina;
model = p.Results.model;


%% 


switch model
case{'linear'}

% Initialize figure
% vcNewGraphWin([],'upperleftbig');
figure; set(gcf,'position',[160 60 1070 740]);
hold on;

% TODO: Loop over all mosaics
nX = 0; nY = 0;
cellTypeInd = 1;

[nX,nY,~] = size(innerRetina.mosaic{cellTypeInd}.responseLinear);
nFrames = length(innerRetina.mosaic{cellTypeInd}.responseLinear{1,1});
% nX = nX + nXi;
% nY = nY = nYi;

% Find max positions
% allpos = vertcat(innerRetina.mosaic{cellTypeInd}.cellLocation{:});
% maxx = max(allpos(:,1))/1; maxy = max(allpos(:,2));

% metersPerPixel = 

% Find spatial RF size
[nPixX, nPixY] = size(innerRetina.mosaic{cellTypeInd}.sRFcenter{1,1});
nFramesRF = length(innerRetina.mosaic{cellTypeInd}.tCenter{1});

stimulusReconstruction = zeros(nPixX*nX, nPixY*nY, nFrames + nFramesRF);

for cellTypeInd = 1:length(innerRetina.mosaic)
    
    
[nX,nY,~] = size(innerRetina.mosaic{cellTypeInd}.responseLinear);

% Loop through each cell and plot spikes over time
for xc = 1:nX
    for yc = 1:nY
        
        [nPixX, nPixY] = size(innerRetina.mosaic{cellTypeInd}.sRFcenter{1,1});
        nFramesRF = length(innerRetina.mosaic{cellTypeInd}.tCenter{1});
        
        % Build the STRF of the cell
        sRF = innerRetina.mosaic{cellTypeInd}.sRFcenter{xc,yc} - innerRetina.mosaic{cellTypeInd}.sRFsurround{xc,yc};
        tRF(1,1,:) = innerRetina.mosaic{cellTypeInd}.tCenter{1};
        strf = repmat(sRF,[1 1 nFramesRF]).*repmat(tRF, [nPixX, nPixY, 1]);
        
        % Get the appropriate spike data
        spPlot=innerRetina.mosaic{cellTypeInd}.responseSpikes{xc,yc,1,1};
        % spPlot=(median(horzcat(innerRetina.mosaic{3}.spikeResponse{xc,yc,:,2})'));
        
        % Add the STRF to the stimulus reconstruction for each spike
        for iFrame = 1:length(spPlot)
            
            % ycoords = (yc-1)*nPixY + 1 : yc*nPixY;
            % xcoords = (xc-1)*nPixX + 1 : xc*nPixX;
            
            centerCoords = innerRetina.mosaic{cellTypeInd}.cellLocation{xc,yc};            
            ycoords = 6 + (ceil(centerCoords(1) - (nPixY/2)) : floor(centerCoords(1) + (nPixY/2))); 
            xcoords = 6 + (ceil(centerCoords(2) - (nPixX/2)) : floor(centerCoords(2) + (nPixX/2))); 
            
            tcoords = ceil(1*spPlot(iFrame)) : ceil(1*spPlot(iFrame))+nFramesRF-1;
            
            stimulusReconstruction(xcoords, ycoords, tcoords) = ...
                stimulusReconstruction(xcoords, ycoords, tcoords) + ...
                strf;
        end%iFrame
        
        
    end%nX
end%nY
end

case{'otherwise'}
    error('Model does not exist');
end

maxR = max(stimulusReconstruction(:));
minR = min(stimulusReconstruction(:));

% Play the movie
for iFrame = 1:size(stimulusReconstruction,3)
    imagesc(stimulusReconstruction(:,:,iFrame));
    colormap gray
    caxis([minR maxR]);
    pause(0.1);
end
