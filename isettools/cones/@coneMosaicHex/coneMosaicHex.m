classdef coneMosaicHex < coneMosaic
% Create a hexagonal cone mosaic class
%
% Syntax:
%   cMosaicHex = coneMosaicHex(resamplingFactor, [varargin]);
%
% Description:
%    The cone mosaic HEX is a subclass of coneMosaic. It differs because
%    the array of cones is placed on a hexagonal, rather than rectangular, 
%    grid. 
%
%    The hex mosaic is sampled according to the resamplingFactor. The cone
%    density can be spatially-varying if eccBasedConeDensity is set to true.
%
%    The customLambda argument is empty to obtain default performance, but
%    may be set to set the spacing for regularly spaced hexagonal mosaics.
%    The units of customLambda are microns
%
% Inputs:
%    None required.
%
% Outputs:
%    The created cone mosaic hex object.
%
% Optional key/value pairs:
%    Some of the possible key/value pairs are listed below:
%    resamplingFactor    - Grid resampling factor
%    eccBasedConeDensity - Boolean denoting whether or not to use the
%                          eccentricity-based cone density. Default False.
%    customLambda        - Custom cone spacing distance in microns
%    customInnerSegmentDiameter
%                        - Customer diameter for inner segment. Default is
%                          the pigment.pdWidth from the super class.
%
% See Also:
%    CONEMOSAIC, t_coneMosaicHex, t_coneMosaicHexReg
%

% History:
%    xx/xx/16  NPC  ISETBIO Team, 2016
%    02/21/18  jnm  Formatting

% Examples:
%{
    resamplingFactor = 8;
    eccBasedConeDensity = true;
    customLambda = 3.0;
    % If not passed (or set to []) @coneMosaiHex chooses the cone spacing
    % based on the eccentricity of the mosaic as determined by the
    % coneSizeReadData function. If set to a value (specified in microns), 
    % cone spacing is set to that value. Note that if the
    % 'eccBasedConeDensity' param is  set to true, the 'customLambda' param
    % is ignored.

    customInnerSegmentDiameter = 2.5;
    % If not passed (or set to []) @coneMosaiHex chooses the default
    % pigment.pdWidth, pigment.pdHeight values from its superclass. If it
    % set to a value (specified in microns) @coneMosaicHex sets
    % pigment.pdWidth and pigment.pdheight to
    % squareSizeFromCircularAperture(...
    % customInnerSegmentDiameter).

    cMosaicHex = coneMosaicHex(resamplingFactor, ...
    'name', 'the hex mosaic', ...
    'fovDegs', 0.35, ...
    'eccBasedConeDensity', eccBasedConeDensity, ...   
    'noiseFlag', 'none', ...
    'spatialDensity', [0 0.6 0.3 0.1], ...
    'maxGridAdjustmentIterations', 100);

    cMosaicHex.window;
%}

    %% Private properties:
    properties (SetAccess=private)
        % lambdaMin  min cone separation in the mosaic
        lambdaMin

        % lambdaMid  the cone separation at the middle of the mosaic
        lambdaMid

        % customLambda  user-supplied lambda value for reg. spaced mosaics.
        %   (cone spacing, in microns)
        customLambda

        % customInnerSegmentDiameter  user-supplied inner segment diameter
        %	(for a circular aperture, in microns).
        customInnerSegmentDiameter

        % eccBasedConeDensity  Bool. Ecc.-based spatially-varying density?
        eccBasedConeDensity

        % resamplingFactor  resamplingFactor
        resamplingFactor

        % marginF  Factor determining how much more mosaic to compute.
        %   more (if > 1) or less (if < 1).
        marginF

        % sConeMinDistanceFactor  min distance between neighboring S-cones
        %   This will make the S-cone lattice semi-regular
        %   min dist = f * local cone separation.
        sConeMinDistanceFactor

        % sConeFreeRadiusMicrons  S-cone free retina radius, default 45 um.
        %   45 um is 0.15, so S-cone free region = 0.3 degs diameter.
        sConeFreeRadiusMicrons

        % coneLocsHexGrid  floating point coneLocs on the hex grid.
        %   This is sampled according to the resamplingFactor.
        coneLocsHexGrid

        % coneLocsOriginatingRectGrid  coneLocs of originating rect grid.
        coneLocsOriginatingRectGrid

        % patternOriginatingRectGrid  cone pattern of originating rec grid.
        patternOriginatingRectGrid

        % patternSampleSizeOriginatingRectGrid  pattern sample size ...
        patternSampleSizeOriginatingRectGrid
        
        % fovOriginatingRectGrid  FOV of the originating rect grid
        fovOriginatingRectGrid

        % rotationDegs  rotation in degrees
        rotationDegs

        % saveLatticeAdjustmentProgression  Bool. save lattice adj. steps?
        %   Flag for whether saving iterative lattice adjustment steps.
        saveLatticeAdjustmentProgression

        %latticeAdjustmentSteps  3D array w/ coneLocsHexGrid at ea. step.
        %   Each step is a lattice adjustment with coneLocsHexGrid
        latticeAdjustmentSteps

        %initialLattice  coneLocsHexGrid at iteration 0 (perfect hex grid)
        initialLattice

        %latticeAdjustmentPositionalToleranceF  Movement tolerance
        %	Tolerance for whether to decide that there is no move movement.
        latticeAdjustmentPositionalToleranceF

        %latticeAdjustmentDelaunayToleranceF  Delaunay trigger tolerance
        %   Tolerance for Delaunay triangularization trigger.
        latticeAdjustmentDelaunayToleranceF
        
        %maxGridAdjustmentIterations   Max number of iterations
        %   Max iterations for deciding whether the grid adjustment is done
        maxGridAdjustmentIterations             

    end
    
    % Public methods
    methods
        % Constructor
        function obj = coneMosaicHex(resamplingFactor, varargin)
            % Initialize the hex cone mosaic class
            %
            % Syntax:
            %    obj = coneMosaicHex(resamplingFactor, [varargin])
            %
            % Description:
            %    The constructor for cone mosaic hex.
            %
            % Inputs:
            %    resamplingFactor - The grid resampling factor
            %
            % Outputs:
            %    obj              - The created cone mosaic hex object
            %
            % Optional key/value pairs:
            %    See pairs in header above.
            %

            % Params that we want to consume
            % (not pass to our super-class @coneMosaic)
            paramsForConeMosaicHex = {...
                'fovDegs', ...
                'eccBasedConeDensity', ...
                'sConeMinDistanceFactor', ...
                'sConeFreeRadiusMicrons', ...
                'customLambda', ...
                'customInnerSegmentDiameter', ...
                'rotationDegs', ...
                'saveLatticeAdjustmentProgression', ...
                'maxGridAdjustmentIterations'...
                'latticeAdjustmentPositionalToleranceF', ...
                'latticeAdjustmentDelaunayToleranceF' ...
                'marginF'};
            
            % Call the super-class constructor.
            vararginForConeMosaic = {};
            vararginForConeHexMosaic = {};
            for k = 1:2:numel(varargin)
                if (ismember(varargin{k}, paramsForConeMosaicHex))
                    vararginForConeHexMosaic{numel(...
                        vararginForConeHexMosaic) + 1} = varargin{k};
                    vararginForConeHexMosaic{numel(...
                        vararginForConeHexMosaic) + 1} = varargin{k + 1};
                else
                    if (strcmp(varargin{k}, 'center'))
                        error(['Currently coneMosaicHex only supports ' ...
                            'mosaics centered at 0 eccentricity. Do ' ...
                            'not pass a ''center'' param.']);
                    end
                    vararginForConeMosaic{numel(vararginForConeMosaic) ...
                        + 1} = varargin{k};
                    vararginForConeMosaic{numel(vararginForConeMosaic) ...
                        + 1} = varargin{k + 1};
                end
            end
            obj = obj@coneMosaic(vararginForConeMosaic{:});

            % parse input
            p = inputParser;
            p.addRequired('resamplingFactor', @isnumeric);
            p.addParameter('fovDegs', 0.25, @(x)(isnumeric(x) && ...
                ((numel(x) == 1) || (numel(x) == 2))));
            p.addParameter('eccBasedConeDensity', false, @islogical);
            p.addParameter('sConeMinDistanceFactor', 3.0, @isnumeric);
            p.addParameter('sConeFreeRadiusMicrons', 45, @isnumeric);
            p.addParameter('customInnerSegmentDiameter', [], @isnumeric);
            p.addParameter('customLambda', [], @isnumeric);
            p.addParameter('rotationDegs', 0, @isnumeric);
            p.addParameter('latticeAdjustmentPositionalToleranceF', ...
                0.01, @isnumeric);
            p.addParameter('latticeAdjustmentDelaunayToleranceF', ...
                0.001, @isnumeric);
            p.addParameter('saveLatticeAdjustmentProgression', ...
                false, @islogical);
            p.addParameter('marginF', 1.0, @(x)((isempty(x)) || ...
                (isnumeric(x) && (x > 0.0))));
            p.addParameter('maxGridAdjustmentIterations', Inf, @isnumeric);
            p.parse(resamplingFactor, vararginForConeHexMosaic{:});
            
            % Set input params
            obj.resamplingFactor = p.Results.resamplingFactor;
            obj.eccBasedConeDensity = p.Results.eccBasedConeDensity;
            obj.sConeMinDistanceFactor = p.Results.sConeMinDistanceFactor;
            obj.sConeFreeRadiusMicrons = p.Results.sConeFreeRadiusMicrons;
            obj.customLambda = p.Results.customLambda;
            obj.customInnerSegmentDiameter = ...
                p.Results.customInnerSegmentDiameter;
            obj.rotationDegs = p.Results.rotationDegs;
            obj.saveLatticeAdjustmentProgression = ...
                p.Results.saveLatticeAdjustmentProgression;
            obj.latticeAdjustmentDelaunayToleranceF = ...
                p.Results.latticeAdjustmentDelaunayToleranceF;
            obj.latticeAdjustmentPositionalToleranceF = ...
                p.Results.latticeAdjustmentPositionalToleranceF;
            obj.maxGridAdjustmentIterations = ...
                p.Results.maxGridAdjustmentIterations;
            
            % Set FOV of the underlying rect mosaic
            if (numel(p.Results.fovDegs) == 1)
                obj.setSizeToFOV(p.Results.fovDegs(1) * [1 1]);
            else
                obj.setSizeToFOV(p.Results.fovDegs);
            end

            if (isempty(p.Results.marginF))
                if (max(p.Results.fovDegs) < 1.0)
                    % For the smaller mosaics, generate a little larger
                    % lattice in order to reduce positional artifacts near
                    % the borders.
                    obj.marginF = 1.0 + (1 - max(p.Results.fovDegs)) ...
                        * 0.25;
                else
                    obj.marginF = 1.0;
                end
            else
                obj.marginF = p.Results.marginF;
            end

            if (isempty(obj.latticeAdjustmentDelaunayToleranceF))
                obj.latticeAdjustmentDelaunayToleranceF = 0.01;
            end

            if (isempty(obj.latticeAdjustmentPositionalToleranceF))
                obj.latticeAdjustmentPositionalToleranceF = 0.001;
            end

            % Get a copy of the original coneLocs
            obj.saveOriginalResState();

            % Set custom pigment light collecting dimensions
            if (~isempty(obj.customInnerSegmentDiameter)) 
              % maxInnerSegmentDiameter = 1e6 * ...
              % circleSizeFromSquareAperture(obj.pigment.pdWidth);
              % if (obj.eccBasedConeDensity) && ...
              %      (obj.customInnerSegmentDiameter > ...
              %      maxInnerSegmentDiameter)
              %   error(['The custom inner segment diameter (%2.4f)' ...
              %      ' is > max inner segment diameter (%2.4f) ' ...
              %      'necessary to keep the default cone density. ' ...
              %      'Either set ''eccBasedConeDensity'' to false, ' ...
              %      'or decrease ''customInnerSegmentDiameter''.',' ...
              %      ' obj.customInnerSegmentDiameter, ' ...
              %      'maxInnerSegmentDiameter]);
              % end
                obj.pigment.pdWidth = 1e-6 * ...
                    squareSizeFromCircularAperture(...
                    obj.customInnerSegmentDiameter);
                obj.pigment.pdHeight = obj.pigment.pdWidth;
            end

            % Set the pigment geometric dimensions
            if (~isempty(obj.customLambda)) 
                maxSpacing = 1e6 * circleSizeFromSquareAperture(...
                    obj.pigment.width);
                if (obj.eccBasedConeDensity) && (obj.customLambda > ...
                        maxSpacing)
                    error(['The custom lambda (%2.4f) is > max ' ...
                        'separation (%2.4f) in order to keep the ' ...
                        'default cone density. Either set ' ...
                        '''eccBasedConeDensity'' to false, or ' ...
                        'decrease ''customLambda''.'], ...
                        obj.customLambda, maxSpacing);
                end
                obj.pigment.width = 1e-6 * obj.customLambda;
                obj.pigment.height = obj.pigment.width;
            end

            % Generate sampled hex grid
            obj.resampleGrid(obj.resamplingFactor);
            
            if ((~isempty(obj.sConeMinDistanceFactor)) || ...
                    (~isempty(obj.sConeFreeRadiusMicrons)))
                % Make s-cone lattice semi-regular, and/or add an s-cone
                % free region.
                obj.reassignConeIdentities('sConeMinDistanceFactor', ...
                    obj.sConeMinDistanceFactor, ...
                    'sConeFreeRadiusMicrons', obj.sConeFreeRadiusMicrons);    
            end
        end

        % Visualize different aspects of the hex grid
        hFig = visualizeGrid(obj, varargin);

        % Method to compute the cone density of @coneMosaicHex
        [densityMap, densityMapSupportX, densityMapSupportY] = ...
            computeDensityMap(obj, computeConeDensityMap)

        % Reshape a full 3D hex activation map (coneRows x coneCols x time]
        % to a 2D map (non-null cones x time)
        hex2Dmap = reshapeHex3DmapToHex2Dmap(obj, hex3Dmap);

        % Reshape a 2D map (non-null cones x time) to the full 3D hex
        % activation map (coneRows x coneCols x time)
        hex3Dmap = reshapeHex2DmapToHex3Dmap(obj, hex2Dmap);

        % Compute activation images for the hex mosaic
        % (all cones + LMS submosaics)
        [activationImage, activationImageLMScone, imageXaxis, ...
            imageYaxis] = computeActivationDensityMap(obj, activation);

        % Visualize activation map images for the hex mosaic (all cones +
        % LMS submosaics)
        hFig = visualizeActivationMaps(obj, activation, varargin);

        % Using the passed axesHandle, render (draw) an activation map for
        % the hex mosaic
        renderActivationMap(obj, axesHandle, activation, varargin);

        % Visualize iterative adjustment of the cone lattice 
        hFig = plotMosaicProgression(obj, varargin);

        % Print various infos about the cone mosaic
        displayInfo(obj);

        % Change cone identities according to arguments passed in varargin
        reassignConeIdentities(obj, varargin);
    end % Public methods
    
    methods (Access = private)
        % Private methods
        saveOriginalResState(obj);
        restoreOriginalResState(obj);

        % Change the FOV of the mosaic
        setSizeToFOVForHexMosaic(obj, fov);

        % Sample the original rectangular mosaic using a hex grid sampled
        % at the passed resamplingFactor
        resampleGrid(obj, resamplingFactor);
    end % Private methods

    methods (Static)
        renderPatchArray(axesHandle, pixelOutline, xCoords, yCoords, ...
            edgeColor, faceColor, lineStyle, lineWidth);
        renderHexMesh(axesHandle, xHex, yHex, meshEdgeColor, ...
            meshFaceColor, meshFaceAlpha, meshEdgeAlpha, lineStyle);
    end % Static methods
    
end

function square = squareSizeFromCircularAperture(diameter)
% Inline function for sizeForSquareApertureFromDiameterForCircularAperture
%
% Syntax:
%   square = squareSizeFromCircularAperture(diameter)
%
% Description:
%    sizeForSquareApertureFromDiameterForCircularAperture is just too long
%    of a function name to be useful. This shorter-named function will
%    merely call it. I will also create one for its inverse.
%
% Inputs:
%    diameter - The diameter of the circular aperture to pass
%
% Outputs:
%    square   - The size of the calculated square aperture
%
% Optional key/value pairs:
%    None.
%
square = sizeForSquareApertureFromDiameterForCircularAperture(diameter);
end

function diameter = circleSizeFromSquareAperture(square)
% Inline function for diameterForCircularApertureFromWidthForSquareAperture
%
% Syntax:
%   diameter = squareSizeFromCircularAperture(square)
%
% Description:
%    diameterForCircularApertureFromWidthForSquareAperture is just too long
%    of a function name to be useful. This shorter-named function will
%    merely call it. There is also a function for the inverse
%
% Inputs:
%    square   - The size of the square aperture to pass
%
% Outputs:
%    diameter - The diameter of the calculated circular aperture
%
% Optional key/value pairs:
%    None.
%
diameter = diameterForCircularApertureFromWidthForSquareAperture(square);
end
