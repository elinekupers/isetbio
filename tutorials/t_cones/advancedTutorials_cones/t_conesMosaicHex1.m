%%t_conesMosaicHex1
%
% Shows how to generate a default hexagonal mosaic, and how to customize it (FOV, resamplingFactor).
%
% NPC ISETBIO Team, Copyright 2016

%% Initialize
ieInit; clear; close all;

% Interactive mode. Set to true to have it pause at useful places.
% Default is false so we can autopublish without user input
interactiveMode = false;

% Freeze random number generator
rng('default'); rng(219347);

%% Unit test 1: generate a hex mosaic using defaults params of the superclass (coneMosaic)
%
% Mosaic Parameters
mosaicParams = struct(...
    'resamplingFactor', 5,  ...                 % controls the accuracy of the hex mosaic grid
    'spatiallyVaryingConeDensity', false, ...   % whether to have an eccentricity based, spatially - varying density
    'customLambda', []  ...                     % custom spacing?
    );

% Generate the hex grid
theHexMosaic = coneMosaicHex(mosaicParams.resamplingFactor, ...
    'eccBasedConeDensity', mosaicParams.spatiallyVaryingConeDensity, ...
    'customLambda', mosaicParams.customLambda);

% Print some grid info and visualize it
theHexMosaic.displayInfo();
theHexMosaic.visualizeGrid('visualizedConeAperture', 'lightCollectingArea');
theHexMosaic.visualizeGrid('visualizedConeAperture', 'geometricArea', 'generateNewFigure', true);

%% Unit test 2a: generate a custom hex mosaic
if (interactiveMode)
    commandwindow;
    fprintf('\n<strong>Hit enter to generate a customized hex mosaic based on an 16x16 rect mosaic with custom lambda (3 microns)\n</strong>');
    fprintf('<strong>Here we use a high resamplingFactor (10) to get a near perfect hex grid\n</strong>');
    pause
end

% Generate the hex grid
mosaicParams.resamplingFactor = 10;
mosaicParams.customLambda = 3;          % 2 microns
theHexMosaic = coneMosaicHex(mosaicParams.resamplingFactor, ...
    'eccBasedConeDensity', mosaicParams.spatiallyVaryingConeDensity, ...
    'customLambda', mosaicParams.customLambda, ...
    'size', [16 16]);

% Print some grid info and visualize it
theHexMosaic.displayInfo();
theHexMosaic.visualizeGrid('visualizedConeAperture', 'lightCollectingArea');
theHexMosaic.visualizeGrid('visualizedConeAperture', 'geometricArea', 'generateNewFigure', true);

%% Unit test 2a: generate a custom hex mosaic
if (interactiveMode)
    commandwindow;
    fprintf('\n<strong>Hit enter to generate a customized hex mosaic based on an 16x16 rect mosaic with custom lambda (6 microns)\n</strong>');
    fprintf('<strong>Here we use a high resamplingFactor (10) to get a near perfect hex grid\n</strong>');
    pause
end

% Generate the hex grid
customLambda = 6;  % 6 microns
theHexMosaic = coneMosaicHex(mosaicParams.resamplingFactor, ...
    'eccBasedConeDensity', mosaicParams.spatiallyVaryingConeDensity, ...
    'customLambda', mosaicParams.customLambda, ...
    'size', [16 16]);

% Print some grid info and visualize it
theHexMosaic.displayInfo();
theHexMosaic.visualizeGrid();

%% Unit test 3: generate a custom hex mosaic
if (interactiveMode)
    commandwindow;
    fprintf('\n<strong>Hit enter to generate a customized hex mosaic based on an 11x16 rect mosaic with equal LMS proportions\n</strong>');
    fprintf('<strong>Here we use a high resamplingFactor (10) to get a near perfect hex grid\n</strong>');
    pause
end

% Mosaic Parameters
mosaicParams = struct(...
    'resamplingFactor', 10, ...                     % controls the accuracy of the hex mosaic grid
    'spatiallyVaryingConeDensity', false, ...       % whether to have an eccentricity based, spatially - varying density
    'customLambda', [], ...                         % custom spacing
    'size', [11 16], ...                            % generate from a rectangular mosaic of 11 x 16 cones
    'spatialDensity', [0 1/3 1/3 1/3]...            % with a LMS density of of 0.33:0.33:0.33
    );
theHexMosaic = coneMosaicHex(mosaicParams.resamplingFactor, ...
    'eccBasedConeDensity', mosaicParams.spatiallyVaryingConeDensity, ...
    'customLambda', mosaicParams.customLambda, ...
    'name', 'the hex mosaic', ...
    'size', mosaicParams.size, ...
    'spatialDensity', mosaicParams.spatialDensity ...
    );

% Print some grid info and visualize it
theHexMosaic.displayInfo();
theHexMosaic.visualizeGrid();