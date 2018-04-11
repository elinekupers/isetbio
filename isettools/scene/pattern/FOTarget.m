function RGB = FOTarget(pattern, params)
% Create a frequency/orientation image
%
% Syntax:
%   target = FOTarget(pattern, parms)
%
% Description: 
%    This target is a black and white image divided into a set of blocks.
%    Each block has a different spatial frequency pattern at one of several
%    orientations. The patterns increase in spatial frequency along the
%    horizontal dimension and change in orientation along  the vertical
%    dimension. This pattern is well-suited for evaluating demosaicing and
%    spatial image processing.
%
%    The spatial patterns can be either square waves or sinusoids. At high
%    spatial frequencies, the optics in a real camera does not produce
%    square waves. So, in general we recommend using the sinusoidal format
%    for testing.
%
%    The detailed parameters of the harmonic patterns and their
%    orientations are set in the parms structure, as explained below.
%
%    Examples are located within the code. To access the examples, type
%    'edit FOTarget.m' into the Command Window.
%
% Inputs:
%    pattern - The pattern of the image. Supported options are 'sine' and
%              'square'. Failing to provide one of the supported patterns
%              will result in an error.
%    params  - (Optional). The structure containing  additional information
%              about the target. Default is empty. Possible additions
%              include the following:
%               angles    - The Angles. Default linspace(0, pi/2, 8)
%               freqs     - The spatial frequency values. Default (1:8)'
%               blockSize - The size of the blocks. Default 32.
%               contrast  - The contrast ratio. Default 1.
%
% Outputs:
%    RGB     - The frequency/orientation image.
%
% Optional key/value pairs:
%    None.
%

% History:
%    xx/xx/05       Copyright ImagEval Consultants, LLC, 2005.
%    01/29/18  jnm  Formatting

% Examples:
%{
    % The number of blocks, spatial frequency values, and contrast are set
    % by the entries of the parms variable as in the example:
    parms.angles = linspace(0, pi / 2, 5);
    parms.freqs =  [1, 2, 4, 8, 16];
    parms.blockSize = 64;
    parms.contrast = .8;
    target = FOTarget('sine', parms);
    imshow(target)
%}

% Read the input parameters
if notDefined('params'), params = []; end
try angles = params.angles; catch, angles = linspace(0, pi / 2, 8); end
try freqs = params.freqs(:); catch, freqs = (1:8)'; end
try contrast = params.contrast; catch, contrast = 1; end
try blockSize = params.blockSize; catch, blockSize = 32; end

% Create the spatial grid
x = (0:blockSize - 1) / blockSize;
[X, Y] = meshgrid(x, x);

switch lower(pattern)
case 'sine'
    thisBlock = kron(cos(angles), X) + kron(sin(angles), Y);
    im = (1 + contrast * sin(2 * pi * kron(freqs, thisBlock))) / 2;
case 'square'
    thisBlock = kron(cos(angles), X) + kron(sin(angles), Y);
    im = (1 + contrast * square(2 * pi * kron(freqs, thisBlock))) / 2;
otherwise
    error('%s:  Unrecognized pattern', mfilename);
end

% We prefer the frequency variation from left to right.
im = im';
RGB = im(:, :, ones(3, 1));

end