function fSupport = oiFrequencySupport(oi, units)
% Compute spatial frequency support for an optical image 
%
% Syntax:
%   fSupport = oiFrequencySupport(oi, [units])
%
% Description:
%    Various calculations, such as the OTF, require the  spatial frequency
%    range supported by the sampling density of the optical image. This
%    routine, and the related sceneFrequencySupport, compute the spatial
%    frequency support and return it as an array.
%
%    The default units of fSupport are cycles/deg of visual angle. Other
%    options are cycles/distance (e.g., cycles/meter, cycles/mm, 
%    cycles/microns).
%
%    The frequency support returns values for all optics models, including
%    ray trace.
%
%    The code contains examples of usage below. To access these examples,
%    type 'edit oiFrequencySupport.m' into the Command Window.
%
% Inputs:
%    oi       - Struct. An optical image structure.
%    units    - (Optional) String. The units. Default 'cyclesPerDegree'.
%               Options include 'cyclesPerDegree', 'meters', 'millimeters',
%               and 'microns'.
%
% Outputs:
%    fSupport - Vector. The spatial frequency Support, as an array.
%
% Optional key/value pairs:
%    None.
%
% Notes:
%    * TODO: Assign someone to fix the example.
%

% History:
%    xx/xx/03       Copyright ImagEval Consultants, LLC, 2003.
%    03/06/18  jnm  Formatting

% Examples:
%{
    oi = oiCreate;
    fSmm  = oiFrequencySupport(oi, 'mm');   % cycles/millimeter
    fScpd = oiFrequencySupport(oi, 'cycPerDeg');
    fScpd = oiFrequencySupport(oi);
%}

if notDefined('units'), units = 'cyclesPerDegree'; end

% We also begin by calculating the frequencies in cycles per degree of
% visual angle.
%
% oi frequency information
% hangular is height (angular), wangular is width (angular)
fovHeight = oiGet(oi, 'hangular');  % oi height in degrees
fovWidth  = oiGet(oi, 'wangular');  % oi width in degrees

% If oi is empty, this returns the number of rows and columns.
nRows = oiGet(oi, 'rows');  % Number of oi row and col samples
nCols = oiGet(oi, 'cols');

% Next, we compute the spatial frequency list in cycles per degree
%
% The Nyquist frequency just with respect to the samples is N/2.
%
%If the oi spans 1 deg, then nCols / 2 or nRows / 2 is the Nyquist
% frequency in cycles/deg.
%
% But the oi field of view may differ from one, so we need to divide by
% the true number of degrees. 
%
% For example:
%    If you have 100 spatial samples in a single degree, the Nyquist limit
%    runs to 50 cyc/deg.
%    If the FOV is 40 deg, though, the highest spatial frequency is 50/40
%    cyc/deg.
maxFrequencyCPD = [(nCols / 2) / fovWidth, (nRows / 2) / fovHeight];

% Now, if the request is in units other than cyc/deg, we convert 
switch lower(units)
    case {'cyclesperdegree', 'cycperdeg'}
        maxFrequency = maxFrequencyCPD;
    case {'meters', 'm', 'millimeters', 'mm', 'microns', 'um'}
        degPerDist = oiGet(oi, 'degPerDist', units);
        maxFrequency = maxFrequencyCPD * degPerDist;
    otherwise 
        error('Unknown spatial frequency units');
end

% DC = 1.  The first coefficient, K, past the Nyquist is (K - 1) > N / 2, 
% K > (N / 2 + 1).  This is managed in the unitFrequencyList routine.
fSupport.fx = unitFrequencyList(nCols) * maxFrequency(1);
fSupport.fy = unitFrequencyList(nRows) * maxFrequency(2);

end