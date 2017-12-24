function fullpathname = ieSaveSpectralFile(wavelength, data, comment, ...
    fullpathname, dFormat)
% Save a spectral data ISET data file
%
% Syntax:
%   fullpathname = ieSaveSpectralFile(wavelength, data, [comment], ...
%        [fullpathname], [dFormat]);
%
% Description:
%    This routine specifies the format that ISET spectral data are stored.
%    The data can be read by ieReadSpectra at a later time.
%
% Inputs:
%    wavelength   - An N-vector of wavelengths
%    data         - A matrix, NxM, of spectral functions in the columns, 
%    comment      - (Optional) A string with a comment. Default is blank.
%    fullpathname - (Optional) Full path name for output file.
%    dFormat      - (Optional) Data format. Default is double. Options are
%                   double, single, compressed32, compressed16 
%
% Notes:
%
% See Also:
%   ieSaveColorFilter, ieReadColorFilter
%

% History:
%    xx/xx/03       Copyright ImagEval Consultants, LLC, 2003.
%    11/27/17  jnm  Formatting
%    11/29/17  jnm  Added Note about example
%    12/24/17   BW  Fixed problems noted by JNM

% Examples:
%{
    comment = 'foo';
    wavelength = [400, 500, 600]';
    variable = [1, 1, 1]';
    fname = fullfile(tempdir,'sifile.mat');
    fullpathname = ieSaveSpectralFile(wavelength, variable, comment,fname)
    data = ieReadSpectra(fullpathname, [400:50:600])
    delete(fullpathname)
%}

if notDefined('data') , error('data required.'); end
if notDefined('wavelength'), error('wavelength required'); end
if notDefined('comment'), comment = ''; end

% Check data format for match with wavelength
if ndims(data) == 3
    % Data are in RGB format
    if length(wavelength) ~= size(data, 3)
        error('Third dimension of data must match number of wavelengths');
    end
elseif ismatrix(data)
    % Data are in the columns
    if length(wavelength) ~= size(data, 1)
        errordlg('Row dimension of data must match number of wavelengths');
    end
end

if notDefined('fullpathname')
    fullpathname = vcSelectDataFile(isetbioDataPath, 'w', 'mat');
    if isempty(fullpathname), disp('User canceled'); return; end
end
if notDefined('dFormat'), dFormat = 'double'; end

% Manage data format for compression.
% This was put in for handling the hyperspectral face data. The file sizes
% were on the order of 2GB. For distribution we decided to save them as
% uint16 in the compressed photon format. BW/JEF
switch dFormat
    case 'double'
        % Do nothing - Typical for filters and simple data
    case 'single'
        % Not yet used
        data = single(data);
    case 'compressed32'
        % Compression is used for image data
        [s, mn, mx] = ieCompressData(data, 32);
        clear data;
        data.s = s; data.mn = mn; data.mx = mx;
    case 'compressed16'
        [s, mn, mx] = ieCompressData(data, 32);
        clear data;
        data.s = s; data.mn = mn; data.mx = mx;
    otherwise
        error('Unknown data format %s\n', dFormat);
end

save(fullpathname,'wavelength','data','comment','dFormat');

end