function [res, wave, comment, partialName] = ieReadSpectra(fname, wave, ...
 extrapVal)
% Read in spectral data and interpolate to the specified wavelengths
%
% Syntax:
%   [res, wave, comment, fName] = ieReadSpectra([fname], [wave], extrapVal)
%
% Description:
%    Spectral data are stored in files that include both the sampled data
%    and the wavelength values. This routine reads the stored values and
%    returns them interpolated or extrapolated to the values in parameter
%    wave. 
%
%    IMPORTANT: Color filters, as opposed to general spectral
%    functions, are handled a little differently because we also store
%    their names. See the functions ieReadColorFilter and
%    ieSaveColorFilter
%
%    The spectral files are created by ieSaveSpectralFile, and the format
%    is determined by that function.
%
%    ISET spectral files are generally saved in the form: 
%        save(fname, 'data', 'wavelength')
%    and most have comment fields: 
%        save(fname, 'data', 'wavelength', 'comment')
%   
%    If the FNAME file does not exist, the return variable, res, is empty.
%    If wave is specified, the returned data are interpolated to wave If
%    wave is not specified, the data are returned at native resolution of
%    the data file and the values of wavelength can be returned.
%
%    Examples are located within the code. To access the examples, type
%    'edit ieReadSpectra.m' into the Command Window.
%
% Inputs:
%    fname       - (Optional) Filename containing spectral data. Default ''
%    wave        - (Optional) Wavelengths to interpolate. Default [].
%    extrapVal   - (Optional) Whether to interpolate or extrapolate the
%                  values. Default 0.
%
% Outputs:
%    res         - Return variable of the filename
%    wave        - Wavelengths used, the native if none provided to
%                  interpolate or extrapolate with
%    comment     - Comment
%    partialName - Partial filename
%
% Optional key/value pairs:
%    None.
%
% See Also:
%    ieReadColorFilter, ieSaveSpectralFile
%

% History:
%    11/23/17  jnm  Formatting
%    11/29/17  jnm  Fix the if/else from one liner, adjust example, add see
%                   also section.
%    01/26/18  jnm  Formatting update to match the Wiki.

% Examples:
%{
    wave = 400:10:700;
    data = ieReadSpectra('D65', wave)
    [data, wave] = ieReadSpectra('D50', 400:5:660)
%}

if notDefined('fname'), fname = ''; end

% Create a partialpath for this file name. For this to work, we need to
% keep all of the spectral data in a single directory, I am afraid.
if isempty(fname)
    partialName = vcSelectDataFile('');
    if isempty(partialName), return; end
else
    partialName = fname;
end

test = exist(partialName, 'file');
% If partialName is a directory or doesn't exist, we have a problem.
if ~test || test == 7
    partialName = sprintf('%s.mat', partialName);
    if ~exist(partialName, 'file')
        res = [];
        comment = [];
        warning('%s: file %s not found. Empty returned', ...
                    mfilename, partialName);
        return;
    end
end

% Load in spectral data We should probably trap this condition so that if
% it fails the user is sent into a GUI to find the data file. Also, we
% should use foo = load(partialName) if isfield(foo, 'comment') ...
% approach in case the file is missing a comment. Then we should return an
% empty comment.
tmp = load(partialName);
if isfield(tmp, 'data'), data = tmp.data; else, data = []; end
if isfield(tmp, 'wavelength')
    wavelength = tmp.wavelength; 
else
    wavelength = [];
end
if isfield(tmp, 'comment')
    comment = tmp.comment;
else
    comment = [];
end
if length(wavelength) ~= size(data, 1)
    error('Mismatch between wavelength and data in %s', partialName);
end

% If wave was not sent in, return the native resolution in the file. No
% interpolation will occur.
if notDefined('wave'), wave = wavelength; end
if notDefined('extrapVal'), extrapVal = 0; end

res = interp1(wavelength(:), data, wave(:), 'linear', extrapVal);
    
end