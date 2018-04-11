function il = illuminantCreate(ilName, wave, varargin)
% Create an illuminant (light source) structure. 
%
% Syntax:
%   il = illuminantCreate([ilName], [wave], [varargin])
%
% Description:
%    Historically the illuminant is just a spectral function. Several of
%    the standard light sources are supported at present. These are d65,
%    d50, tungsten, fluorescent, blackbody's (over a range of color
%    temperatures), and 550nm. There is an internal routine, named
%    illuminantRead shown below.
%
%    We are now starting to experiment with spatial spectrum illuminants,
%    that is a separate illuminant for each point in the scene data.
%
%    The illuminant data are stored in units of [photons/(sr m^2 nm)]
%
%    N.B. There are examples contained in the code. To access, type 'edit
%    illuminantCreate.m' into the Command Window.
%
%    Illuminant list
%       555nm
%       blackbody
%       d50
%       d65
%       equal energy
%       fluorescent
%       illuminant c
%       tungsten
%
% Inputs:
%    ilName   - (Optional) The illuminant name. Default is 'd65'
%    wave     - The wavelength(s). Default 400:10:700 nm.
%    varargin - (Optional) Additional parameters as required based upon the
%               illuminant type. Default is None. Some possible options
%               include luminance, spectrum, and temperature.
%
% Outputs:
%    il       - The illuminant
%
% Optional key/value pairs:
%    None.
%
% Notes:
%    * [Note: JNM - Removed tempDegKelvin from illuminant list, as it is
%      not supported in the case statement below.]
%
% See Also:
%    illuminantSet/Get, s_sceneIlluminant, s_sceneIlluminantSpace, 
%    illuminantRead
%

% History:
%    xx/xx/05       Copyright ImagEval Consultants, LLC, 2005.
%    01/26/18  jnm  Formatting

% Examples:
%{
    
    il1 = illuminantCreate('d65')
    il2 = illuminantCreate('blackbody', 400:10:700, 3500, 100)
    il3 = illuminantCreate('blackbody', [], 6500, 100)
    il4 = illuminantCreate('illuminant c', 400:1:700, 500)

    spectrum.wave = (380:4:1068);
    il = illuminantCreate('equalEnergy', [], 100, spectrum)
%}


%% Initialize parameters
if notDefined('ilName'), ilName = 'd65'; end

il.name = ilName;
il.type = 'illuminant';
il = initDefaultSpectrum(il, 'hyperspectral');
if exist('wave', 'var') && ~isempty(wave), il.spectrum.wave = wave; end

%% There is no default
% The absence of a default could be a problem.

switch ieParamFormat(ilName)
    case {'d65', 'd50', 'tungsten', 'fluorescent', ...
          '555nm', 'equalenergy', 'illuminantc', 'equalphotons'}
        % illuminantCreate('d65', luminance)
        illP.name = ilName;
        illP.luminance = 100;
        illP.spectrum.wave = illuminantGet(il, 'wave');
        if ~isempty(varargin), illP.luminance = varargin{1}; end
        
        iEnergy = illuminantRead(illP);  % [W/(sr m^2 nm)]
        % Check the below step!
        iPhotons = Energy2Quanta(illuminantGet(il, 'wave'), iEnergy);
        il = illuminantSet(il, 'name', illP.name);

    case 'blackbody'
        % illuminantCreate('blackbody', 5000, luminance);
        illP.name = 'blackbody';
        illP.temperature = 5000;
        illP.luminance = 100;
        illP.spectrum.wave = illuminantGet(il, 'wave');
        
        if ~isempty(varargin), illP.temperature = varargin{1}; end
        if length(varargin) > 1, illP.luminance = varargin{2}; end
        
        iEnergy = illuminantRead(illP);  % [W/(sr m^2 nm)]
        % Check the below step!
        iPhotons = Energy2Quanta(illuminantGet(il, 'wave'), iEnergy);
        
        il = illuminantSet(il, 'name', ...
            sprintf('blackbody-%.0f', illP.temperature));
        
    otherwise
        error('unknown illuminant type %s\n', ilName);
end

%% Set the photons and return
il = illuminantSet(il, 'photons', iPhotons);  % [photons/(s sr m^2 nm)]

end