function irradiance = oiCalculateIrradiance(scene, optics)
% Calculate optical image irradiance
%
% Syntax:
%   irradiance = oiCalculateIrradiance(scene, optics)
%
% Description:
%    The scene spectral radiance (photons/s/m2/sr/nm) is turned into
%    optical image irradiance (photons/s/m2/nm) based on information in the
%    optics. The formula for converting radiance to irradiance is:
%
%       irradiance = pi /(1 + 4 * fN ^ 2 * (1 + abs(m)) ^ 2) * radiance;
%
%    where m is the magnification and fN is the f-number of the lens.
%    Frequently, in online references one sees the simpler formula:
%
%       irradiance = pi / (4 * fN ^ 2 * (1 + abs(m)) ^ 2) * radiance;
%
%    (e.g., Gerald C. Holst, CCD Arrayas, Cameras and Displays, 2nd
%    Edition, pp. 33-34 (1998))
%
%    This second formula is accurate for small angles, say when the sensor
%    sees only the paraxial rays. The formula used here is more general
%    and includes the non-paraxial rays.
%
%    On the web one even finds simpler formulae, such as
%
%       irradiance = pi / (4 * FN ^ 2) * radiance
%
%    For example, this formula is used in these online notes
%       <http://www.ece.arizona.edu/~dial/ece425/notes7.pdf>
%       <http://www.coe.montana.edu/ee/jshaw/teaching/RSS_S04/
%           Radiometry_geometry_RSS.pdf>
%
% Inputs:
%    scene      - Struct. A scene structure.
%    optics     - Struct. An optical structure. Can be an OI.
%
% Outputs:
%    irradiance - The calculated irradiance of the OI.
%
% Optional key/value pairs:
%    None.
%
%  References:
%    The formula is derived in Peter Catrysse's dissertation (pp. 150-151).
%    See also http://eeclass.stanford.edu/ee392b/, course handouts
%    William L. Wolfe, Introduction to Radiometry, SPIE Press, 1998.
%

% History:
%    xx/xx/05       Copyright ImagEval Consultants, LLC, 2005.
%    03/02/18  jnm  Formatting

% optics might be an oi or an optics
if isequal(optics.type, 'opticalimage')
    % It is an oi, so get the optics from it
    optics = oiGet(optics, 'optics');
end

% Scene data are in radiance units
radiance = sceneGet(scene, 'photons');
wave = sceneGet(scene, 'wave');

% oi = vcGetObject('oi');
model = opticsGet(optics, 'model');
model = ieParamFormat(model);
switch model
    case {'diffractionlimited', 'shiftinvariant'}
        sDist = sceneGet(scene, 'distance');
        fN = opticsGet(optics, 'fNumber');  % What should this be?
        m = opticsGet(optics, 'magnification', sDist);
    otherwise
        % In ISET there is a ray trace model. But not here.
        error('Unknown optics model');
end

% Apply lens transmittance.
% Perhaps we should be getting the transmittance out of ZEMAX/CODEV
if isfield(optics, 'lens')
    transmittance = opticsGet(optics, 'transmittance', 'wave', wave);
else
    transmittance = opticsGet(optics, 'transmittance', wave);
end

% If transmittance is all 1s, we can skip this step
if any(transmittance(:) ~= 1)
    % Do this in a loop to avoid large memory demand
    transmittance = reshape(transmittance, [1 1 length(transmittance)]);
    radiance = bsxfun(@times, radiance, transmittance);
end

% Apply the formula that converts scene radiance to optical image
% irradiance
irradiance = pi / (1 + 4 * fN ^ 2 * (1 + abs(m)) ^ 2) * radiance;

end