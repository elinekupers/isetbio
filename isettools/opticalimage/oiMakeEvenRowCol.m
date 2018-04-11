function oi = oiMakeEvenRowCol(oi, sDist)
% Pad the OI to make the rows and cols even numbers
%
% Syntax:
%   oi = oiMakeEvenRowCol(oi, sDist)
%
% Description:
%    There are interpolation problems at the OTF calculation when the
%    row/col size of scene is odd. The OTF calculation is not putting the
%    DC term in the right place.
%
%    For now, until I understand the general way to handle the odd row and
%    col values, we pad the oi to have an even number of rows and columns
%    at the start of oiCompute.
%
%    This is not a terrible hack because there is oi padding in any event.
%
% Inputs:
%    oi    - Struct. An optical image structure.
%    sDist - (Optional) Scalar Numerical. The scene distance. Default
%            queries from an existing scene.
%
% Outputs:
%    oi    - Struct. The modified (padded) optical image structure.
%
% Optional key/value pairs:
%    None.
%

if notDefined('sDist')
    scene = vcGetObject('scene');
    sDist = sceneGet(scene, 'distance'); 
end

sz = oiGet(oi, 'size');

if isodd(sz(1)), padSize(1) = 1; end
if isodd(sz(2)), padSize(2) = 1; end

oi = oiPad(oi, [padSize 0], sDist, 'post');

end