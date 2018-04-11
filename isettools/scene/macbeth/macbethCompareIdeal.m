function [embRGB, mRGB, pSize] = macbethCompareIdeal(mRGB, pSize, illType)
% Create an image of an ideal MCC (color, temp, ...) with data embedded
% 
% Syntax:
%	[embRGB, mRGB, pSize] = macbethCompareIdeal(mRGB, pSize, illType)
%
% Description:
%    Create an image of an ideal Macbeth Color Chart (including color, 
%    temperature, etc...) with embedded data.
%
%    Examples are contained in the code. To access, type 'edit
%    macbethCompareIdeal.m' into the Command Window.
% Inputs:
%    mRGB    - Macbeth RGB values of the data in the vcimageWindow. Default
%              is to calculate from the current vcimage.
%    pSize   - (Optional) Patch size for the embedded targets.Default is to
%              calculate from the current vcimage.
%    illType - (Optional) Illuminant name (e.g., 'd65'). See illuminantRead
%              for all illuminant type options. Default 'd65'.
%
% Outputs:
%    embRGB  - The embedded RGB color data.
%    mRGB    - The same mRGB provided in input.
%    pSize   - The patch size.
%
% Optional key/value pairs:
%    None.
%
% Notes:
%    * TODO: Need to be able to set illType parameter for color temperature
%    * TODO: Either create macbethSelect, or deprecate this function.
%
% See Also:
%    macbethIdealColor, macbethSelect
%

% History:
%    xx/xx/03       Copyright ImagEval Consultants, LLC, 2003.
%    01/31/18  jnm  Formatting
%    04/07/18  dhb  Do not run broken example.

% Examples:
%{
    % ETTBSkip. This function is broken, so the example doesn't work.
    [embRGB, mRGB, pSize] = macbethCompareIdeal; 

    macbethCompareIdeal(mRGB, pSize, 4000);
    macbethCompareIdeal(mRGB, pSize, 6000);
    macbethCompareIdeal(mRGB, pSize, 'd65');
%}

%% Arguments
vci = vcGetObject('vci');

% If the mRGB or pSize not defined, we need to do some processing.
if notDefined('mRGB') || notDefined('pSize')
    % Now, we get the RGB values for the image data displayed in the
    % image processing window. We treat this as lRGB (not sRGB) data.
    [mRGB, ~, pSize]= macbethSelect(vci);
    mRGB = reshape(mRGB, 4, 6, 3);
    mRGB = mRGB / max(mRGB(:));
end
if notDefined('illType'), illType = 'd65'; end

%% Calculate the lRGB values under this illuminant for an ideal MCC

% The first returns a 24x3 matrix. These are the linear rgb values for the
% MCC assuming an sRGB display.
ideal = macbethIdealColor(illType, 'lrgb');

% We reshape into a mini-image 
idealLRGB = XW2RGBFormat(ideal, 4, 6);

% Now expand the image to a bigger size to allow for the insertion of the
% data we are comparing.
fullIdealRGB = imageIncreaseImageRGBSize(idealLRGB, pSize);

%% Make the image with the data embedded

% Start with the full RGB image rendered for an sRGB display.
embRGB = fullIdealRGB;  % imagesc(embRGB)

% Embed the mRGB values into the ideal RGB images
w = pSize + round(-pSize / 3:0);
for ii = 1:4
    l1 = (ii - 1) * pSize + w;
    for jj = 1:6
        l2 = (jj - 1) * pSize + w;
        rgb = squeeze(mRGB(ii, jj, :));
        for kk = 1:3, embRGB(l1, l2, kk) = rgb(kk); end
    end
end

%% Display in graph window
% At this point, both of the images are in the linear RGB mode. The ideal
% are linear RGB for an sRGB display. We don't know the display for the
% vcimage RGB data, but the default is for the lcdExample display in the
% ISET distribution, which is close to an sRGB display. We should probably
% convert the ISET mRGB data to the sRGB format, which will account for the
% current display.

figNum = vcNewGraphWin([], 'wide');
str = sprintf('%s: MCC %s', imageGet(vci, 'name'), illType);
set(figNum, 'name', str);
set(figNum, 'Color', [1 1 1] * .7);

mRGB = lrgb2srgb(mRGB);
subplot(1, 2, 1)
imagesc(mRGB)
axis image;
axis off;
title('ISET MCC D65 simulation')

embRGB = lrgb2srgb(embRGB);
subplot(1, 2, 2)
imagesc(embRGB)
axis image;
axis off;
title('Simulation embedded in an ideal MCC D65')

end