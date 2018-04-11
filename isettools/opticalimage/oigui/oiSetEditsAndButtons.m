function oiSetEditsAndButtons(handles)
% Refresh the buttons and edit fields in the optical image window
%
% Syntax:
%   oiSetEditsAndButtons(handles)
%
% Description:
%    Refresh the current optical image window. If there is no optical image, 
%    then a default oi is created.
%
%    Perhaps we should have a flag to leave the inline message in place?
%
% Inputs:
%    handles - Handle to the optical image's window.
%
% Outputs:
%    None.
%
% Optional key/value pairs:
%    None.
%

% History:
%    xx/xx/03       Copyright ImagEval Consultants, LLC, 2003.
%    03/19/18  jnm  Formatting

[val, oi] = vcGetSelectedObject('OPTICALIMAGE');
if isempty(oi)
    oi = oiCreate;
    val = 1;
end

% Clear the inline message
optics = oiGet(oi, 'optics');
ieInWindowMessage('', handles, []);

% Which optics model
opticsModel = opticsGet(optics, 'model');

% Append the oi names to 'New'
names = {'New'};
oiNames = vcGetObjectNames('OPTICALIMAGE');
for ii = 1:length(oiNames), names{ii + 1} = char(oiNames(ii)); end

% Select the appropriate OI entry to display.
if length(names) > 1, val = val + 1; end
set(handles.SelectOptImg, 'String', names, 'Value', val);
% We use a slightly different logic in sceneSetEditsAndButtons.

% Set up the buttons for different optics models.
switch lower(opticsModel)

    case {'diffractionlimited', 'dlmtf'}
        set(handles.popOpticsModel, 'Value', 1);
        switchControlVisibility(handles, 'on');

        % Set the diffraction limited optics parameters
        optics = oiGet(oi, 'optics');
        str = sprintf('%2.2f', opticsGet(optics, 'focal Length', 'mm'));
        set(handles.editFocalLength, 'String', str);
        str = sprintf('%1.2f', opticsGet(optics, 'fnumber'));
        set(handles.editFnumber, 'String', str);

        val = opticsGet(optics, 'off axis method');
        if strcmpi(val, 'skip'), set(handles.btnOffAxis, 'Value', 0);
        else, set(handles.btnOffAxis, 'Value', 1);

        end

    case 'shiftinvariant'
        set(handles.popOpticsModel,'Value',2);
        switchControlVisibility(handles,'off','shiftinvariant');
        
    case 'iset3d'
        set(handles.popOpticsModel,'Value',3);
        switchControlVisibility(handles,'off','iset3d');
        
    otherwise
        error('Unknown optics model')
end

% Adjust the anti-alias filter
dMethod = oiGet(oi, 'diffuserMethod');

switch lower(dMethod)
    case 'skip'
        set(handles.popDiffuser, 'val', 1)
        set(handles.editDiffuserBlur, 'visible', 'off');
        set(handles.txtBlurSD, 'visible', 'off');
        set(handles.popDiffuser, 'Position', [0.756 0.109 0.11 0.043])
        set(handles.txtDiffuser, 'Position', [0.756 .165 0.082 0.029])
        
    case 'blur'
        set(handles.popDiffuser, 'val', 2)
        set(handles.editDiffuserBlur, 'visible', 'on');
        set(handles.txtBlurSD, 'string', 'FWHM (um)');
        set(handles.txtBlurSD, 'visible', 'on');
        set(handles.txtBlurSD, 'TooltipString', 'Full width half maximum of Gaussian spread');
        
        set(handles.txtBlurSD, 'Position', [0.8693 .117 0.089 0.029])
        set(handles.editDiffuserBlur, 'Position', [0.82 0.115 0.038 0.033])
        set(handles.popDiffuser, 'Position', [0.693 0.109 0.11 0.043])
        set(handles.txtDiffuser, 'Position', [0.693 .165 0.082 0.029])
        
        val = oiGet(oi, 'diffuserBlur', 'um');
        if isempty(val)
            val = ieReadNumber('Enter blur sd (FWHM, um)', 2, ' %.2f');
            oi = oiSet(oi, 'diffuserBlur', val*10^-6);
            vcReplaceObject(oi);
        end
        set(handles.editDiffuserBlur, 'String', num2str(val));

    case 'birefringent'
        set(handles.popDiffuser, 'val', 3)
        set(handles.popDiffuser, 'Position', [0.756 0.109 0.11 0.043])
        set(handles.txtDiffuser, 'Position', [0.756 .165 0.082 0.029])

        % For now off, but if we decide to allow the value to change, then
        % we can use the buttons like this.
        set(handles.editDiffuserBlur, 'visible', 'off');
        set(handles.txtBlurSD, 'visible', 'off');
        set(handles.txtBlurSD, 'string', 'Disp (um)');
        set(handles.txtBlurSD, 'TooltipString', 'Birefringent displacement in microns');
    otherwise
        error('Unknown diffuser method %s\n', dMethod);
end

% If the incoming call set consistency true, then we eliminate the red
% square on the window.  Otherwise, consistency is false.  We always set it
% to false on the way out.
if oiGet(oi, 'consistency')
    set(handles.txtConsistency, 'BackgroundColor', [0.831373 0.815686 0.784314]);
    oi = oiSet(oi, 'consistency', 0);
    vcReplaceObject(oi);
else
    set(handles.txtConsistency, 'BackgroundColor', [1, 0, 0]);
end

gam = str2double(get(handles.editGamma, 'String'));

% Select the figure
% figure(ieSessionGet('oi figure'));
figNum = vcSelectFigure('OI');
figure(figNum);

% For NIR, SWIR and so forth we might use a different displayFlag value.
% See oiShowImage.  In the future, we will read the displayFlag from
% either a global or a setting in the oi GUI.
displayFlag = get(handles.popupDisplay, 'Value');
oiShowImage(oi, displayFlag, gam);
set(handles.txtOpticalImage, 'String', oiDescription(oi));

end

%------------------------------------------------------
function switchControlVisibility(handles,state,model)
% Control the display of the diffraction limited buttons and edit
% fields, fnumber and focal length.
%
% Also turns off the diffuser and blur buttons.
%
%

switch state
    case 'on'
        set(handles.txtFocalLength,'visible','on')
        set(handles.txtFnumber,'visible','on')
        set(handles.editFocalLength,'visible','on')
        set(handles.editFnumber,'visible','on')
        set(handles.txtM,'visible','on')
        set(handles.btnOffAxis,'visible','on');
        set(handles.editDiffuserBlur,'visible','on');
        set(handles.txtBlurSD,'visible','on');
        set(handles.popDiffuser,'visible','on');
        set(handles.txtDiffuser,'visible','on');
        
    case 'off'
        set(handles.txtFocalLength,'visible','off')
        set(handles.txtFnumber,'visible','off')
        set(handles.editFocalLength,'visible','off')
        set(handles.editFnumber,'visible','off')
        set(handles.txtM,'visible','off')
        set(handles.txtBlurSD,'visible','off');
    
        % Turn stuff off a little differently from the various models
        % Not quite sure we have the right conditions here.  For
        % diffraction, I think it is always on.  For iset3d we have
        % none of the options, but for shift invariant, we might have
        % some of the other calculations (off axis, anti-alias).  Not
        % sure. (BW).
        switch model
            case 'shiftinvariant'
            case 'diffraction'
            case 'iset3d'
                set(handles.btnOffAxis,'visible','off');
                set(handles.popDiffuser,'visible','off');
                set(handles.txtDiffuser,'visible','off');
                set(handles.editDiffuserBlur,'visible','off');
        end

    otherwise
        error('Unknown state.');
        
end

end