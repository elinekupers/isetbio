function [sample_mean, sample_cove] = ...
    wvfLoadThibosVirtualEyes(pupilDiameterMM)
% Load Thibos mean and covariance of Zernicke coefficients for human data
%
% Syntax:
%   [sample_mean, sample_cov] = ...
%     wvfLoadThibosVirtualEyes([pupilDiameterMM])
%
% Description:
%    Load statistical summary of the Zernicke polynomial coeffcients
%    measured in the human eye. These come out in OSA order, including the
%    j = 0 (piston) term.
%
%    For each pupil size, there are different numbers of Zernicke
%    polynomials. Presumably this is because as the pupil area gets bigger
%    they felt they needed more basis terms?  Read the paper to understand.
%
%    The data and model are described in 
%       "Retinal Image Quality for Virtual Eyes Generated by a Statistical
%       Model of Ocular Wavefront Aberrations" published in
%       Ophthalmological and Physiological Optics (2009).
%
%    and in 
%       Statistical variation of aberration structure and image quality in
%       a normal population of healthy eyes Larry N. Thibos, Xin Hong, %
%       Arthur Bradley, and Xu Cheng (2002, JOSA).
%
%    Concerning the program, the authors write:
%       Permission is granted to use this program for scientific research
%       purposes. Commercial users should contact Indiana University for
%       licensing arrangements.
%
%    There are no specific comments about the data.
%
%    At one point, DHB compared these calculations to the figures in the
%    Autrusseau paper. He also compared the values of the Zernike
%    polynomial coefficients that we load to the data shown in Figure 6
%    of the JOSA-A paper. These align as well.
%
% Inputs:
%    pupilDiameterMM - The diameter of the pupil in mm that was used for
%                      the measurements.  This can be 7.5, 6.0, 4.5 or 3.0.
%                      If this is not passed, it defaults to 6.0.
%
% Outputs:
%    sample_mean     - Zernike mean vector
%    sample_cov      - Zernike covariance matrix
%
% References:
%    * A related paper:  Autrusseau, Thibos and Shevell (2011)
%      Vision Research
%      Chromatic and wavefront aberrations: L-, M- and S-cone stimulation
%      with typical and extreme retinal image quality
%      http://www.sciencedirect.com/science/article/pii/S0042698911003099
%
% Notes:
%    * [Note: DHB - What was measurement wavelength.  Add note about phase
%       of average zcoeffs not matching any individual observer.  NCP
%       worked this all out somewhere, we need to bring that into isetbio.]
%
% See Also:
%    s_wvfThibosModel
% 

% History:
%    xx/xx/12       Copyright Wavefront Toolbox Team, 2012
%    11/09/17  jnm  Formatting

% Examples:
%{
    measPupilDiameter = 6.0;
    [sample_mean sample_cov] = wvfLoadThibosVirtualEyes(measPupilDiameter);
%}
%{
    s3 = wvfLoadThibosVirtualEyes(3.0);
    s4 = wvfLoadThibosVirtualEyes(4.5);
    s6 = wvfLoadThibosVirtualEyes(6.0);
    s7 = wvfLoadThibosVirtualEyes(7.5);
    N = length(s3);

    vcNewGraphWin;
    plot(s3(1:N), s4(1:N), 'o');
    axis equal;
    grid on

    vcNewGraphWin;
    plot(s4(1:N), s6(1:N), 'o'); 
    axis equal; 
    grid on

    vcNewGraphWin;
    plot(s6(1:N), s7(1:N), 'o'); 
    axis equal; 
    grid on

    vcNewGraphWin;
    plot(s3(1:N), s7(1:N), 'o'); 
    axis equal; 
    grid on
%}

if notDefined('pupilDiameterMM'), pupilDiameterMM = 6; end

%%
sample_mean = [];
switch pupilDiameterMM
    case 7.5
        load('IASstats75', 'S', 'sample_mean');
    case 6.0
        load('IASstats60', 'S', 'sample_mean');
    case 4.5
        load('IASstats45', 'S', 'sample_mean');
    case 3.0
        load('IASstats30', 'S', 'sample_mean');      
    otherwise
        error('Unknown pupil size %.1f. Options are 3, 4.5, 6, 7.5.\n', ...
            pupilDiameterMM)
end

sample_cov = S;

end