%% Recover the relative pose from images using QuEst
% 
% Inputs:
%
%       - imgArrayInp : Image data from onboard cameras of all UAVs
%       - imgWidthInp : Width of each image
%       - imgHeightInp: Height of each image
%       - saveIdx     : 1 to save data or 0 to not save
%       - adj         : Adjacency matrix    
%       - itr         : Iteration number of Python loop
%
% Outputs:
%
%       - Q : Relative quaternions
%       - T : Relative translations
%       - flag : Flag to indicate if pose is recovered correctly (1 = sucess, 0 = failure)
%
% (C)Kaveh Fathian, 2018.
% Email: kavehfathian@gmail.com
%
%%
function [Q, T, flag] = GetRelativePose_Ver1_7(imgArrayInp, imgWidthInp, imgHeightInp, saveIdx, adj, itr) 

% Create parallel pool in the first run to increase the computation speed
persistent flg
if isempty(flg)  % First run   
    parpool('local',3);
    flg = true; 
    
    addpath(genpath('Helpers'));
    addpath(genpath('QuEst'));
    addpath(genpath('QuEst RANSAC'));
end


%%
% Transform matrix to vector
imgArray = imgArrayInp;
disp(["numel(imgArrayInp):", numel(imgArrayInp)]);

imgWidth = double(imgWidthInp);
imgHeight = double(imgHeightInp);

% Number of pixels in each image
arrayLength = imgWidth * imgHeight;

% Number of agents
n = size(adj,1); 

% Flag to show if pose estimation was successful (it will become zero if
% there are not enough feature points)
flag = ones(1,n);  

% Reshape images into a matrix
imgs = zeros(imgHeight, imgWidth, 3, n, 'uint8');
% imgArray = reshape(imgArray, imgHeight*imgWidth*3*n);
disp(["imgHeight:", imgHeight, "imgWidth:", imgWidth, "n:", n]);
disp(["numel(imgs):", numel(imgs), "numel(imgArray):", numel(imgArray)]);
% disp(["size(imgs):", size(imgs)]);
disp(["size(imgs):", size(imgs), "size(imgArray):", size(imgArray)]);
disp(["isscalar(imgs)", isscalar(imgs), "isscalar(imgArray)", isscalar(imgArray)]);
disp(["class(imgs)", class(imgs), "class(imgArray)", class(imgArray)]);
for i = 1 : n
    % imgs(:,:,i) = reshape( imgArray((i-1)*arrayLength+1 : i*arrayLength), imgWidth, imgHeight).';
    imgs(:,:,1,i) = reshape( imgArray((i-1)*4*arrayLength+1 : 4 : i*4*arrayLength), imgWidth, imgHeight).';
    imgs(:,:,2,i) = reshape( imgArray((i-1)*4*arrayLength+2 : 4 : i*4*arrayLength), imgWidth, imgHeight).';
    imgs(:,:,3,i) = reshape( imgArray((i-1)*4*arrayLength+3 : 4 : i*4*arrayLength), imgWidth, imgHeight).';
end


%% Turn RGB to gray 

imgsg =  cell(1,n);
for i = 1 : n    
    imgsg{i} = rgb2gray(imgs(:,:,:,i));      
end


%% Camera calibration matrix

x0 = imgWidth /2;
y0 = imgHeight /2;
fx = x0;
fy = fx;

K = [fx  0  x0;
      0 fy  y0;
      0  0   1];

  
%%

surfThresh  = 200;     % SURF feature point detection threshold
% Number of feature points
minPts  = 10;       % Minimum number of feature points required (6 to estimate a unique pose from RANSAC)
maxPts  = Inf;      % Maximum number of feature points to use for pose estimation (lower value increases speed)


% Extract feature points from all images
fpd = cell(1,n); % Feature point descriptors
pts = cell(1,n); % Feature point pixel coordinats
for i = 1 : n
    % Surf feature points
    points = detectSURFFeatures(imgsg{i}, 'MetricThreshold', surfThresh);   % Detect feature points
    [fi,vpi] = extractFeatures(imgsg{i},points);    

    numPts = min(maxPts, size(fi,1));
    fpd{i} = fi(1:numPts,:);
    pts{i} = vpi.Location(1:numPts,:);
end


% Match feature points of UAVs that are neighbors
mchs = cell(n,n); % Matched point indices
for i = 1 : n-1    
    for j = i+1 : n   
        if adj(i,j)
            [indexPairs, mmetric] = matchFeatures(fpd{i}, fpd{j});  
            mchs{i,j} = indexPairs;    
            mchs{j,i} = [indexPairs(:,2), indexPairs(:,1)];            
        end
    end
end


% Use only common matched feature points
numMch = zeros(1,n);
for i = 1 : n
    % Find intersection of matched feature points for agent i
    mchi = [];
    for j = 1 : n
        if adj(i,j)                                
            if isempty(mchi)
                mchi = mchs{i,j}(:,1);
            else
                mchi = intersect(mchi, mchs{i,j}(:,1));
            end            
        end
    end
    
    % Update matched points to only include common points
    idxi = [];
    for j = 1 : n        
        if adj(i,j)                        
            % Index for common points
            [~, ~, idxi] = intersect(mchi, mchs{i,j}(:,1));
            mchs{i,j} = mchs{i,j}(idxi,:);
        end
    end
    numMch(i) = size(idxi,1); % Number of matched points in the intersection
end


% Get only the pixel positions to speed up the parallel loop
for i = 1 : n
    % If not enough matched feature points
    if numMch(i) < minPts
        flag(i) = 0;
    end
end


%% Recover relative pose using QuEst

maxZPts = 20;           % Maximum number of points used for transaltion/depth estimation (lower value increases speed)
ranThresh = 2e-6;       % RANSAC outlier threshold

% Recover relative pose
Qall = ones(4,n,n);  % Recovered quaternion
Tall = zeros(3,n,n);  % Recovered translation

parfor i = 1 : n
if flag(i) == 1  % if enough feature points are available  
    Q = zeros(4,n);  % Recovered quaternions
    T = zeros(3,n);  % Recovered translations
    for j = 1 : n
    if adj(i,j)    
    
        % Pixel coordinates of matched feature points          
        pi = pts{i}(mchs{i,j}(:,1),:);
        pj = pts{j}(mchs{i,j}(:,2),:); 
        
        % Euclidean coordinates
        numpts = size(pi,1);
        mi = K \ double( [pi, ones(numpts, 1)].' );
        mj = K \ double( [pj, ones(numpts, 1)].' );        
        
        % Recover relative rotation using QuEst RANSAC
        [M, inl] = QuEst_RANSAC_Ver1_2(mj, mi, ranThresh); 
          
        if isempty(M)  % If no good pose solution is found
            flag(i) = 0;
            break;
        end
        Q(:,j) = M.Q;                
        
        % Recover relative translation and depth up to a common scale 
        % factor using common feature points
        numPts = min(maxZPts, size(mi,2));
        [tij, z1, z2, R, Res] = FindTransDepth_Ver1_0(mj(:,1:numPts),mi(:,1:numPts), M.Q); 
        
        % Common scale
        s = mean(abs(z1));  % L_1 norm / number of common points
        
        % Translation with a common scale factor 
        T(:,j) = tij / s;
                                      
    end
    end
    
    if flag(i) == 0  % If no good pose solution is found
        continue;
    end
    
    Qall(:,:,i) = Q;
    Tall(:,:,i) = T;
end    
end


%% Generate output

Q = zeros(n^2, 4);
T = zeros(n^2, 3);

for i = 1 : n
    for j = setdiff(1:n, i)
        idx = (i-1)*n + j;
        Q(idx, :) = Qall(:,j,i).';
        T(idx, :) = Tall(:,j,i).';
    end
end


%% Save data

if saveIdx == 1
    if ~( exist('SavedData', 'dir') == 7 )
        mkdir SavedData
    end
    timestamp = datestr(now,'mmddyy_HHMMSSFFF');
    cpuTime = cputime;
%     filename = strcat('SavedData\', timestamp);
    filename = strcat('SavedData\', strcat('Data', num2str(itr)));
    save(filename)
end
























