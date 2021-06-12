% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

% Number of poses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% % the number of grids for 1 meter.
myResol = param.resol;
% % the origin of the map in pixels
myorigin = param.origin; 

% The initial pose is given
myPose(:,1) = param.init_pose;
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.

% occupancy value of unexplored pixels
unknown = mode(reshape(map, size(map,1)*size(map,2), 1));

% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M =  200;                      % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = repmat(myPose(:,1), [1, M]);

for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).
    
  %max_score = 0;  
  %while  max_score < 0.7*size(scanAngles,1)  
     % 1) Propagate the particles 
     P = P + .1*randn(3,size(P,2));   
     weight_P =1/size(P,2)*ones(1,size(P,2));
     
     % 2) Measurement Update 
     %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)    
     for i = 1:size(P,2)
         occ_real = ([ranges(:,j).*cos(scanAngles+P(3,i)),...  % [x_occ;y_occ]
              -ranges(:,j).*sin(scanAngles+P(3,i))] +repmat([P(1,i),P(2,i)],size(scanAngles,1),1))';
         occ_sub = ceil(myResol*occ_real) + repmat(myorigin,1,size(scanAngles,1));% [x_occ;y_occ]                 
     %   2-2) For each particle, calculate the correlation scores of the particles
         if sum(occ_sub(2,:)>size(map,1)) || sum(occ_sub(1,:)>size(map,2)) || sum(occ_sub(1,:)<=0) ...
             || sum(occ_sub(2,:)<=0)   
             score(i) = 0;
         else
             occ_index = sub2ind(size(map),occ_sub(2,:),occ_sub(1,:));   
             score(i) =  sum(map(occ_index)>unknown);     % occupy - occupy -> score 1
         end
     end
     %   2-3) Update the particle weights         
     weight_P = weight_P.*score;
     weight_P = weight_P/sum(weight_P);
     %   2-4) Choose the best particle to update the pose
     %[max_score,score_index] = max(score);  % if consider threshold of weight,hard to set it
  %end
     myPose(:,j) = sum(P.*repmat(weight_P,3,1),2);
     %myPose(:,j) = P(:,score_index);
     % 3) Resample if the effective number of particles is smaller than a threshold
     %index_remain = weight_P>=1/size(P,2);  % if score = 0, weight = 0
     %index_remain = score>=0.7*size(scanAngles,1);
     %P = P(:,index_remain);
     %weight_P = weight_P(index_remain);     
     
     thr = 100;
     num_eff = sum(weight_P)^2/sum(weight_P.^2);  % effective particle
     if num_eff < thr
        for i = 1 : size(P,2)  
            % find( ,1) return the first one meets the requirment 
            dup_index = find(rand <= cumsum(weight_P),1); % cumsum(weight_P) : 1*size(weight_P,2)
            P_new(:,i) = P(:,dup_index);   % 粒子权重大的将多得到后代
            %weight_P_new(i) = weight_P(dup_index);
        end
        P = P_new;
        %weight_P = 1/size(P,2)*ones(1,size(P,2));         
     end
     %if size(P,2) < thr         
     %    mul =  ceil(M/size(P,2)); 
     %    P = repmat(P, 1, mul);
     %    weight_P = repmat(weight_P, 1, mul);       
     %end
     score = zeros(1,size(P,2));
     % 4) Visualize the pose on the map as needed
    
 
end

end

