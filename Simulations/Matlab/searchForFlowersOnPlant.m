
 function [thereIsFlower] = searchForFlowersOnPlant(img)
    %Load the image for the segmantation
%     close all;
zucc = img;%imread(img);% imshow(zucc);
thereIsFlower = 0;
%move the image from the RGB space to the CIE XYZ (Lab) one and extract the
%color plane
xyz_zucc = rgb2lab(zucc);
xz = xyz_zucc(:,:,2:3);
xz = im2single(xz);

% number of different colors to be extracted. It characterize the number of
% the final clusters
n_Clusters = 5;

%k-mean segmantation
pixel_labels = imsegkmeans(xz,n_Clusters,'NumAttempt',3);

% %shows the clusters labelled with different grays colors
% figure
% imshow(pixel_labels,[]) 
% 
% %use different masks to extract form the original image the clusters found.
mask1 = pixel_labels==1;
cluster1 = zucc .* uint8(mask1);
% figure
% imshow(cluster1)

mask2 = pixel_labels==2;
cluster2 = zucc .* uint8(mask2);
% figure
% imshow(cluster2)

mask3 = pixel_labels==3;
cluster3 = zucc .* uint8(mask3);
% figure
% imshow(cluster3)

mask4 = pixel_labels==4;
cluster4 = zucc .* uint8(mask4);
% figure
% imshow(cluster4)

mask5 = pixel_labels==5;
cluster5 = zucc .* uint8(mask5);
% figure
% imshow(cluster5)

clusters={cluster1 cluster2 cluster3 cluster4 cluster5};
masks={mask1 mask2 mask3 mask4 mask5};


% find the flower cluster
for i=1:n_Clusters
    cluster=clusters{i};
    mask=masks{i};
    
    redChannel = cluster(:, :, 1);
    greenChannel = cluster(:, :, 2);
    blueChannel = cluster(:, :, 3);
    
    meanR = mean(redChannel(mask));
    meanG = mean(greenChannel(mask));
    meanB = mean(blueChannel(mask));
    
    %fprintf('meanR: %3.f    meanG:%3.f    meanB: %3.f\n',meanR,meanG,meanB)
    if (meanR>185 && meanG>75 && meanB<30)
       thereIsFlower=1;
%        imshow(cluster);
%        title('flower cluster')
%        pause(2)
    end
end

end
