close all;
clear;  
clc;

addpath('Functions')
addpath('Classes')
addpath('Scripts')

percorsoRandom
% da qui si ha: 
% 1) xVett
% 2) yVett
% 3) uR, uL
% 4) uRe, uLe 
% 5) NNfase ---> gli errori di misura sulla fase

% Define position of the tag
tag_position = [10;100];

weights_history = zeros(nM, nPassi);
weights = ones(nM,1)/nM;

for k =1:nPassi-1
    distanzaVera = sqrt((tag_position(1,1)-xVett(k+1))^2+(tag_position(1,2)-yVett(k+1))^2);
    misuraFase = mod(-K*2*distanzaVera/100+ sigma_phi*NNfase(:,k+1),2*pi);
end




