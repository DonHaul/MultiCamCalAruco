
load bunny.mat
MaxSteps = 50;
tic
[R, t, s, Centroid, corr, registeredModel] = globalProcrustes(modelSimply, MaxSteps);
toc

% Plot model before-after registration
figure(1); clf;
visModel(modelSimply);
figure(2); clf;
visModel(registeredModel);
