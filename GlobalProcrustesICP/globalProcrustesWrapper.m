function  globalProcrustesWrapper(pcs,steps)
%GLOBALPROCRUSTESWRAPPER Summary of this function goes here
%   Detailed explanation goes here

for i=1:length(pcs)
    model(i).vertices = pcs{i}
end

[R, t, s, Centroid, corr, registeredModel] = globalProcrustes(model, steps)

save('globalIcpOut.mat','R','t','s','Centroid','corr','registeredModel')

disp('Output can be found in globalIcpOut.mat')

end
