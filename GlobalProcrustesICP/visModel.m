function visModel(model)
clf;
for i=1:length(model)

   color = chooseColor(i);
   plot3(model(i).vertices(:,1),model(i).vertices(:,2), model(i).vertices(:,3),color, 'Markersize',5);
   hold on;
end

axis equal

end

%%
function color = chooseColor(i)
        switch mod(i,7)+1
                case 1
                    color = 'b.';
                case 2
                    color = 'g.';
                case 3
                    color = 'r.';
                case 4
                    color = 'c.';
                case 5
                    color = 'm.';
                case 6
                    color = 'y.';
                case 7
                    color = 'k.';                
        end    
end