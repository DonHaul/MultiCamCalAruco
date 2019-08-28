function plotMesh(vertices,faces,options)
% plot_mesh - plot a 3D mesh.
%
%   plot_mesh(vertices,faces, options);
%
%   'options' is a structure that may contains:
%       - 'face_color' : a float specifying the color of the faces.
%       - 'face_vertex_color' : a color per vertex or face.
%       - 'paths': geodesic curves path
%  Roberto Toldo roberto.toldo@gmail.com - Vips Laboratory University of Verona '08


if nargin<2
    error('Not enough arguments.');
end
if nargin<3
    options = [];
end


% can flip to accept data in correct ordering
if size(vertices,1)==3 && size(vertices,2)~=3
    vertices = vertices';
end
if size(faces,1)==3 && size(faces,2)~=3
    faces = faces';
end

if size(faces,2)~=3 || (size(vertices,2)~=3 && size(vertices,2)~=2)
    error('face or vertex does not have correct format.');
end


if ~isfield(options, 'face_color')
    options.face_color = 0.7;
end
face_color = options.face_color;

nverts = size(vertices,1);
if isfield(options, 'face_vertex_color')
   face_vertex_color = options.face_vertex_color;
else
    face_vertex_color = repmat(face_color,nverts,1);
end

if isfield(options, 'shading')
   shading = options.shading;
else
    shading = 'interp';
end

if isfield(options, 'EdgeColor')
   EdgeColor = options.EdgeColor;
else
    EdgeColor = [0.0; 0.0; 0.0;];
end

colormap('default')

patch('vertices',vertices,'faces',faces,'FaceVertexCData',face_vertex_color, 'FaceColor',shading,'LineWidth',0.05,'EdgeColor',EdgeColor);
lighting none;
axis equal; 
hold on;

if isfield(options, 'paths')
    paths = options.paths;
    for i=1:length(options.paths)
        p = paths{i};
        plot3(p(1,:), p(2,:), p(3,:),'r-','LineWidth',5)        
    end
end

if isfield(options, 'mark')
        plot3(vertices(options.mark,1), vertices(options.mark,2), vertices(options.mark,3),'ro','MarkerSize',10);
end
