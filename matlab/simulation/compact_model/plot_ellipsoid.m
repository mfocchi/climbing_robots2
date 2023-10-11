function [h ]= plot_ellipsoid(center, radii_x, radii_y,radii_z , min_z, max_z, min_y,max_y)


    radius = radii_x;
    a_y = radii_x^2/radii_y^2;
    a_z = radii_x^2/radii_z^2;    
    
    
    [Z,Y] = meshgrid(min_z:.1:max_z, min_y:0.1:max_y);
    X = sqrt(radius.^2 - a_z*(Z-center(3)).^2 -a_y*(Y-center(2)).^2);
    X = X+center(1) ;
    X(imag(X) ~= 0) = 0;
    
    h = mesh(X,Y,Z);

end