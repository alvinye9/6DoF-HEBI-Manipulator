syms thetaz_i dz_i ax_i alphax_i

H_i = [ cos(thetaz_i), (-sin(thetaz_i)*cos(alphax_i)), (sin(thetaz_i)*sin(alphax_i)), (ax_i*cos(thetaz_i));
        sin(thetaz_i), (cos(thetaz_i)*cos(alphax_i)), (-cos(thetaz_i)*sin(alphax_i)), (ax_i*sin(thetaz_i));
        0, sin(alphax_i), cos(alphax_i), dz_i ;
        0, 0, 0, 1]
    
 DH_i = [thetaz_i dz_i ax_i alphax_i];
 
 matlabFunction(H_i, 'File','symbolic_function/from_DH_to_H','Vars',{DH_i})
 
 