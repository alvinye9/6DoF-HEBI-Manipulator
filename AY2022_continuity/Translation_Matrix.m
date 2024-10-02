syms th

R1 = [ 1 , 0 , 0 ;
       0 , cos(th) , -sin(th) ;
       0 , sin(th) , cos(th)  ]

R2 = [ cos(th) , 0 , sin(th) ;
       0 , 1 , 0 ;
       -sin(th), 0 , cos(th) ]
   
R3 = [ cos(th) , -sin(th), 0 ;
       sin(th) , cos(th) , 0 ;
       0, 0 , 1 ]

matlabFunction( R1 , 'File','symbolic_function/sym_R1','Vars', { th }) ;
matlabFunction( R2 , 'File','symbolic_function/sym_R2','Vars', { th }) ;
matlabFunction( R3 , 'File','symbolic_function/sym_R3','Vars', { th }) ;
        