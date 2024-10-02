clear; clc; close all; format compact;

q = sym('q' , [6,1]) ;

%% Base

DH1A = [q(1) , 0 , 0 , pi/2;
       0 , 0.150 , 0.200 , 0];
  
DH2A = [q(1) , 0. , 0 , pi/2;
       0 , 0.150 , -0.100 , 0];
      
DH3A = [q(1) , 0 , 0 , pi/2;
       0 , -0.150 , 0.200 , 0];
     
DH4A = [q(1) , 0 , 0 , pi/2;
       0 , -0.150 , -0.100 , 0];
   
DH5A = [q(1) , 0.35 , 0 , pi/2;
       0 , 0.150 , 0.200 , 0];
  
DH6A = [q(1) , 0.35 , 0 , pi/2;
       0 , 0.150 , -0.100 , 0];
      
DH7A = [q(1) , 0.35 , 0 , pi/2;
       0 , -0.150 , 0.200 , 0];
     
DH8A = [q(1) , 0.35 , 0 , pi/2;
       0 , -0.150 , -0.100 , 0];

 
%% Arm1

DH1C = [q(1) , -0.10 , 0 , pi/2;
        q(3) , 0.065 , 0.150 , 0];
  
DH2C = [q(1) , -0.10 , 0 , pi/2;
        q(3) , 0.065 , 0.530 , 0];
  
DH3C = [q(1) , -0.10 , 0 , pi/2;
        q(3) , -0.065 , 0.150 , 0];
  
DH4C = [q(1) , -0.10, 0 , pi/2;
        q(3) , -0.065 , 0.530 , 0];
  
DH5C = [q(1) , 0.170 , 0 , pi/2;
        q(3) , 0.065 , 0.150 , 0];
  
DH6C = [q(1) , 0.170 , 0 , pi/2;
        q(3) , 0.065 , 0.530 , 0];
  
DH7C = [q(1) , 0.170 0 , pi/2;
        q(3) , -0.065 , 0.150 , 0];
  
DH8C = [q(1) , 0.170 , 0 , pi/2;
        q(3) , -0.065 , 0.530 , 0];

%% Arm2

DH1D = [q(1) , -0.10 , 0 , pi/2;
        q(3) , 0 , 0.500 , 0;
        q(5) , 0.045, 0 , 0];
  
DH2D = [q(1) , -0.10 , 0 , pi/2;
        q(3) , 0 , 0.500 , 0;
        q(5) , 0.180, 0 , 0];
  
DH3D = [q(1) , 0.165 , 0 , pi/2;
        q(3) , 0 , 0.500 , 0;
        q(5) , 0.045, 0 , 0];
  
DH4D = [q(1) , 0.165 , 0 , pi/2;
        q(3) , 0 , 0.500 , 0;
        q(5) , 0.180, 0 , 0];
  
DH5D = [q(1) , -0.10 , 0 , pi/2;
        q(3) , 0 , 0.500 , 0;
        q(5) , 0.045, 0.390 , 0];
  
DH6D = [q(1) , -0.10 , 0 , pi/2;
        q(3) , 0 , 0.500 , 0;
        q(5) , 0.180, 0.390 , 0];
  
DH7D = [q(1) , 0.165 , 0 , pi/2;
        q(3) , 0 , 0.500 , 0;
        q(5) , 0.045, 0.390 , 0];
  
DH8D = [q(1) , 0.165 , 0 , pi/2;
        q(3) , 0 , 0.490 , 0;
        q(5) , 0.180, 0.390 , 0];

%% Arm3

DH1E = [q(1) , .100 , 0 , pi/2;
        q(3) , 0 , 0.470 , 0;
        q(5) , 0.085, 0.30 , 0;
        -pi/2-(q(3)+q(5)) , 0.035, -0.08 , 0];
  
DH2E = [q(1) , 0.100 , 0 , pi/2;
        q(3) , 0 , 0.470 , 0;
        q(5) , 0.085, 0.30 , 0;
        -pi/2-(q(3)+q(5)) , 0.245, -0.08 , 0];
  
DH3E = [q(1) , 0.100 , 0 , pi/2;
        q(3) , 0 , 0.470 , 0;
        q(5) , 0.085, 0.45 , 0;
        -pi/2-(q(3)+q(5)) , 0.035, -0.08 , 0];
  
DH4E = [q(1) , 0.100 , 0 , pi/2;
        q(3) , 0 , 0.470 , 0;
        q(5) , 0.085, 0.45 , 0;
        -pi/2-(q(3)+q(5)) , 0.245, -0.08 , 0];
    
DH5E = [q(1) , 0.100 , 0 , pi/2;
        q(3) , 0 , 0.470 , 0;
        q(5) , 0.085, 0.30 , 0;
        -pi/2-(q(3)+q(5)) , 0.035, 0.300 , 0];
  
DH6E = [q(1) , 0.100 , 0 , pi/2;
        q(3) , 0 , 0.470 , 0;
        q(5) , 0.085, 0.30 , 0;
        -pi/2-(q(3)+q(5)) , 0.245, 0.300 , 0];
  
DH7E = [q(1) , 0.100 , 0 , pi/2;
        q(3) , 0 , 0.470 , 0;
        q(5) , 0.085, 0.45 , 0;
        -pi/2-(q(3)+q(5)) , 0.035, 0.300 , 0];
  
DH8E = [q(1) , 0.100 , 0 , pi/2;
        q(3) , 0 , 0.470 , 0;
        q(5) , 0.085, 0.45 , 0;
        -pi/2-(q(3)+q(5)) , 0.245, 0.300 , 0];
%% H parameters

% Base

H_1A  = from_DH_to_H( DH1A(1,:) );
H_10A  = from_DH_to_H( DH1A(2,:) ) ;
H_01A  = H_1A * H_10A ;  
H_2A  = from_DH_to_H( DH2A(1,:) );
H_20A  = from_DH_to_H( DH2A(2,:) ) ;
H_02A  = H_2A * H_20A ;  
H_3A  = from_DH_to_H( DH3A(1,:) );
H_30A  = from_DH_to_H( DH3A(2,:) ) ;
H_03A  = H_3A * H_30A ; 
H_4A  = from_DH_to_H( DH4A(1,:) );
H_40A  = from_DH_to_H( DH4A(2,:) ) ;
H_04A  = H_4A * H_40A ;  
H_5A  = from_DH_to_H( DH5A(1,:) );
H_50A  = from_DH_to_H( DH5A(2,:) ) ;
H_05A  = H_5A * H_50A ;  
H_6A  = from_DH_to_H( DH6A(1,:) );
H_60A  = from_DH_to_H( DH6A(2,:) ) ;
H_06A  = H_6A * H_60A ; 
H_7A  = from_DH_to_H( DH7A(1,:) );
H_70A  = from_DH_to_H( DH7A(2,:) ) ;
H_07A  = H_7A * H_70A ;  
H_8A  = from_DH_to_H( DH8A(1,:) );
H_80A  = from_DH_to_H( DH8A(2,:) ) ;
H_08A  = H_8A * H_80A ; 


% Arm1

H_1C  = from_DH_to_H( DH1C(1,:) );
H_10C  = from_DH_to_H( DH1C(2,:) ) ;
H_01C  = H_1C * H_10C ;  
H_2C  = from_DH_to_H( DH2C(1,:) );
H_20C  = from_DH_to_H( DH2C(2,:) ) ;
H_02C  = H_2C * H_20C ;  
H_3C  = from_DH_to_H( DH3C(1,:) );
H_30C  = from_DH_to_H( DH3C(2,:) ) ;
H_03C  = H_3C * H_30C ; 
H_4C  = from_DH_to_H( DH4C(1,:) );
H_40C  = from_DH_to_H( DH4C(2,:) ) ;
H_04C  = H_4C * H_40C ;  
H_5C  = from_DH_to_H( DH5C(1,:) );
H_50C  = from_DH_to_H( DH5C(2,:) ) ;
H_05C  = H_5C * H_50C ;  
H_6C  = from_DH_to_H( DH6C(1,:) );
H_60C  = from_DH_to_H( DH6C(2,:) ) ;
H_06C  = H_6C * H_60C ; 
H_7C  = from_DH_to_H( DH7C(1,:) );
H_70C  = from_DH_to_H( DH7C(2,:) ) ;
H_07C  = H_7C * H_70C ;  
H_8C  = from_DH_to_H( DH8C(1,:) );
H_80C  = from_DH_to_H( DH8C(2,:) ) ;
H_08C  = H_8C * H_80C ; 

% Arm2

H_1D  = from_DH_to_H( DH1D(1,:) );
H_10D  = from_DH_to_H( DH1D(2,:) ) ;
H_11D = from_DH_to_H( DH1D(3,:) );
H_01D  = H_1D * H_10D * H_11D;  
H_2D  = from_DH_to_H( DH2D(1,:) );
H_20D  = from_DH_to_H( DH2D(2,:) ) ;
H_12D  = from_DH_to_H( DH2D(3,:) ) ;
H_02D  = H_2D * H_20D * H_12D;  
H_3D  = from_DH_to_H( DH3D(1,:) );
H_30D  = from_DH_to_H( DH3D(2,:) ) ;
H_13D  = from_DH_to_H( DH3D(3,:) ) ;
H_03D  = H_3D * H_30D *H_13D; 
H_4D  = from_DH_to_H( DH4D(1,:) );
H_40D  = from_DH_to_H( DH4D(2,:) ) ;
H_14D  = from_DH_to_H( DH4D(3,:) ) ;
H_04D  = H_4D * H_40D * H_14D;  
H_5D  = from_DH_to_H( DH5D(1,:) );
H_50D  = from_DH_to_H( DH5D(2,:) ) ;
H_15D  = from_DH_to_H( DH5D(3,:) ) ;
H_05D  = H_5D * H_50D * H_15D ;  
H_6D  = from_DH_to_H( DH6D(1,:) );
H_60D  = from_DH_to_H( DH6D(2,:) ) ;
H_16D  = from_DH_to_H( DH6D(3,:) ) ;
H_06D  = H_6D * H_60D * H_16D ; 
H_7D  = from_DH_to_H( DH7D(1,:) );
H_70D  = from_DH_to_H( DH7D(2,:) ) ;
H_17D  = from_DH_to_H( DH7D(3,:) ) ;
H_07D  = H_7D * H_70D * H_17D ;  
H_8D  = from_DH_to_H( DH8D(1,:) );
H_80D  = from_DH_to_H( DH8D(2,:) ) ;
H_18D  = from_DH_to_H( DH8D(3,:) ) ;
H_08D  = H_8D * H_80D * H_18D ; 

% Arm3

H_1E  = from_DH_to_H( DH1E(1,:) );
H_10E  = from_DH_to_H( DH1E(2,:) ) ;
H_11E = from_DH_to_H( DH1E(3,:) );
H_21E = from_DH_to_H( DH1E(4,:) );
H_01E  = H_1E * H_10E * H_11E * H_21E;  
H_2E  = from_DH_to_H( DH2E(1,:) );
H_20E  = from_DH_to_H( DH2E(2,:) ) ;
H_12E  = from_DH_to_H( DH2E(3,:) ) ;
H_22E = from_DH_to_H( DH2E(4,:) );
H_02E  = H_2E * H_20E * H_12E * H_22E;  
H_3E  = from_DH_to_H( DH3E(1,:) );
H_30E  = from_DH_to_H( DH3E(2,:) ) ;
H_13E  = from_DH_to_H( DH3E(3,:) ) ;
H_23E = from_DH_to_H( DH3E(4,:) );
H_03E  = H_3E * H_30E * H_13E * H_23E; 
H_4E  = from_DH_to_H( DH4E(1,:) );
H_40E  = from_DH_to_H( DH4E(2,:) ) ;
H_14E  = from_DH_to_H( DH4E(3,:) ) ;
H_24E = from_DH_to_H( DH4E(4,:) );
H_04E  = H_4E * H_40E * H_14E * H_24E; 
H_5E  = from_DH_to_H( DH5E(1,:) );
H_50E  = from_DH_to_H( DH5E(2,:) ) ;
H_15E  = from_DH_to_H( DH5E(3,:) ) ;
H_25E = from_DH_to_H( DH5E(4,:) );
H_05E  = H_5E * H_50E * H_15E * H_25E;   
H_6E  = from_DH_to_H( DH6E(1,:) );
H_60E  = from_DH_to_H( DH6E(2,:) ) ;
H_16E  = from_DH_to_H( DH6E(3,:) ) ;
H_26E = from_DH_to_H( DH6E(4,:) );
H_06E  = H_6E * H_60E * H_16E * H_26E; 
H_7E  = from_DH_to_H( DH7E(1,:) );
H_70E  = from_DH_to_H( DH7E(2,:) ) ;
H_17E  = from_DH_to_H( DH7E(3,:) ) ;
H_27E = from_DH_to_H( DH7E(4,:) );
H_07E  = H_7E * H_70E * H_17E * H_27E;   
H_8E  = from_DH_to_H( DH8E(1,:) );
H_80E  = from_DH_to_H( DH8E(2,:) ) ;
H_18E  = from_DH_to_H( DH8E(3,:) ) ;
H_28E = from_DH_to_H( DH8E(4,:) );
H_08E  = H_8E * H_80E * H_18E * H_28E;
%% D parameters

% Base

d_01A  = H_01A(1:3,end)  ;
d_02A  = H_02A(1:3,end)  ;
d_03A  = H_03A(1:3,end)  ;
d_04A  = H_04A(1:3,end)  ;
d_05A  = H_05A(1:3,end)  ;
d_06A  = H_06A(1:3,end)  ;
d_07A  = H_07A(1:3,end)  ;
d_08A  = H_08A(1:3,end)  ;

% Arm1

d_01C  = H_01C(1:3,end)  ;
d_02C  = H_02C(1:3,end)  ;
d_03C  = H_03C(1:3,end)  ;
d_04C  = H_04C(1:3,end)  ;
d_05C  = H_05C(1:3,end)  ;
d_06C  = H_06C(1:3,end)  ;
d_07C  = H_07C(1:3,end)  ;
d_08C  = H_08C(1:3,end)  ;

% Arm2

d_01D  = H_01D(1:3,end)  ;
d_02D  = H_02D(1:3,end)  ;
d_03D  = H_03D(1:3,end)  ;
d_04D  = H_04D(1:3,end)  ;
d_05D  = H_05D(1:3,end)  ;
d_06D  = H_06D(1:3,end)  ;
d_07D  = H_07D(1:3,end)  ;
d_08D  = H_08D(1:3,end)  ;

% Arm3

d_01E  = H_01E(1:3,end)  ;
d_02E  = H_02E(1:3,end)  ;
d_03E  = H_03E(1:3,end)  ;
d_04E  = H_04E(1:3,end)  ;
d_05E  = H_05E(1:3,end)  ;
d_06E  = H_06E(1:3,end)  ;
d_07E  = H_07E(1:3,end)  ;
d_08E  = H_08E(1:3,end)  ;


%% Compile d

d = [d_01A,d_02A,d_03A,d_04A,d_05A,d_06A,d_07A,d_08A,d_01C,d_02C,d_03C,d_04C,d_05C,d_06C,d_07C,d_08C,d_01D,d_02D,d_03D,d_04D,d_05D,d_06D,d_07D,d_08D,d_01E,d_02E,d_03E,d_04E,d_05E,d_06E,d_07E,d_08E];   
    
matlabFunction(d, 'File','sym_arm_kinematics','Vars',{q})