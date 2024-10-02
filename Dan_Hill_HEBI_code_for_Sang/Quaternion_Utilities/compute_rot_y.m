function DCM_Y = compute_rot_y(angle_rad)

  cs_angle = cos(angle_rad); 
  sn_angle = sin(angle_rad); 

  DCM_Y = [cs_angle 0 sn_angle; 0 1 0;-sn_angle 0 cs_angle];
end