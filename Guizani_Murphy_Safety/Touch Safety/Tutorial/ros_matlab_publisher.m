clear ; close all; format compact ; clc; 
matlab_talker = rospublisher ("chatter_mat", "std_msgs/Float64MultiArray", "DataFormat", "struct") ; 
matlab_message = rosmessage(matlab_talker) 
while true 
    [time,issimtime] = rostime ('now') ;
    ROS_time = time.Sec+time.Nsec/1e9 ; 
    matrix = [1.0 2.0 3.0 4.0; 5.0 6.0 7.0 8.0; 9.0 10.0 11.0 12.0 ;  13.0 14.0 15.0 ROS_time] ; 
    matrix_flat= reshape (matrix', 1,16) ;
    matlab_message.Data= matrix_flat; 
    send (matlab_talker,matlab_message) 
end
    


