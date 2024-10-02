# import library

import hebi
from math import pi
from time import sleep, time

#list the module and wait for 2 seconds to let it populate
lookup = hebi.Lookup()
sleep(2.0)


#define family and name of each moducle

Arm= '1DOM'
m1="mod1"
m2="mod2"

#grouping up
family = [Arm]
name = [m1,m2]

group = lookup.get_group_from_names(family,name)

if group is None:
    print('HEY ALVIN, GROUP IS NOT FOUND')
    exit(1)
    
    
group_command = hebi.GroupCommand(group.size)
group_feedback = hebi.GroupFeedback(group.size)    

# Start logging in the background
group.start_log('logs', mkdirs=True)



freq_hz = 0.5                 # [Hz]
freq = freq_hz * 2.0 * pi  # [rad / sec]
amp = pi * 0.25           # [rad] (45 degrees)

duration = 4              # [sec]
start = time()
t = time() - start

while t < duration:
    # Even though we don't use the feedback, getting feedback conveniently
    # limits the loop rate to the feedback frequency
    group.get_next_feedback(reuse_fbk=group_feedback)
    t = time() - start

    group_command.position = 0.25*t
    group.send_command(group_command)


    

# Stop logging. `log_file` contains the contents of the file
log_file = group.stop_log()

if log_file is not None:
    hebi.util.plot_logs(log_file, 'position')



