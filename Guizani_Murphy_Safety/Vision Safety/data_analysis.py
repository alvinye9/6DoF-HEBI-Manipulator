import nums_from_string
import statistics
import matplotlib.pyplot as plt
def anal_dict (file_name ) : 
    lol= []
    empty_list = []
    with open (file_name, 'r') as in_handle : 
        for line in in_handle :
            lol.append (nums_from_string.get_nums(line))
    return lol
lol = anal_dict('test_dataa.txt')


def avg_data(list_of_list) : 
    list_1 = [] 
    list_2 = []
    list_3 = []
    list_4 = []
    list_5 = []
    list_6 = []
    list_7 = []
    list_8 = []
    s=0
     
        
    for i in list_of_list : 
        try :    
            if i is not None :
                s+=1
            
                if i[0] == 1 : 
                    list_1.append(i[1])
                if i[0] == 2 : 
                    list_2.append(i[1])
                if i[0] == 3 : 
                    list_3.append(i[1])
                if i[0] == 4: 
                    list_4.append(i[1])
                if i[0] == 5 : 
                    list_5.append(i[1])
                if i[0] == 6 : 
                    list_6.append(i[1])
                if i[0] == 7 : 
                    list_7.append(i[1])
                if i[0] == 8: 
                    list_8.append(i[1])
                    
        except :
            pass
    
    empty_list= []
    empty_list.append (statistics.fmean(list_1))
    empty_list.append (statistics.fmean(list_2))
    empty_list.append (statistics.fmean(list_3))
    empty_list.append (statistics.fmean(list_4))
    empty_list.append (statistics.fmean(list_5))
    empty_list.append (statistics.fmean(list_6))
    empty_list.append (statistics.fmean(list_7))
    empty_list.append (statistics.fmean(list_8))

    
    return empty_list 





laten_time = avg_data(lol)
nb_of_objects = [1,2,3,4,5,6,7,8]
plt.rcParams.update({'font.size': 28})
plt.figure(figsize=(4,3))
plt.plot (nb_of_objects,laten_time) 
plt.xlabel('Number of Objects',fontsize=30)
plt.ylabel('Latency Time [s]',fontsize=30)
plt.show()