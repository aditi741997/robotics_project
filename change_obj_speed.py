import sys
import os

# JUst need to change the time stamps of the end points
new_var = sys.argv[1]
tf = float(new_var)

fname = 'src/turtlebot3_simulations/turtlebot3_gazebo/worlds/empty.world'
with open(fname, 'r') as fr:
    data = fr.readlines()

if sys.argv[2] == '1':
    # human moving at fixed speed, TODO : change pose as well
    # square path now. change 8lines :
    t_arr = [29, 33, 37, 41, 45, 49, 53, 57, 61]
    for i in range(len(t_arr)):
        data[t_arr[i]] = "            <time>" + str(tf*(i)) + "</time>\n"
        print t_arr[i], data[t_arr[i]]

    '''
    data[33] = "            <time>" + new_var + "</time>\n"
    print "Point 2 Reach Time", data[33]

    data[37] = "            <time>" + str(tf+0.25) + "</time>\n"
    print "Point 2 Leave Time", data[37]

    data[41] = "            <time>" + str(0.25+2*tf) + "</time>\n"
    print "Point 1 Reach Time", data[41]

    data[45] = "            <time>" + str(2*tf+0.5) + "</time>\n"
    print "Point 1 Leave Again Time", data[45]
    '''

elif sys.argv[2] == '2':
    # jumping human : tf is half of length jumped, should be <2
    data[30] = "            <pose>3 %f 0.7 0 0 3.14</pose>\n"%(tf) #at t=0

    data[33] = "            <time>7</time>\n"
    data[34] = "            <pose>3 %f 0.7 0 0 3.14</pose>\n"%(tf) #at t=0.5

    data[37] = "            <time>7.25</time>\n"
    data[38] = "            <pose>3 %f 0.7 0 0 3.14</pose>\n"%(-1*tf) #at t=1.5

    data[41] = "            <time>14.25</time>\n"
    data[42] = "            <pose>3 %f 0.7 0 0 3.14</pose>\n"%(-1*tf) #at t=2

    data[45] = "            <time>14.5</time>\n"
    data[46] = "            <pose>3 %f 0.7 0 0 3.14</pose>\n"%(tf) #at t=3

with open(fname, 'w') as fw:
    fw.writelines(data)
