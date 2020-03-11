import sys
import os

f = (sys.argv[1])
fname = 'src/turtlebot3/turtlebot3_description/urdf/turtlebot3_' + sys.argv[2] + '.gazebo.xacro'
with open(fname, 'r') as fr:
    data = fr.readlines()

print "Old Freq : ", data[164]
data[164] = "        <updateRate>" + f + "</updateRate>\n"
print "New Freq : ", data[164]

with open(fname, 'w') as fr:
    fr.writelines(data)