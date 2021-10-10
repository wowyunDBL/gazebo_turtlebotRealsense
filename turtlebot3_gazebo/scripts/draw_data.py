#!/usr/bin/env python
import matplotlib.pyplot as plt
import csv
#######################################
t1 = []
x1 = []
y1 = []
#x2 = []
#y2 = []
with open('/home/anny/wall_follow_with_PID/10/result_odom.csv', 'r') as csvfile:
  rows = csv.reader(csvfile, delimiter=',')
  for row in rows:
	t1.append(float(row[0]))
	x1.append(float(row[1]))
	y1.append(float(row[2])-5)
'''
with open('/home/anny/result_imu.csv', 'r') as csvfile:
  rows = csv.reader(csvfile, delimiter=',')
  for row in rows:
	x2.append(float(row[0]))
	y2.append(float(row[1]))
'''
fig1 = plt.figure()
plt.plot(t1,y1, label='odom', color='b', linewidth = 2)

plt.axhline(y = 1, label='reference', color='g', linestyle='-')

#plt.plot(x2,y2, label='imu')
plt.xlabel('t(s)', fontsize=18)
plt.ylabel('y(m)', fontsize=18)
#plt.xlim(0,120)
#plt.ylim(0.8,1.5)
plt.xticks(fontsize=14)
plt.title('Dis btw the wall and robot',fontsize=18)
plt.legend( loc = 'lower right', fontsize=18)
fig1.savefig("/home/anny/wall_follow_with_PID/10/odom.jpg")
plt.show()

##########################################
t1c = []
x1c = []
z1c = []
fig2 = plt.figure()
with open('/home/anny/wall_follow_with_PID/10/result_cmd.csv', 'r') as csvfile:
  rows = csv.reader(csvfile, delimiter=',')
  for row in rows:
	t1c.append(float(row[0]))
	x1c.append(float(row[1]))
	z1c.append(float(row[2]))

plt.plot(t1c,x1c, label='linear', color='g')
plt.plot(t1c,z1c, label='angular', color='orange')
plt.axhline(y = 0, label='v=0', color='black', linestyle='-')

plt.xlabel('t(s)', fontsize=18)
plt.ylabel('velocity(m/s)(rad/s)', fontsize=18)
plt.xlim(0,60)
plt.ylim(-1,1)
plt.xticks(fontsize=14)
plt.title('velocity command',fontsize=18)
plt.legend( loc = 'lower right', fontsize=18)
fig2.savefig("/home/anny/wall_follow_with_PID/10/cmd.jpg", format = 'jpg', dpi=1200)
plt.show()

#################################
'''
t1m = []
x1m = []
z1m = []
fig3 = plt.figure()
with open('/home/anny/wall_follow_with_PID/5/result_odom.csv', 'r') as csvfile:
  rows = csv.reader(csvfile, delimiter=',')
  for row in rows:
	t1m.append(float(row[0]))
	y1m.append(float(row[2])-6)
with open('/home/anny/wall_follow_with_PID/5/result_cmd.csv', 'r') as csvfile:
  rows = csv.reader(csvfile, delimiter=',')
  for row in rows:
	t1m.append(float(row[0]))
	x1m.append(float(row[1]))
	z1m.append(float(row[2]))

plt.plot(t1c,x1c, label='linear', color='g')
plt.plot(t1c,z1c, label='angular', color='orange')
plt.axhline(y = 0, label='reference', color='black', linestyle='-')

plt.xlabel('t(s)', fontsize=18)
plt.ylabel('velocity(m/s)(rad/s)', fontsize=18)
plt.xlim(0,60)
plt.ylim(-1,1)
plt.xticks(fontsize=14)
plt.title('velocity command',fontsize=18)
plt.legend( loc = 'lower right', fontsize=18)
fig2.savefig("/home/anny/wall_follow_with_PID/5/cmd.jpg", format = 'jpg', dpi=1200)
plt.show()
'''
