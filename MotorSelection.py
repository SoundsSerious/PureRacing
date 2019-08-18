# -*- coding: utf-8 -*-
"""
Created on Fri Aug  9 17:23:31 2019

@author: Nico
"""

import matplotlib.pyplot as plt


class motor(object):
    __slots__ = ['NoLoadSpeed', 'LoadSpeed','StallTorque']
    


#Input motor data most commonly available

#Worm Gear Motor ZD1633
NoLoadSpeed = 55. #RPM
StallTorque = 50. #Nm

ZD1633_x = [StallTorque,0]
ZD1633_y = [0,NoLoadSpeed]

#Bosch PN 0986337410
NoLoadSpeed = 75. #RPM
StallTorque = 70. #Nm

Bosch_x = [StallTorque,0]
Bosch_y = [0,NoLoadSpeed]

#ZD1430B 40W 24V Wiper Motor
NoLoadSpeed = 40. #RPM
StallTorque = 29. #Nm

ZD1430B_x = [StallTorque,0]
ZD1430B_y = [0,NoLoadSpeed]


fig, ax = plt.subplots(figsize=(14,10))
plt.xlabel('Torque Nm')
plt.ylabel('RPM')
plt.axis([0,80,0,80])

#plt.annotate('ZD1633', xy=(0, 55), xytext=(30, 30),
#            arrowprops=dict(facecolor='black', shrink=0.05),
#            )
#plt.annotate('Bosch', xy=(0, 75), xytext=(30, 45),
#            arrowprops=dict(facecolor='black', shrink=0.05),
#            )
#plt.annotate('ZD1430B', xy=(0, 40), xytext=(30, 15),
#            arrowprops=dict(facecolor='black', shrink=0.05),
#            )

ax.plot(ZD1633_y,ZD1633_x, 'gray', linestyle='--', marker ='', label = 'ZD1633', lw=3)
ax.plot(Bosch_y,Bosch_x, 'green', linestyle='-', marker ='', label = 'Bosch', lw=3)
ax.plot(ZD1430B_y,ZD1430B_x, 'black', linestyle=':', marker ='', label = 'ZD1430B', lw=3)
ax.plot([30],[10],'om', label = 'Requirement Point')
ax.plot([30],[5],'ok', label = 'Requirement Point (Springs)')
plt.legend()
plt.grid()
plt.title('Motor Requirements')
plt.show()
fig.savefig('MotorRequirementsPlot.png')