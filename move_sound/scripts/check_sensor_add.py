#!/usr/bin/env python3

from tuning import Tuning
from std_msgs.msg import Int32, Int32MultiArray
import usb.core
import usb.util
import rospy

from geometry_msgs.msg import PoseStamped



# dev = usb.core.find(idVendor=0x2886,idProduct=0x0018, bcdDevice=3.00)
devices = tuple(usb.core.find(
    find_all=True, idVendor=0x2886, idProduct=0x0018))
for i in range(len(devices)):
    print(devices[i].bus, devices[i].address)

##############################################
dev1 = devices[0]
dev2 = devices[1]
dev3 = devices[2]
# print(dev2)
# print("==================================================================")
# print(dev1)
# dev = usb.core.find(find_all=True)
# for d in dev:
#     print(usb.util.get_string(d,d.iManufacturer))
#     print(usb.util.get_string(d,d.iProduct))
#     print(d.idProduct,d.idVendor)
if len(devices) > 1:
    Mic_tuning1 = Tuning(dev1)
    Mic_tuning2 = Tuning(dev2)
    Mic_tuning3 = Tuning(dev3)
    dir = Int32MultiArray()
    rospy.init_node('doa_node')
    pub = rospy.Publisher('Initial_goal', PoseStamped, queue_size=10)
    # rospy.loginfo(Mic_tuning.direction)
    rate = rospy.Rate(1)  # 10hz
    
    while not rospy.is_shutdown():
        
        vad1 = Mic_tuning1.is_voice()
        vad2 = Mic_tuning2.is_voice()
        vad3 = Mic_tuning3.is_voice()
        # if vad1 and vad2:
        #     print(Mic_tuning1.direction, Mic_tuning2.direction)
        if vad1:
            print("Mic1",Mic_tuning1.direction)
        if vad2:
            print("Mic2",Mic_tuning2.direction)
        if vad3:
            print("Mic3", Mic_tuning3.direction)