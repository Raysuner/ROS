#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import re
from std_msgs.msg import String
class DealCmd():
    def __init__(self):
        rospy.init_node('dealcmd',anonymous=False)
        self.cmd_pub=rospy.Publisher('cmd_output',String,queue_size=10)
	#self.wakeup.publish("wake")
        #rospy.wait_for_message('xfspeech',String)
	rospy.Subscriber('/Rog_result', String, self.simplify_cmd)  
	rospy.loginfo("staring simlify words")
    def simplify_cmd(self,msg):
        string=msg.data
	print string
	a = re.compile(r'(厨房).*?(客厅)' )
	b=re.compile(r'(客厅).*?(厨房)')
	c = re.compile(r'(厨房).*?(卧室)' )
	d=re.compile (r'(卧室).*?(厨房)')
	e = re.compile(r'(厨房).*?(餐厅)' )
	f=re.compile( r'(餐厅).*?(厨房)')
	g = re.compile(r'(卧室).*?(客厅)' )
	h=re.compile (r'(客厅).*?(卧室)')
	i = re.compile(r'(餐厅).*?(客厅)' )
	j=re.compile( r'(客厅).*?(餐厅)')
	k = re.compile(r'(卧室).*?(餐厅)' )
	l=re.compile( r'(餐厅).*?(卧室)')
	line = string
	m1 = a.findall(line)
	m2 = b.findall(line)
	m3 = c.findall(line)
	m4 = d.findall(line)
	m5 = e.findall(line)
	m6 = f.findall(line)
	m7 = g.findall(line)
	m8 = h.findall(line)
	m9 = i.findall(line)
	m10 = j.findall(line)
	m11= k.findall(line)
	m12= l.findall(line)
	target = (len(m1), len(m2), len(m3), len(m4), len(m5), len(m6),len(m7),len(m8),len(m9),len(m10),len(m11),len(m12))
	print target
	try:
        	pos = target.index(1) + 1
        	if (pos == 1): m = m1
        	if (pos == 2): m = m2
        	if (pos == 3): m = m3
        	if (pos == 4): m = m4
        	if (pos == 5): m = m5
        	if (pos == 6): m = m6
        	if (pos == 7): m = m7
        	if (pos == 8): m = m8
        	if (pos == 9): m = m9
        	if (pos == 10): m = m10
        	if (pos == 11): m = m11
        	if (pos == 12): m = m12
		if(m[0][0]=="厨房" and m[0][1]=="客厅" ): self.cmd_pub.publish("kit liv")
		if(m[0][0]=="厨房" and m[0][1]=="卧室" ): self.cmd_pub.publish("kit bed")
		if(m[0][0]=="厨房" and m[0][1]=="餐厅" ): self.cmd_pub.publish("kit din")
		if(m[0][1]=="厨房" and m[0][0]=="客厅" ): self.cmd_pub.publish("liv kit")
		if(m[0][1]=="厨房" and m[0][0]=="卧室" ): self.cmd_pub.publish("bed kit")
		if(m[0][1]=="厨房" and m[0][0]=="餐厅" ): self.cmd_pub.publish("din kit")
		if(m[0][0]=="客厅" and m[0][1]=="卧室" ): self.cmd_pub.publish("liv bed")
		if(m[0][0]=="客厅" and m[0][1]=="餐厅" ): self.cmd_pub.publish("liv din")
		if(m[0][1]=="客厅" and m[0][0]=="卧室" ): self.cmd_pub.publish("bed liv")
		if(m[0][1]=="客厅" and m[0][0]=="餐厅" ): self.cmd_pub.publish("din liv")
		if(m[0][0]=="餐厅" and m[0][1]=="卧室" ): self.cmd_pub.publish("din bed")
                if(m[0][1]=="餐厅" and m[0][0]=="卧室" ): self.cmd_pub.publish("bed din")
	except :
	        self.cmd_pub.publish("FAILED")
if __name__ == '__main__': 
	DealCmd()
	rospy.spin()
