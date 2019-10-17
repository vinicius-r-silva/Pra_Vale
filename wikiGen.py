# -*- coding: UTF-8 -*-
import glob
import os


out = open("out.txt","w")

ind=0

for filename in glob.glob(os.path.join('script', '*.py')):
	arq = open(filename)
	for line in arq:
		#rospy.Subscriber('/pra_vale/arm_tilt', Float32, arm_tilt)
		#arm_publisher = rospy.Publisher('/ur5/jointsPosTargetCommand', ManipulatorJoints, queue_size=10)
		
		if(line.find("Subscriber") != -1):
			out.write("Subscriber  ")
			pos=line.find(",")
			
			out.write(line[line.find("(")+2:pos-1])
			
			out.write("->")
			out.write(line[pos+1:line.find(",",pos+1)])
			out.write(": ")
			out.write(filename)
			out.write("\n")

		elif(line.find("Publisher") != -1):
			out.write("Publisher  ")
			pos=line.find(",")
			
			out.write(line[line.find("(")+2:pos-1])
			
			out.write("->")
			out.write(line[pos+1:line.find(",",pos+1)])
			out.write(": ")
			out.write(filename)
			out.write("\n")
	out.write("\n")

for filename in glob.glob(os.path.join('src', '*.cpp')):
	arq = open(filename)
	for line in arq:
		#rospy.Subscriber('/pra_vale/arm_tilt', Float32, arm_tilt)
		#arm_publisher = rospy.Publisher('/ur5/jointsPosTargetCommand', ManipulatorJoints, queue_size=10)
		
		if(line.find("Subscriber") != -1):
			out.write("Subscriber  ")

			out.write(line[:-1])
			out.write(" ")
			out.write(filename)
			out.write("\n")

		elif(line.find("Publisher") != -1):
			out.write("Publisher  ")

			out.write(line[:-1])
			out.write(" ")
			out.write(filename)
			out.write("\n")
	out.write("\n")