import rosbag, sys, csv
import time
import string
import os #for file management make directory
import shutil #for file management, copy file

import argparse

import rospy
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#verify correct input arguments: 1 or 2
if (len(sys.argv) > 2):
	print("invalid number of arguments:   " + str(len(sys.argv)))
	print("should be 2: 'bag2csv.py' and 'bagName'")
	print("or just 1  : 'bag2csv.py'")
	sys.exit(1)
elif (len(sys.argv) == 2):
	listOfBagFiles = [sys.argv[1]]
	numberOfFiles = "1"
	print("reading only 1 bagfile: " + str(listOfBagFiles[0]))
elif (len(sys.argv) == 1):
	listOfBagFiles = [f for f in os.listdir(".") if f[-4:] == ".bag"]	#get list of only bag files in current dir.
	numberOfFiles = str(len(listOfBagFiles))
	print("reading all " + numberOfFiles + " bagfiles in current directory: \n")
	for f in listOfBagFiles:
		print(f)
	print("\n press ctrl+c in the next 2 seconds to cancel \n")
	time.sleep(2)
else:
	print("bad argument(s): " + str(sys.argv))	#shouldnt really come up
	sys.exit(1)

listOfImages = ["/realsense/color/image_raw", "/realsense_right/color/image_raw"]

#known list of image topics and unnecessary/large topics
unwantedTopics = ["/realsense/color/image_raw", 
				"/realsense/color/image_raw/compressed", 
				"/realsense/depth/image_rect_raw", 
				"/realsense/color/image_raw/theora",
				"/realsense/depth/color/points",
				"/realsense_right/color/image_raw", 
				"/realsense_right/color/image_raw/compressed", 
				"/realsense_right/depth/image_rect_raw", 
				"/realsense_right/color/image_raw/theora",
				"/realsense_right/depth/color/points",
				"/gazebo/link_states",
				"/gazebo/model_states"]

count = 0
for bagFile in listOfBagFiles:
	count += 1
	print("reading file " + str(count) + " of  " + numberOfFiles + ": " + bagFile)
	#access bag
	bag = rosbag.Bag(bagFile)
	bagContents = bag.read_messages()
	bagName = bag.filename

	#create a new directory
	folder = string.rstrip(bagName, ".bag")
	try:	#else already exists
		os.makedirs(folder)
	except:
		pass	
	# shutil.copyfile(bagName, folder + '/' + bagName)
    
	#get list of topics from the bag
	listOfTopics = []
	for topic, msg, t in bagContents:
		if (topic not in listOfTopics) and (topic not in unwantedTopics):
			listOfTopics.append(topic)
			print(topic)

	for topicName in listOfTopics:
		#Create a new CSV file for each topic
		filename = folder + '/' + string.replace(topicName, '/', '-') + '.csv'
		print(filename)
		with open(filename, 'w+') as csvfile:
			filewriter = csv.writer(csvfile, delimiter = ',')
			firstIteration = True	#allows header row
			for subtopic, msg, t in bag.read_messages(topicName):	# for each instant in time that has data for topicName
				#parse data from this instant, which is of the form of multiple lines of "Name: value\n"
				#	- put it in the form of a list of 2-element lists
				msgString = str(msg)
				msgList = string.split(msgString, '\n')
				instantaneousListOfData = []
				for nameValuePair in msgList:
					splitPair = string.split(nameValuePair, ':')
					for i in range(len(splitPair)):	#should be 0 to 1
						splitPair[i] = string.strip(splitPair[i])
					instantaneousListOfData.append(splitPair)
				#write the first row from the first element of each pair
				if firstIteration:	# header
					headers = ["rosbagTimestamp"]	#first column header
					for pair in instantaneousListOfData:
						headers.append(pair[0])
					filewriter.writerow(headers)
					firstIteration = False
				# write the value from each pair to the file
				values = [str(t)]	#first column will have rosbag timestamp
				for pair in instantaneousListOfData:
					if len(pair) > 1:
						values.append(pair[1])
				filewriter.writerow(values)
	

	bag = rosbag.Bag(bagFile, 'r')
	bagContents = bag.read_messages(topics = listOfImages)
	bagName = bag.filename
	bridge = CvBridge()


	for topic, msg, t in bagContents:
		top = string.replace(topic, '/', '-')

		metaDir = folder + '/' + top
		metaFile = folder + '/' + top + '/' + (string.replace(folder, '/', '-') + '_' + top + '_list.csv')
		# metaFile = folder + 'list.csv'
		print(metaFile)
		try:
			os.makedirs(os.path.join(folder, top))
		except:
			pass

		openMetaFile = open(metaFile,'ab')


		with openMetaFile as f:
			thewriter = csv.writer(f)
			if os.path.getsize(metaFile) == 0:
				thewriter.writerow(['topic', 'seqno','timeStamp','fileName'] )

			file = string.replace(folder, '/', '-') + '_' + top + '_' + str(msg.header.stamp) + '.jpeg'
			thewriter.writerow([str(topic), str(msg.header.seq), str(msg.header.stamp), file])   

			imFolder = folder + '/' + top


			try:	#else already exists
				os.makedirs(imFolder)
			except:
				pass
			
			try: 
				cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
				# path = r"/%s", top
				ret = cv.imwrite(os.path.join(imFolder, file), cv_img)
				if ret:
					pr = 'Wrote image ' + file + ' in ' + imFolder
					print(pr)
			except CvBridgeError:
				print(CvBridgeError)
	bag.close()

print("Done reading all " + numberOfFiles + " bag files.")

# os.remove(bagFile)
