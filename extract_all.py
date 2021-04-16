import rosbag, sys, csv
import time
import string
import os #for file management make directory
import shutil #for file management, copy file

import argparse

import numpy as np
from matplotlib import pyplot as plt

import rospy
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

### From https://gist.github.com/marc-hanheide/4c35796e6a7cd0042dca274bf9e5e9f5

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

#known topic namese for RGB and DEPTH images
listOfImages = ["/realsense/color/image_raw", "/realsense_right/color/image_raw"]
listOfDepth = ["/realsense/depth/image_rect_raw", "/realsense_right/depth/image_rect_raw"]

#known list of unwanted image topics and unnecessary/large topics
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
	print("\ntopics to save as csv:")
	for topic, msg, t in bagContents:
		if (topic not in listOfTopics) and (topic not in unwantedTopics):
			listOfTopics.append(topic)
			print(topic)
	print('')

	### TOPIC EXTRACTOR
	#write topic info to csv file
	for topicName in listOfTopics:
		#Create a new CSV file for each topic
		filename = folder + '/' + string.replace(topicName, '/', '-') + '.csv'
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
		print("wrote {}".format(filename))
	print('')

	### IMAGE EXTRACTOR
	bagContents = bag.read_messages(topics = (listOfDepth + listOfImages))
	bridge = CvBridge()

	for topic, msg, t in bagContents:
		top = string.replace(topic, '/', '-')
		imFolder = folder + '/' + top
		metaFile = imFolder + '/' + (string.replace(folder, '/', '-') + '_' + top + '_list.csv')

		try: #make folder if none exists
			os.makedirs(os.path.join(folder, top))
			print("saving {} in {}".format(topic, imFolder))
		except:
			pass

		openMetaFile = open(metaFile,'ab')

		with openMetaFile as f:
			thewriter = csv.writer(f)
			if os.path.getsize(metaFile) == 0:
				thewriter.writerow(['topic', 'seqno','timeStamp','fileName'] )

			if listOfImages.__contains__(topic):
				file = string.replace(folder, '/', '-') + '_' + top + '_' + str(msg.header.stamp) + '.jpeg'
			else:
				file = string.replace(folder, '/', '-') + '_' + top + '_' + str(msg.header.stamp) + '.npy'
			
			thewriter.writerow([str(topic), str(msg.header.seq), str(msg.header.stamp), file])   

			try: 	
				if listOfImages.__contains__(topic):
					cv_img = bridge.imgmsg_to_cv2(msg,msg.encoding)
					cv.imwrite(os.path.join(imFolder, file), cv_img)
				else: # FROM https://github.com/nihalsoans91/Bag_to_Depth/blob/master/src/bag2rgbdepth/scripts/grabdepth.py
					cv_image = bridge.imgmsg_to_cv2(msg,msg.encoding)
					numpy_image= np.array(cv_image,dtype=np.uint8)
					np.save(os.path.join(imFolder, file), numpy_image)
				# pr = 'wrote image ' + file + ' in ' + imFolder
				# print(pr)
			except CvBridgeError:
				print(CvBridgeError)
	print('')
	bag.close()
	
print("done reading all " + numberOfFiles + " bag files.")

# os.remove(bagFile)
