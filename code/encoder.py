import math
import sys
import os
import random
from PIL import Image
from PIL import ImageOps
import numpy as np
import libs.ssim as ssim
import libs.utils as ut
import ctls.mpc as mpccontroller
import ctls.random as randomcontroller
import ctls.closed_loop as closedloopcontroller
import ctls.pid as pidcontroller
import statistics
import matplotlib.pyplot as plt
from PIL import ImageFile

def image_to_matrix(path):
        ImageFile.LOAD_TRUNCATED_IMAGES = True
	img = Image.open(str(path)) # opens the img
	img = ImageOps.grayscale(img) # converts image to a greyscale image
	img_data = img.getdata() # taking data of the image
	img_tab = np.array(img_data) # includes the rgb values of the image
	w,h = img.size # w= 720 h= 1280
	img_mat = np.reshape(img_tab, (h,w)) # its is converted into a matrix now
	return img_mat

def compute_ssim(path_a, path_b):
	matrix_a = image_to_matrix(path_a)# path a 0000064.jpg number of the image orginal 
	matrix_b = image_to_matrix(path_b)#path b frames/second/proc/random-Q0.8-F5000/00000064.jpg encoded
	# the path of that image
	return ssim.compute_ssim(matrix_a, matrix_b) # calculate ssim
	
def generate_random_configuration():
	# random quality - min or max
	if bool(random.getrandbits(1)):
		quality = 100
	else:
		quality = 1
	# random sharpen - min or max
	if bool(random.getrandbits(1)):
		sharpen = 5
	else:
		sharpen = 0
	# random noise - min or max
	if bool(random.getrandbits(1)):
		noise = 5
	else:
		noise = 0
	# return random choice
	return (quality, sharpen, noise)

def encode(i, frame_in, frame_out, quality, sharpen, noise):
	framename = str(i).zfill(8) + '.jpg'
	img_in = frame_in + '/' + framename # 0000064.jpg number of the image
	img_out = frame_out + '/' + framename # frames/second/proc/random-Q0.8-F5000/00000064.jpg
	# the path of that image

	# generating os command for conversion
	# sharpen actuator
	if sharpen != 0:
		sharpenstring = ' -sharpen ' + str(sharpen) + ' ' # it prints -sharpen 5 any num
	else:
		sharpenstring = ' '
	# noise actuator
	if noise != 0:
		noisestring = ' -noise ' + str(noise) + ' ' # it prints -noise 5 any num
	else:
		noisestring = ' '
	# command setup
	command = 'convert {file_in} -quality {quality} '.format(
			file_in = img_in, quality = quality)
	command += sharpenstring # the values above will be store in command
	command += noisestring
	command += img_out
	# executing conversion
	os.system(command)
	# computing current values of indices
	current_quality = compute_ssim(img_in, img_out) # compute ssim
	current_size = os.path.getsize(img_out)# gets the size of image out
	return (current_quality, current_size)

# -------------------------------------------------------------------

def main(args):
        ssim=[]
        size=[]

	# parsing arguments
	mode = args[1] # identify, mpc
	folder_frame_in = args[2]# frames/ONE/orig
	folder_frame_out = args[3] # frames/ONE/proc/bangbang-Q0.2-F500000
	folder_results = args[4] # results/ONE/bangbang-Q0.2-F500000
	
	setpoint_quality = float(args[5]) # similarity index
	setpoint_compression = float(args[6]) # frame size in kb
	
	# getting frames and opening result file
	path, dirs, files = os.walk(folder_frame_in).next()
	frame_count = len(files)
	final_frame = frame_count + 1 # +1 for forloop
	log = open(folder_results + '/results.csv', 'w')
	
	if mode == "mpc": # check if its mpc random or bangbang
		controller = mpccontroller.initialize_mpc()
	elif mode == "random":# if its random now controller. will only point to random
                # controller class
		controller = randomcontroller.RandomController() 
	elif mode == "closed_loop":
		controller = closedloopcontroller.ClosedLoopController()
		print "hello1"
	elif mode == "pid":
                controller = pidcontroller.PidController()
	
	# initial values for actuators
	ctl = np.matrix([[30], [0], [0]])

	sizes=[]
	for i in range(1, final_frame):
                sizes.append(i)
		# main loop
		ut.progress(i, final_frame) # display progress bar
		quality = np.round(ctl.item(0)) # 100
		sharpen = np.round(ctl.item(1)) # 0
		noise = np.round(ctl.item(2)) # 0
		# encoding the current frame
		# ssim and size is returned via encode 
		(current_quality, current_size) = \
			encode(i, folder_frame_in, folder_frame_out, quality, sharpen, noise) 
		log_line = '{i}, {quality}, {sharpen}, {noise}, {ssim}, {size}'.format(
			i = i, quality = quality, sharpen = sharpen, noise = noise,
			ssim = current_quality, size = current_size)
		print >>log, log_line
		# the values of quality and size that we give in the terminal
		setpoints = np.matrix([[setpoint_quality], [setpoint_compression]])
		#print setpoints
		# contains the current quality and size of the frame
		current_outputs = np.matrix([[current_quality], [current_size]])
		
		# computing actuator values for the next frame
		ssim.append(current_outputs.item(0))
		size.append(current_outputs.item(1))
		if mode == "mpc":
			try:
				ctl = controller.compute_u(current_outputs, setpoints)
				print current_outputs.item(1)
				print current_outputs.item(0)
				print "quality"
				print ctl.item(0)
				print "sharpness"
				print ctl.item(1)
				print "noise"
				print ctl.item(2)
			except Exception:
				pass
    
		elif mode == "random":
			ctl = controller.compute_u()
			
		elif mode == "closed_loop":
			ctl = controller.compute_u(current_outputs, setpoints)
		elif mode == "pid":
                        ctl = controller.compute_u(current_outputs, setpoints)
        
	
	print " done"

if __name__ == "__main__":
	main(sys.argv)
  
