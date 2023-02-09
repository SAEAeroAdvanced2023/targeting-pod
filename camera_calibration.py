import os

path = "./picam-pictures/" # Where to store these files relative to current path (concatenates with filename so add trailing "/")
filename = "image" # Base name of image files 
height = 480 # Height in pixels of output image
width = 640 # Width in pixels of output image
delay = 5000 # Delay between each picture (ms)
pictures = 20 # Number of pictures you want taken

for i in range(pictures):
	os.system("raspistill -o " + path + filename + str(i) + ".jpg -h " + str(height) + " -w " + str(width) + " -t " + str(delay))
