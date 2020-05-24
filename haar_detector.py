import cv2
import os
import time
import sys
import math
import random

cascade = "/home/ubuntu/catkin_ws/src/rbx/data/haar_detectors/haarcascade_frontalface_alt2.xml"
cs1 = cv2.CascadeClassifier(cascade)
haar_scaleFactor = 1.4
haar_minNeighbors = 3
haar_minSize = 30
haar_maxSize = 150

haar_params = dict(scaleFactor = haar_scaleFactor,
    minNeighbors = haar_minNeighbors,
    flags = cv2.CASCADE_DO_CANNY_PRUNING,
    minSize = (haar_minSize, haar_minSize),
    maxSize = (haar_maxSize, haar_maxSize)
    )
    #flags = cv2.CASCADE_SCALE_IMAGE,

print cv2.getNumThreads(), "cv num threads"

def do_random_haar():
	fpath = "/home/ubuntu/catkin_ws/src/rbx/data/images_fei/Haar_FaceDetected/"
	farr = os.listdir(fpath)
	r = random.randint(0, len(farr)-1)
	#print "Processing index ", r, ", img : ", fpath+farr[r]
	do_haar(cv2.imread(fpath+farr[r]))	

def do_haar_ob_track(ind):
	fpath = "/home/ubuntu/catkin_ws/obj_track_imgs/img_" + str(ind) + ".xml"
	#do haar on this img!
	cv_file = cv2.FileStorage(fpath, cv2.FILE_STORAGE_READ)
	img = cv_file.getNode("img").mat()
	do_haar(img)

def do_haar_vw_img(ind):
	#ind must be in 0-999
	fpath = "/home/ubuntu/catkin_ws/vw_face_vid7_imgs/img_" + str(ind) + ".png"
	do_haar(cv2.imread(fpath))

def do_haar(image):
	t1 = time.time()
	(h, w) = image.shape[:2]
	if cs1:
		faces = cs1.detectMultiScale(image, **haar_params)
	# if len(faces) == 0 and cs3:
	# 	faces = cs3.detectMultiScale(image, **haar_params)
	#if len(faces) == 0 and cs2:
	#	faces = cs2.detectMultiScale(image, **haar_params)

	if len(faces) > 0:
		face_box = faces[0]
	else:
		face_box = None
	#print "Time taken for haar:", time.time() - t1
	return face_box

def print_tarr(a, sqa, nc, ss):
	sa = sorted(a)
	l = len(sa)
	if l>0:
		ex = sum(a)/l
		print "Len of array : ", l
		print "Mean, median, tail of", ss, " on", nc, "cores :", ex, sa[l/2], sa[(95*l)/100]
		sq_arr = sorted(sqa)
		sig2 = (sum(sq_arr)/l) - ex*ex
		print ss, "on", nc, "cores, E(X2) :", sum(sq_arr)/l, ", sig2:", sig2, ", sig:", math.sqrt(sig2)
		return (ex, sa[l/2], sa[(95*l)/100], math.sqrt(sig2))
	else:
		return (0,0,0)

h_arr = []
h_sq_arr = []

nrc_h_arr = []
nrc_h_sq_arr = []

ob_h_arr = []
ob_h_sq_arr = []

face_count = 0
face_h_arr = []
face_h_sq_arr = []
# reading all imgs in a folder
fpath = "/home/ubuntu/catkin_ws/src/rbx/data/images_fei/Haar_FaceDetected/"

if __name__ == "__main__":
	nc = int(sys.argv[1])
	nt = int(sys.argv[2])
	cv2.setNumThreads(nt)
	print cv2.getNumThreads(), ":cv num threads, #cores : ", cv2.getNumberOfCPUs()
	# reading .avi video :
	#cap = cv2.VideoCapture('/home/ubuntu/catkin_ws/nrc_iit_face_videos/00-1.avi')
	cap = cv2.VideoCapture('/home/ubuntu/catkin_ws/vw_face_videos/vid.avi')
	fcount = 0
	while(cap.isOpened()):
		try:
			ret, frame = cap.read()
			print fcount
			t1 = time.time()
			#print frame.shape[:2]
			frame = cv2.resize(frame, (0,0), fx=0.5, fy=0.5)
			#print frame.shape[:2]
			do_haar(frame)
			#cv2.imwrite('vw_face_vid7_imgs/img_' + str(fcount) + '.png', frame)
			fcount += 1
			tt = time.time() - t1
			nrc_h_arr.append(tt)
			nrc_h_sq_arr.append(tt*tt)
			if fcount > 1000:
				cap.release()
		except:
			print fcount, "ERRORRRR"
	print_tarr(nrc_h_arr, nrc_h_sq_arr, nc, "Haar on nrc iit face video")
	do_haar(cv2.imread('vw_face_vid7_imgs/img_17.png'))
	'''
	ob_track_fpath = "/home/ubuntu/catkin_ws/obj_track_imgs/"
	print "Processing files in folder : ", ob_track_fpath
	for i in [1, 11, 341, 415]:
		print "For i : ", i
		do_haar_ob_track(i)
	for fil in os.listdir(ob_track_fpath):
		fname = ob_track_fpath + fil
		print "Doing file ", fname
		t1 = time.time()
		cv_file = cv2.FileStorage(fname, cv2.FILE_STORAGE_READ)
		img = cv_file.getNode("img").mat()
		cv2.imshow('image',img)
		cv2.waitKey(100)
		cv2.destroyAllWindows() 
		do_haar(img)
		cv_file.release()
		tt = time.time() - t1
		ob_h_arr.append(tt)
		ob_h_sq_arr.append(tt*tt)
	print_tarr(ob_h_arr, ob_h_sq_arr, nc, "Haar on obj track scene imgs")
	'''
	for file in os.listdir(fpath):
		#do_random_haar()
		fname = fpath + file
		image = cv2.imread(fname)
		# print "Processing ", fname
		start = time.time()
		fb = do_haar(image)
		tt = time.time() - start
		h_arr.append(tt)
		h_sq_arr.append(tt*tt)
		if fb is not None:
			print "Found face in ", fname
			face_count += 1
			face_h_arr.append(tt)
			face_h_sq_arr.append(tt*tt)

	print "Haar found face in ", face_count, " images out of ", len(h_arr), " FEI Dataset"
	face_haar_time = print_tarr(face_h_arr, face_h_sq_arr, nc, "Haar Detector time for IMgs with face on FEI Dataset")
	haar_total_time = print_tarr(h_arr, h_sq_arr, nc, "Haar face detector on FEI Dataset")
