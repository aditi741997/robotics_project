import cv2
import os
import time
import sys
import math
import random

cascade = "/home/ubuntu/catkin_ws/src/rbx/data/haar_detectors/haarcascade_frontalface_alt2.xml"
cascade2 = "/home/ubuntu/catkin_ws/src/rbx/data/haar_detectors/haarcascade_frontalface_alt.xml"
cascade3 = "/home/ubuntu/catkin_ws/src/rbx/data/haar_detectors/haarcascade_profileface.xml"

cascades_txt = ["haarcascade_profileface.xml", "haarcascade_frontalface_alt.xml", "haarcascade_frontalface_alt2.xml", "haarcascade_eye.xml", "haarcascade_upperbody.xml", "haarcascade_fullbody.xml", "haarcascade_frontalface_default.xml", "haarcascade_smile.xml", "haarcascade_lowerbody.xml", "haarcascade_frontalface_alt_tree.xml"]

cpath = "/home/ubuntu/catkin_ws/src/rbx/data/haar_detectors/"

cs1 = cv2.CascadeClassifier(cascade)
cs2 = cv2.CascadeClassifier(cascade2)
cs3 = cv2.CascadeClassifier(cascade3)

facecascade1 = cv2.CascadeClassifier(cpath+'haarcascade_frontalface_default.xml')
facecascade2 = cv2.CascadeClassifier(cpath+"haarcascade_frontalface_alt.xml")
eyecascade = cv2.CascadeClassifier(cpath+'haarcascade_eye.xml')
smilecascade = cv2.CascadeClassifier(cpath+'haarcascade_smile.xml')

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

# smaller scaleFactor should make it slower.
haar_params2 = dict(scaleFactor = 1.1,
    minNeighbors = haar_minNeighbors,
    flags = cv2.CASCADE_DO_CANNY_PRUNING,
    minSize = (haar_minSize, haar_minSize),
    maxSize = (haar_maxSize, haar_maxSize)
    )

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

def do_haar_vw_img(ind, twice):
	#ind must be in 0-999
	fpath = "/home/ubuntu/catkin_ws/vw_face_vid7_imgs/img_" + str(ind) + ".png"
	do_haar(cv2.imread(fpath), twice)

def do_haar_cond_lfw(ind):
    fpath = "/home/ubuntu/catkin_ws/lfw_all_imgsN/img" + str(ind) + ".jpg"
    do_haar_cond(cv2.imread(fpath), False)

def do_haar_cond(image,x):
    t1 = time.time()
    (h, w) = image.shape[:2]
    faces = facecascade1.detectMultiScale(image, **haar_params)
    did2face = False
    dideyes = False
    if len(faces) == 0:
        faces = facecascade2.detectMultiScale(image, **haar_params2)
        did2face = True
    if len(faces) > 0:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        for fi in faces:
            x,y,w,h = fi
            img = cv2.rectangle(image,(x,y),(x+w,y+h),(255,0,0),2)
            roi_gray = gray[y:y+h, x:x+w]
            eyes = eyecascade.detectMultiScale(roi_gray)
            smile = smilecascade.detectMultiScale(roi_gray, minNeighbors=20, scaleFactor=1.1)
        dideyes = True
    facebox = faces[0] if len(faces) > 0 else None
    return (facebox, did2face, dideyes)

cascades = []
cascade_imghit_ct = []

def do_haar(image, twice):
	t1 = time.time()
	(h, w) = image.shape[:2]
        haar_ct = 0
        '''
        faces_csi = [x.detectMultiScale(image, **haar_params) for x in cascades]
        global cascade_imghit_ct
        cascade_imghit_ct = [cascade_imghit_ct[i]+(len(faces_csi[i])>0) for i in range(len(cascades))]
        faces_agg = filter(lambda x: len(x)>0, faces_csi)
	#faces = faces_csi[0] if (len(faces_agg) == 0) else faces_agg[0]
        if len(faces_agg) > 0:
            print((h,w), "Arr of cascade hits: ", cascade_imghit_ct)
        '''
        # Update: do face cascade, then eye cascade on each face.
        if cs1:
		faces = cs1.detectMultiScale(image, **haar_params)
	        haar_ct += 1
        #if len(faces) == 0 and cs2:
		#faces = cs2.detectMultiScale(image, **haar_params)
	        #haar_ct += 1
	#if (twice > 0) and cs2:
		#faces = cs2.detectMultiScale(image, **haar_params)
	#if len(faces) == 0 and cs3:
	 	#faces = cs3.detectMultiScale(image, **haar_params)

	if len(faces) > 0:
		face_box = faces[0]
	else:
		face_box = None
	#print "Time taken for haar:", time.time() - t1
	return (face_box,haar_ct)

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
        cpath = sys.argv[3]
        cascades = [cv2.CascadeClassifier(cpath+x) for x in cascades_txt]
        cascade_imghit_ct = [0 for x in cascades_txt]

        nc = int(sys.argv[1])
	nt = int(sys.argv[2])
	cv2.setNumThreads(nt)
	print cv2.getNumThreads(), ":cv num threads, #cores : ", cv2.getNumberOfCPUs()
	'''
        # reading .avi video :
	#cap = cv2.VideoCapture('/home/ubuntu/catkin_ws/nrc_iit_face_videos/00-1.avi')
	cap = cv2.VideoCapture('/home/ubuntu/catkin_ws/vid.avi')
	fcount = 0
	resz_time_arr = []
	while(cap.isOpened()):
		try:
			ret, frame = cap.read()
			print fcount
			t1 = time.time()
			#print frame.shape[:2]
			t1 = time.time()
			frame = cv2.resize(frame, (0,0), fx=0.5, fy=0.5)
			resz_time_arr.append(time.time() - t1)
			#print frame.shape[:2]
			do_haar(frame, True)
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
	do_haar(cv2.imread('vw_face_vid7_imgs/img_17.png'), False)
	resz_sorted = sorted(resz_time_arr)
	'''
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
	fpath = sys.argv[4] #'/home/ubuntu/catkin_ws/vw_face_vid7_imgs/'
	haar_ct_arr = []
        f1_f2_ct = 0
        f1_f2_e_ct = 0
        f1_e_ct = 0
        eyesct = 0
        f1_e_arr = []
        ci_arr = {"f1_e": [], "f1_f2_e": [], "f1_f2": []}
        ci_sq_arr = {"f1_e": [], "f1_f2_e": [], "f1_f2": []}
        for file in os.listdir(fpath):
		#do_random_haar()
		fname = fpath + file
		image = cv2.imread(fname)
		# print "Processing ", fname
		start = time.time()
		#fb,haarct = do_haar(image, False)
		fb, did2face, dideyes = do_haar_cond(image,1)
                f1_e_ct += (not did2face) and dideyes
                if (not did2face) and dideyes:
                    f1_e_arr.append(file)
                f1_f2_ct += (did2face) and not dideyes
                f1_f2_e_ct += did2face and dideyes
                tt = time.time() - start
                category = "f1_e" if ((not did2face) and dideyes) else "f1_f2" if ((did2face) and not dideyes) else "f1_f2_e"
                ci_arr[category].append(tt)
		ci_sq_arr[category].append(tt*tt)
                h_arr.append(tt)
		h_sq_arr.append(tt*tt)
		#haar_ct_arr.append(haarct)
                if fb is not None:
			print "Found face in ", fname
			face_count += 1
			face_h_arr.append(tt)
			face_h_sq_arr.append(tt*tt)

	#print "Haar found face in ", face_count, " images out of ", len(h_arr), " FEI Dataset"
        print("per cascade img split: ", cascade_imghit_ct)
        f1_e_arr_s = sorted(f1_e_arr)
        print("f1_e ARR: ", f1_e_arr_s[:100])
        for cat in ci_arr:
            print_tarr(ci_arr[cat], ci_sq_arr[cat], nc, "CI distr for "+cat)
        print("3cases split: f1_e: %i, f1_f2: %i, f1_f2_e: %i"%(f1_e_ct, f1_f2_ct, f1_f2_e_ct))
        #print("haar_ct arr : ", haar_ct_arr, "#elems =2 : ", len(filter(lambda x: x > 1, haar_ct_arr)) )
	face_haar_time = print_tarr(face_h_arr, face_h_sq_arr, nc, "Haar Detector time for IMgs with face on FEI Dataset")
	haar_total_time = print_tarr(h_arr, h_sq_arr, nc, "Haar face detector on FEI Dataset")
