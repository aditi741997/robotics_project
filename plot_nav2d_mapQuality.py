import numpy as np
import matplotlib
#matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

from skimage import data, img_as_float
from skimage.metrics import structural_similarity as ssim
from skimage.metrics import mean_squared_error

from skimage.transform import rescale, resize, downscale_local_mean
from skimage import io

im1 = io.imread('frac3_run2_sp5_map16.pgm')
im2 = io.imread('frac2_run4_sp5_map2.pgm')
im3 = io.imread('frac1A_run2_map5.pgm')
badim = io.imread('frac9A_run1_map2.pgm')
imfull = io.imread('frac3A_run2_map8.pgm')

plt.imshow(im1, plt.cm.gray)
plt.savefig("mygraph.png")

rows, cols = im1.shape
print("Size of im1: ",rows,cols)
print("Size of im2: ", im2.shape)
print("Size of im3: ", im3.shape)

img1 = resize(im1, (520, 500) )
img2 = resize(im2, (520, 500) )
img3 = resize(im3, (520, 500) )
imgFull = resize(imfull, (520, 500) )
badimg = resize(badim, (520, 500) )

plt.imshow(img1, plt.cm.gray)
plt.savefig("mygraph1.png")

plt.imshow(img2, plt.cm.gray)
plt.savefig("mygraph2.png")

plt.imshow(img3, plt.cm.gray)
plt.savefig("mygraph3.png")

plt.imshow(imgFull, plt.cm.gray)
plt.savefig("mygraph.png")


mse_11 = mean_squared_error(img1,img1)
mse_12 = mean_squared_error(img1,img2)
mse_22 = mean_squared_error(img2,img2)
mse_23 = mean_squared_error(img2, img3)
mse_13 = mean_squared_error(img1, img3)

print("MSE Error 11: %f, 12: %f, 22: %f, 23: %f, 13: %f"%(mse_11, mse_12, mse_22, mse_23, mse_13))

ssim11 = ssim(img1, img1, data_range=img1.max()-img1.min())
ssim22 = ssim(img2, img2, data_range=img2.max()-img2.min())
ssim12 = ssim(img1, img2, data_range=img1.max()-img1.min())
ssim23 = ssim(img2, img3, data_range=img1.max()-img1.min())
ssim13 = ssim(img1, img3, data_range=img1.max()-img1.min())
print("SSIM 12: ", ssim12, "ssim11 %f, ssim22 %f, ssim23 %f, ssim13: %f"%(ssim11, ssim22, ssim23, ssim13) )

print("Comparing with FULL map:")
print("MSE 1full: %f, 2full: %f, 3full: %f, badimg-full: %f"%(mean_squared_error(img1, imgFull) ,mean_squared_error(img2, imgFull) ,mean_squared_error(img3, imgFull), mean_squared_error(badimg, imgFull) ) )
print("SSIM with full: 1f: %f, 2f: %f, 3f: %f, badimg-full %f"%( ssim(img1, imgFull, data_range=imgFull.max()-imgFull.min()), ssim(img2, imgFull, data_range=imgFull.max()-imgFull.min()), ssim(img3, imgFull, data_range=imgFull.max()-imgFull.min()), ssim(badimg, imgFull, data_range=imgFull.max()-imgFull.min()) ) )
