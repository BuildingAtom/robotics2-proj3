#!/usr/bin/env python3

# not going to be a ros script, but just used to process the images that make the map

from skimage.io import imread
from skimage.io import imsave
from skimage import measure
from scipy import ndimage
import matplotlib.pyplot as plt
import numpy as np

# load the image
image = imread("./saved_maps/original_map/room.pgm")
print(image)
plt.figure()
plt.imshow(image, cmap='gray')
plt.axis('off')
plt.draw()

# get continuous regions
regions = measure.label(image)
print(regions)
plt.figure()
plt.imshow(regions, cmap='nipy_spectral')
plt.axis('off')
plt.draw()

# convert appropriately (by inspection, region 2 is the free space)
(region_ids, region_counts) = np.unique(regions, return_counts=True)
free_space_id = region_ids[np.argmax(region_counts)]
print(free_space_id)
free_space = np.zeros_like(image)
free_space[regions==free_space_id] = 255
print(free_space)
plt.figure()
plt.imshow(free_space, cmap='gray')
plt.axis('off')
plt.draw()

# save out as modified image
imsave("free_space.pgm",free_space)

## now generate the discretized configuration space
# given that the robot is .2m in radius, with the origin in the center,
# and the quantization is 0.05m to a pixel the convolution mask should be 
# 9x9 if we want to add a small buffer (8x8 +1 to describe robot)
robot = np.zeros([9,9])
for i in range(0,9):
    for j in range(0,9):
        #check for if point is inside circle centered at 4,4 with radius of 4.5
        #(expanded to get inner points of larger circle)
        #location is because of orientation of map, and filter is mirrored
        value = ((i-4)**2 + (j-4)**2 <= 4.5**2)
        robot[j,i] = value

print(robot)

#convolve it with the inverted free space to generate the configuration space
configuration_space = ndimage.convolve(~free_space, robot, mode='nearest')

#threshold anything nonzero and invert again
configuration_space[configuration_space>0] = 255
configuration_space = ~configuration_space

print(configuration_space)
plt.figure()
plt.imshow(configuration_space, cmap='gray')
plt.axis('off')
plt.draw()

# save out as modified image
imsave("configuration_space.pgm",configuration_space)

plt.show()
