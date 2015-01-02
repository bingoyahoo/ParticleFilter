ParticleFilter
==============
This is an example of a Particle Filter using colour segmentation and OpenCV that I extended from this repository https://github.com/lililqth/ParticleFilter. Particle filter is a popular technique used in robot localization. I modified some of the code especially the statistical functions, mathematical expressions and added some comments in english.

Idea behind Particle Filter
---------
This is a pretty good analogy to understand Particle Filter from Udacity.

1. Think of a big field where there is a bucket of sugar (robot) somewhere.
2. You release a lot of bees (particles) and they initially spread over the entire field.
3. The bees then realised that there is sugar on the field and get an idea(sense) about how far they are from the sugar but not what direction it is, but they can communicate with one another.
4. As the time passes(loop), the bees tend to move towards the bees that say that they are closest to the bucket(move + resampling with substitution step i.e none of the bees die)
5. While you cannot know where the bucket is, you can see where the bees are, and when they all concentrate in one big lump, you can be pretty certain that the bucket is there.
6. Otherwise, there are some really confused bees that have lost their bearings and confused the rest of the swarm.

In reality (To find x and y)
-----
Particle filters make use of a lot of probability. We make measurements and assign a higher weight to the more probable particles. We then resample the particle based on their weighted probabilities. Over time, the most consistent particles will survive and we will have successfully localized the robot.

Based on my experience, particle filters seem more robust than simple colour segmentation/blob detection when used to track robots in different lighting conditions.

How to find z coordinates with one camera?
-------
Finding z is easier with two cameras aka stereo vision. It is how humans perceive 3D objects in real life. However, if we only have one camera, we can still get a fairly good estimate of z using the following idea:

![alt tag](http://cdn-7.nikon-cdn.com/en_INC/IMG/Images/Learn-Explore/Photography-Techniques/2009/Focal-Length/Media/focal-length-graphic.jpg)

Notice that there are two similar triangles (in yellow) formed by the actual object, lens of the camera, and the image. By proportionality of similar triangles, 

`object distance/image distance = object height/image height`

When we are trying to get z, we are actually looking for the object distance from the camera. The image distance is usually known as the focal length. It is a measure of how strongly the lens converges or diverges light and remains fixed. 

What we can do to get z is to use the camera to get the image distance aka focal length first:
1. Place an object of known height at a known distance (say 30cm) away from the camera. You will get the object distance and object height in cm or m.
2. Use the camera to capture the object.
3. Find out the height of the object in the image(using OpenCV). This will be in pixels.
4. Use the formula to get the image distance aka focal length of the camera lens.
5. From now on, you can use the focal length and the formula to estimate the distance that this object is away from the camera i.e the z coordinates.

Note: Every time you change the resolution of the camera or use another camera, you have to redo this process as different cameras have different focal lengths.

Note
------
In my code, I calibrated the program to work with a GoPro camera with high fisheye distortion. If you are working with a webcam, try removing the calls to initUndistortRectifyMap and remap.
