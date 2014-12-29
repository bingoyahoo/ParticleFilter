ParticleFilter
==============
This is an example of a Particle Filter using colour segmentation and OpenCV that I extended from this repository. Particle filter is a popular technique used in robot localization. I modified some of the code especially the statistical functions, mathematical expressions and added some comments in english.

Idea behind Particle Filter
---------
This is a pretty good analogy to understand Particle Filter from the Udacity forums.

1. Think of a big field where there is a bucket of sugar (robot) somewhere.
2. You release a lot of bees (particles) and they initially spread over the entire field.
3. The bees then realised that there is sugar on the field and get an idea(sense) about how far they are from the sugar but not what direction it is, but they can communicate with one another.
4. As the time passes(loop), the bees tend to move towards the bees that say that they are closest to the bucket(move + resampling with substitution step i.e none of the bees die)
5. While you cannot know where the bucket is, you can see where the bees are, and when they all concentrate in one big lump, you can be pretty certain that the bucket is there.
6. Otherwise, there are some really confused bees that have lost their bearings and confused the rest of the swarm.

In reality
-----
Particle filters make use of a lot of probability. We make measurements and assign a higher weight to the more probable particles. We then resample the particle based on their weighted probabilities. Over time, the most consistent particles will survive and we will have successfully localized our robot.

Based on my experience, particle filters seem more robost than simple colour segmentation/blob detection when used to track robots in different lighting conditions.

Note
------
In my code, I calibrated the program to work with a GoPro camera. If you are working with a webcam, try removing the calls to initUndistortRectifyMap and remap.
