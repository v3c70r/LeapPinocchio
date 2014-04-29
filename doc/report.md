Project Description
=============
This project implemented a Leap Motion puppet using skeleton and LBS. Leap Motion is a gesture capture device that can recognize hand shape and gestures. We can combine the hand position data and point vectors with skeleton to deform a mesh.

External Library Used
==========

Pinocchio Library for skeleton and weight generation

http://www.mit.edu/~ibaran/autorig/pinocchio.html

Leap Motion SDK

https://developer.leapmotion.com/





Key Ideas
========


1. Skeleton Embedding

The skeleton I used is a human skeleton that was implemented in the Pinocchio library. The skeleton is rigged to human like meshes. 

2. Skin Attachment

The weight is calculated by solving heat equilibrium over surface, which is much cheaper than solve in volume. The weight can be solved by following equation.
$$\mathbf{-\Delta w^i+Hw^i=Hp^i}$$
Where $\Delta$ is the cotangent Laplacian, $\mathbf{p}^i$ is a vector with $p^i_j=1$ and $\mathbf{H}$ is the diagonal matrix with $H_{jj}=c/d(j)^2$ in which $c=1$ and $d(j)$ is the distance from vertex $j$ to its nearest bone.

3. Leap Motion Deformation

From Leap Motion SDK, it is possible to get the direction of fingers. In this project, I pick three fingers as controller of meshes’ heads, left hands and right hands, and bind them to the bons. To calculate the rotation of bones, I provide an initial pose, and calculate the rotation between the current pose, and bind the rotation to 3 bones.

Performance
===============
The mesh is attached well with skeleton. Any human like mesh can be used here. However, the behavior of puppet is not quite intuitive. One reason is that the Leap Motion is not that accurate while the fingers overlap. Another reason is that the way I define rotation is not that good.

For the future work, I will optimize the interact with Leap Motion and find better bone binding. I will also provide an UI which can display multiple objs.


Reference
========
[1]Baran, Ilya, and Jovan Popović. "Automatic rigging and animation of 3d characters." ACM Transactions on Graphics (TOG). Vol. 26. No. 3. ACM, 2007.


