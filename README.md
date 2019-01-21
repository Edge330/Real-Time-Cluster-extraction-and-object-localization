# Real time cluster extraction and object localization

This is a simple example of how it is possible to isolate object from the scene and find its location.

Bouding boxes are used to describe orientation and translation of the object. Also bouding boxes can be used to calculate dimension of the objects (width, lenght and height).

Application works in real time using Kinect v2.

Pseudo code:

1. Passthrough, Voxel, Noise removal filtration.
2. MLS surface reconstruction.
3. Remove table's surface.
3. Euclidian cluster extraction.
4. Integrating bounding box around cluster.
5. Printout orientation and translation of the object (4x4 matrix).

![Yellow box around cluster0](https://imgur.com/a/ypI1Zyo)
