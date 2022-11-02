# Method
* For each axis, a set of binary patterns of increasing granularity (and their inverses) are projected and recorded by the camera.
* For each pixel, the average difference between intensities inverted frames is measured.
* Pixels whose intensity difference had a signal:noise ratio greater than a given threshold are retained
* The binary patterns are then decoded into position information within the projection
* RANSAC is then used to determine the projection matrix parameters. Random points are converted to a plausible model via Richardson iteration searching for an element of the nullspace of a homogenous matrix.
