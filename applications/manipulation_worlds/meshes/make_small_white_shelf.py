#!/usr/bin/env python
from make_ply import *

outfile = file("small_white_shelf.ply", 'w')

vertices = []
faces = []
add_box([0.,0.,0.], 31.4, 29.5, 1.2, vertices, faces)
add_box([0.,0.,30.], 31.4, 29.5, 1.2, vertices, faces)
add_box([0.,0.,60.], 31.4, 29.5, 1.2, vertices, faces)
add_box([0.,0.,1.2], 1.4, 29.5, 58.8, vertices, faces)
add_box([30.,0.,1.2], 1.4, 29.5, 58.8, vertices, faces)
add_box([0.,29.25,1.2], 31.4, 0.25, 58.8, vertices, faces)

#convert from cm to m
for i in range(len(vertices)):
    for j in range(3):
        vertices[i][j] *= .01

write_ply(vertices, faces, outfile)

