#!/usr/bin/env python
from make_ply import *

outfile = file("sushi_shelf.ply", 'w')

vertices = []
faces = []
#vertical posts
add_box([0.,0.,0.], 8.5,1.5,174., vertices, faces)
add_box([73.4,0.,0.], 8.5,1.5,174., vertices, faces)
add_box([146.8,0.,0.],  8.5,1.5,174., vertices, faces)
add_box([0.,31.8,0.], 8.5,1.5,174., vertices, faces)
add_box([73.4,31.8,0], 8.5,1.5,174., vertices, faces)
add_box([146.8,31.8,0.], 8.5,1.5,174., vertices, faces)

#shelves
add_box([0.,2.4,4.6], 155.3,28.5,1.5, vertices, faces)
add_box([0.,2.4,55.8], 155.3,28.5,1.5, vertices, faces)
add_box([0.,2.4,94.2], 155.3,28.5,1.5, vertices, faces)
add_box([0.,2.4,171.], 155.3,28.5,1.5, vertices, faces)

#convert from cm to m
for i in range(len(vertices)):
    for j in range(3):
        vertices[i][j] *= .01

write_ply(vertices, faces, outfile)

