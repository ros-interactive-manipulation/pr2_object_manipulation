#!/usr/bin/env python

#write a ply file for a list of vertices and faces
def write_ply(vertices, faces, outfile):
    vertex_count = len(vertices)
    face_count = len(faces)
    outfile.write("ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nelement face %d\nproperty list uchar int vertex_index\nend_header\n"%(vertex_count, face_count))
    for vertex in vertices:
        outfile.write("%0.5f %0.5f %0.5f\n"%(vertex[0], vertex[1], vertex[2]))
    for face in faces:
        outfile.write("%d "%len(face))
        for index in face:
            outfile.write("%d "%index)
        outfile.write("\n")

#generate vertices and faces for a box
def add_box(corner, x_len, y_len, z_len, vertices, faces):
    box_vertices = [[corner[0], corner[1], corner[2]],
                [corner[0]+x_len, corner[1], corner[2]],
                [corner[0]+x_len, corner[1]+y_len, corner[2]],
                [corner[0], corner[1]+y_len, corner[2]],
                [corner[0], corner[1], corner[2]+z_len],
                [corner[0]+x_len, corner[1], corner[2]+z_len],
                [corner[0]+x_len, corner[1]+y_len, corner[2]+z_len],
                [corner[0], corner[1]+y_len, corner[2]+z_len]]
    box_faces = [[0,2,1],[0,1,2],
             [0,3,2],[0,2,3],
             [0,1,4],[0,4,1],
             [1,5,4],[1,4,5],
             [1,2,6],[1,6,2],
             [1,6,5],[1,5,6],
             [0,4,7],[0,7,4],
             [0,7,3],[0,3,7],
             [3,2,7],[3,7,2],
             [2,6,7],[2,7,6],
             [4,5,6],[4,6,5],
             [4,6,7],[4,7,6]]
    for i in range(len(box_faces)):
        for j in range(3):
            box_faces[i][j] += len(vertices)
    vertices.extend(box_vertices)
    faces.extend(box_faces)

