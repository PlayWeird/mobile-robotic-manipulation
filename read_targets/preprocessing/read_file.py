#!/usr/bin/env python3

# write triangle normals and their positions to files
# reads "boat.stl" and creates the files "Triangle_normals.txt" and "Triangle_center_position.txt"
# install Open3D before using this file

import open3d as o3d
import numpy as np

if __name__ == "__main__":

    # read file with Open3D
    mesh = o3d.io.read_triangle_mesh("boat.stl")
    # get triangles to determine the corresponding vertices of each triangle
    triangles = np.asarray(mesh.triangles)    
    # get triangle normals
    triangle_normals = np.asarray(mesh.triangle_normals)
    np.savetxt("Triangle_normals.txt", triangle_normals, delimiter = ", ")
    # get vertex coordinates
    vertices = np.asarray(mesh.vertices)

    # calculate Triangle mesh coordinates
    file = open("Triangle_center_position.txt", "w")
    # take 3 rows of vertices at a time. Corresponding vertices of each triangle were determined with "triangles" parameter
    for i in range(0, len(vertices), 3):
        vertices_three_rows = vertices[i:i + 3]
        x1, x2, x3 = vertices_three_rows[0, 0], vertices_three_rows[1, 0], vertices_three_rows[2, 0]
        y1, y2, y3 = vertices_three_rows[0, 1], vertices_three_rows[1, 1], vertices_three_rows[2, 1]
        z1, z2, z3 = vertices_three_rows[0, 2], vertices_three_rows[1, 2], vertices_three_rows[2, 2]
        # calculate center of triangle
        x = (x1 + x2 + x3) / 3
        y = (y1 + y2 + y3) / 3
        z = (z1 + z2 + z3) / 3
        triangle_center_pos = np.array([[x, y, z]])
        np.savetxt(file, triangle_center_pos, delimiter = ", ")

    file.close()


