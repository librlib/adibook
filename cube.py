import OpenGL.GL as GL
import glfw
import OpenGL.GLUT as glut
import pyrr
import numpy as np
import time
import glutils
from mesh import Mesh
from cpe3d import Object3D, Camera, Transformation3D, Text
import constants
from math import sqrt


class Cube():
    def __init__(self, pos_x, pos_y, pos_z, cubeMesh, object_list_chara):
        # Mesh de l'objet
        self.object_data,  self.object_tr = Mesh.create_instance(cubeMesh, pos_x, pos_y, pos_z, 0.2)
        # Textures de l'objet
        self.texture = glutils.load_texture('cube.jpg')


        # Position de l'objet
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.pos_z = pos_z

        # Centre de rotation, on prend le plan comme référence
        #tr.rotation_center.z = tr_rotation_center_z

        # Nb de triangles
        self.nb_triangle = cubeMesh.get_nb_triangles()

        # Return la data dans le main
        self.return_data(object_list_chara)

    def add_viewer(self, viewer, object_list_astre, program3d_id):
        for i in range(len(object_list_astre)):
                o = Object3D(self.object_data, self.nb_triangle, program3d_id, self.texture, self.object_tr)
                viewer.add_object(o)


    def return_data(self, object_list_chara):
        object_list_chara.append(self.object_data)