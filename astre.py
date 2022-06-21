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
from math import sqrt, cos, sin 


class Astre():
    def __init__(self, pos_x, pos_y, pos_z, sphereMesh, Name, object_list_astre):
        # Mesh de l'objet
        self.object_data,  self.object_tr = Mesh.create_instance(sphereMesh, pos_x, pos_y, pos_z, 0.2)
        # Textures de l'objet
        self.name = Name
        if self.name == "sun":
            self.texture = glutils.load_texture('sun2.png')
        else:
            self.texture = glutils.load_texture('moon2.png')

        # Position de l'objet
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.pos_z = pos_z
        self.pos_z_new = 0
        self.pos_y_new = 0

        # Centre de rotation, on prend le plan comme référence
        # + Angle de rotation
        #tr.rotation_center.z = tr_rotation_center_z
        self.angle = 0
        # Nb de Triangles
        self.nb_triangle = sphereMesh.get_nb_triangles()

        # Return la data danbs le main
        self.return_data(object_list_astre)

    def add_viewer(self, viewer, object_list_astre, program3d_id):
        for i in range(len(object_list_astre)):
                o = Object3D(self.object_data, self.nb_triangle, program3d_id, self.texture, self.object_tr)
                viewer.add_object(o)


    def return_data(self, object_list_astre):
        object_list_astre.append(self.object_data)
    #def rotate(self):
    #    current_time = time.time()
    #    step = 10
    #    glLoadIdentity();
    #    glTranslated(0,0,0);
    #    glrotated(0.f,0.f, time * step);
    #    dessineObjetA();

    def rotation(self):
        if self.angle <= 90 :
            self.angle += 1
            self.pos_z_new = 80*cos(self.angle + (self.angle+90))
            self.pos_y_new = 80*sin(self.angle + (self.angle+90))

            self.pos_z -= self.pos_z_new
            if self.pos_y != 0:
                
                self.pos_y_new -= self.pos_y
            else:
                self.pos_y = self.pos_y_new
            return -self.pos_z, self.pos_y

        if self.angle >= 90 and self.angle <= 180:
            self.angle += 1
            self.pos_z_new = 80*cos(self.angle + (self.angle+90))
            self.pos_y_new = 80*sin(self.angle + (self.angle+90))

            self.pos_y -= self.pos_y_new
            if self.pos_z != 0:
                self.pos_z += self.pos_z_new
                
            else :
                self.pos_z = self.pos_z_new
            return -self.pos_z, -self.pos_y

        if self.angle >= 180 and self.angle <= 270:
            self.angle += 1
            self.pos_z_new = 80*cos(self.angle + (self.angle+90))
            self.pos_y_new = 80*sin(self.angle + (self.angle+90))

            self.pos_z -= self.pos_z_new
            if self.pos_y != 0:
                
                self.pos_y -= self.pos_y_new
            else:
                self.pos_y = self.pos_y_new
            return -self.pos_z, -self.pos_y

        if self.angle >= 270 :
            self.angle += 1
            self.pos_z_new = 80*cos(self.angle + (self.angle+90))
            self.pos_y_new = 80*sin(self.angle + (self.angle+90))

            self.pos_y -= self.pos_y_new
            if self.pos_z != 0:
                self.pos_z -= self.pos_z_new
            else:
                self.pos_z = self.pos_z_new
            
            return -self.pos_z, -self.pos_y




