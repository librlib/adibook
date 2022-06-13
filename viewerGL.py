#!/usr/bin/env python3

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

start_time = time.time()
inventaire_state = False
map_state = False


class ViewerGL:
    def __init__(self):
        # initialisation de la librairie GLFW
        glfw.init()
        # paramétrage du context OpenGL
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
        glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, GL.GL_TRUE)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
        # création et paramétrage de la fenêtre
        glfw.window_hint(glfw.RESIZABLE, False)
        self.window = glfw.create_window(800, 800, 'OpenGL', None, None)
        # paramétrage de la fonction de gestion des évènements
        glfw.set_key_callback(self.window, self.key_callback)
        # activation du context OpenGL pour la fenêtre
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)
        # activation de la gestion de la profondeur
        GL.glEnable(GL.GL_DEPTH_TEST)
        # choix de la couleur de fond
        GL.glClearColor(0.5, 0.6, 0.9, 1.0)
        print(f"OpenGL: {GL.glGetString(GL.GL_VERSION).decode('ascii')}")

        self.objs = []
        self.touch = {}

        # Variables liées au mouvement de la souris
        self.pitch = 0
        self.jaw = -90

    def run(self):
        # boucle d'affichage
        while not glfw.window_should_close(self.window):
            # nettoyage de la fenêtre : fond et profondeur
            GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)

            self.update_key()

            for obj in self.objs:
                GL.glUseProgram(obj.program)
                if isinstance(obj, Object3D):
                    self.update_camera(obj.program)
                obj.draw()

            # changement de buffer d'affichage pour éviter un effet de scintillement
            glfw.swap_buffers(self.window)
            # gestion des évènements
            glfw.poll_events()
            
            
        
    def key_callback(self, win, key, scancode, action, mods):
        # sortie du programme si appui sur la touche 'échappement'
        if key == glfw.KEY_ESCAPE and action == glfw.PRESS:
            glfw.set_window_should_close(win, glfw.TRUE)
        self.touch[key] = action
    
    def add_object(self, obj):
        self.objs.append(obj)

    def del_object(self, object_type):
        if object_type == 'timer' :
            self.objs.pop()
        return
    

    def set_camera(self, cam):
        self.cam = cam

    def update_camera(self, prog):
        GL.glUseProgram(prog)
        # Récupère l'identifiant de la variable pour le programme courant
        loc = GL.glGetUniformLocation(prog, "translation_view")
        # Vérifie que la variable existe
        if (loc == -1) :
            print("Pas de variable uniforme : translation_view")
        # Modifie la variable pour le programme courant
        translation = -self.cam.transformation.translation
        GL.glUniform4f(loc, translation.x, translation.y, translation.z, 0)

        # Récupère l'identifiant de la variable pour le programme courant
        loc = GL.glGetUniformLocation(prog, "rotation_center_view")
        # Vérifie que la variable existe
        if (loc == -1) :
            print("Pas de variable uniforme : rotation_center_view")
        # Modifie la variable pour le programme courant
        rotation_center = self.cam.transformation.rotation_center
        GL.glUniform4f(loc, rotation_center.x, rotation_center.y, rotation_center.z, 0)

        self.cam.transformation.rotation_euler = self.objs[0].transformation.rotation_euler.copy() 
        self.cam.transformation.rotation_euler[pyrr.euler.index().yaw] += np.pi
        self.cam.transformation.rotation_center = self.objs[0].transformation.translation + self.objs[0].transformation.rotation_center
        self.cam.transformation.translation = self.objs[0].transformation.translation + pyrr.Vector3([0, 2, 5])

        rot = pyrr.matrix44.create_from_eulers(-self.cam.transformation.rotation_euler)
        loc = GL.glGetUniformLocation(prog, "rotation_view")
        if (loc == -1) :
            print("Pas de variable uniforme : rotation_view")
        GL.glUniformMatrix4fv(loc, 1, GL.GL_FALSE, rot)
    
        loc = GL.glGetUniformLocation(prog, "projection")
        if (loc == -1) :
            print("Pas de variable uniforme : projection")
        GL.glUniformMatrix4fv(loc, 1, GL.GL_FALSE, self.cam.projection)

    # def update_key(self):
    #     if glfw.KEY_UP in self.touch and self.touch[glfw.KEY_UP] > 0:
    #         self.objs[0].transformation.translation += \
    #             pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[0].transformation.rotation_euler), pyrr.Vector3([0, 0, 0.02]))
    #     if glfw.KEY_DOWN in self.touch and self.touch[glfw.KEY_DOWN] > 0:
    #         self.objs[0].transformation.translation -= \
    #             pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[0].transformation.rotation_euler), pyrr.Vector3([0, 0, 0.02]))
    #     if glfw.KEY_LEFT in self.touch and self.touch[glfw.KEY_LEFT] > 0:
    #         self.objs[0].transformation.rotation_euler[pyrr.euler.index().yaw] -= 0.1
    #     if glfw.KEY_RIGHT in self.touch and self.touch[glfw.KEY_RIGHT] > 0:
    #         self.objs[0].transformation.rotation_euler[pyrr.euler.index().yaw] += 0.1

    #     if glfw.KEY_I in self.touch and self.touch[glfw.KEY_I] > 0:
    #         self.cam.transformation.rotation_euler[pyrr.euler.index().roll] -= 0.1
    #     if glfw.KEY_K in self.touch and self.touch[glfw.KEY_K] > 0:
    #         self.cam.transformation.rotation_euler[pyrr.euler.index().roll] += 0.1
    #     if glfw.KEY_J in self.touch and self.touch[glfw.KEY_J] > 0:
    #         self.cam.transformation.rotation_euler[pyrr.euler.index().yaw] -= 0.1
    #     if glfw.KEY_L in self.touch and self.touch[glfw.KEY_L] > 0:
    #         self.cam.transformation.rotation_euler[pyrr.euler.index().yaw] += 0.1

    #     # if glfw.KEY_SPACE in self.touch and self.touch[glfw.KEY_SPACE] > 0:
    #     #     self.cam.transformation.rotation_euler = self.objs[0].transformation.rotation_euler.copy() 
    #     #     self.cam.transformation.rotation_euler[pyrr.euler.index().yaw] += np.pi
    #     #     self.cam.transformation.rotation_center = self.objs[0].transformation.translation + self.objs[0].transformation.rotation_center
    #     #     self.cam.transformation.translation = self.objs[0].transformation.translation + pyrr.Vector3([0, 1, 5])

    def update_key(self):

        # Mouvements clavier (avancer, reculer et aller sur les côtés gauche et droit)
        x_axis = 0
        z_axis = 0
        if glfw.KEY_UP in self.touch and self.touch[glfw.KEY_UP] > 0:
            z_axis += 1
        if glfw.KEY_DOWN in self.touch and self.touch[glfw.KEY_DOWN] > 0:
            z_axis -= 1
        if glfw.KEY_LEFT in self.touch and self.touch[glfw.KEY_LEFT] > 0:
            x_axis += 1
        if glfw.KEY_RIGHT in self.touch and self.touch[glfw.KEY_RIGHT] > 0:
            x_axis -= 1
        
        # S'assurer que la caméra se déplace bien à la bonne vitesse, même en mouvement diagonal
        if (x_axis != 0 and z_axis != 0):
            x_axis *= (1/sqrt(2)) * constants.PLAYER_SPEED
            z_axis *= (1/sqrt(2)) * constants.PLAYER_SPEED
        else:
            x_axis *= constants.PLAYER_SPEED
            z_axis *= constants.PLAYER_SPEED

        # Appliquer la translation du joueur dans le jeu
        self.objs[0].transformation.translation += \
            pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[0].transformation.rotation_euler), pyrr.Vector3([x_axis, 0, z_axis]))


        if glfw.KEY_I in self.touch and self.touch[glfw.KEY_I] > 0:
            self.cam.transformation.rotation_euler[pyrr.euler.index().roll] -= 0.1
        if glfw.KEY_K in self.touch and self.touch[glfw.KEY_K] > 0:
            self.cam.transformation.rotation_euler[pyrr.euler.index().roll] += 0.1
        if glfw.KEY_J in self.touch and self.touch[glfw.KEY_J] > 0:
            self.cam.transformation.rotation_euler[pyrr.euler.index().yaw] -= 0.1
        if glfw.KEY_L in self.touch and self.touch[glfw.KEY_L] > 0:
            self.cam.transformation.rotation_euler[pyrr.euler.index().yaw] += 0.1
        if start_time != 0:
            vao = Text.initalize_geometry()
            texture = glutils.load_texture('fontB.jpg')
            #print('getting into E')
            new_time = time.time()
            o = Text(str(-(start_time-new_time)), np.array([-0.10, 0.90], np.float32), np.array([0.90, 0.99], np.float32), vao, 2, programGUI_id, texture)
            ViewerGL.del_object(viewer, 'timer')
            ViewerGL.add_object(viewer, o)
        elif map_state == False :
            m = Mesh()
            p0, p1, p2, p3 = [-120, 0, -120], [120, 0, -120], [120, 0, 120], [-120, 0, 120]
            n, c = [0, 1, 0], [1, 1, 1]
            t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
            m.vertices = np.array([[p0 + n + c + t0], [p1 + n + c + t1], [p2 + n + c + t2], [p3 + n + c + t3]], np.float32)
            m.faces = np.array([[0, 1, 2], [0, 2, 3]], np.uint32)
            texture = glutils.load_texture('circuit.jpg')
            o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D())
            viewer.add_object(o)
            map_state = True
        if glfw.KEY_E in self.touch and self.touch[glfw.KEY_E] > 0:
            if inventaire_state == False:
                vao = Text.initalize_geometry()
                texture = glutils.load_texture('circuit.jpg')
                o = Text('test', np.array([-0.10, 0.90], np.float32), np.array([0.30, 0.3], np.float32), vao, 2, programGUI_id, texture)
                ViewerGL.del_object(viewer, 'timer')
                ViewerGL.add_object(viewer, o)
            return
        # if glfw.KEY_SPACE in self.touch and self.touch[glfw.KEY_SPACE] > 0:
        #     self.cam.transformation.rotation_euler = self.objs[0].transformation.rotation_euler.copy() 
        #     self.cam.transformation.rotation_euler[pyrr.euler.index().yaw] += np.pi
        #     self.cam.transformation.rotation_center = self.objs[0].transformation.translation + self.objs[0].transformation.rotation_center
        #     self.cam.transformation.translation = self.objs[0].transformation.translation + pyrr.Vector3([0, 1, 5])




viewer = ViewerGL()
programGUI_id = glutils.create_program_from_file('gui.vert', 'gui.frag')
program3d_id = glutils.create_program_from_file('shader.vert', 'shader.frag')
def main():
    

    viewer.set_camera(Camera())
    viewer.cam.transformation.translation.y = 2
    viewer.cam.transformation.rotation_center = viewer.cam.transformation.translation.copy()

    
    #programGUI_id = glutils.create_program_from_file('gui.vert', 'gui.frag')

    # m = Mesh.load_obj('stegosaurus.obj')
    # m.normalize()
    # m.apply_matrix(pyrr.matrix44.create_from_scale([2, 2, 2, 1]))
    # tr = Transformation3D()
    # tr.translation.y = -np.amin(m.vertices, axis=0)[1]
    # tr.translation.z = -5
    # tr.rotation_center.z = 0.2
    # texture = glutils.load_texture('stegosaurus.jpg')
    # o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, tr)
    # viewer.add_object(o)

    # Carré censé représenter le joueur
    # m = Mesh(.load_obj('stegosaurus.obj'))
    # m.normalize()
    # m.apply_matrix(pyrr.matrix44.create_from_scale([2, 2, 2, 1]))
    # tr = Transformation3D()
    # tr.translation.y = -np.amin(m.vertices, axis=0)[1]
    # tr.translation.z = -5
    # tr.rotation_center.z = 0.2
    # texture = glutils.load_texture('stegosaurus.jpg')
    # o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, tr)
    # viewer.add_object(o)

    m = Mesh.load_obj('cube.obj')
    m2 = Mesh.load_obj('sphere.obj')
    m.normalize()
    m2.normalize()
    m.apply_matrix(pyrr.matrix44.create_from_scale([1, 1, 1, 1]))
    m2.apply_matrix(pyrr.matrix44.create_from_scale([30, 30, 30, 30]))
    nb_triangle = m.get_nb_triangles()
    nb_triangle2 = m2.get_nb_triangles()
    tr_translation_y = -np.amin(m.vertices, axis=0)[1]
    tr_translation_y2 = -np.amin(m2.vertices, axis=0)[1]
    # Creates vao ids and coordinates
    # create_instance(object, tr_translation_x, tr_translation_y, tr_translation_z, tr_rotation_center_z)
    # y is the altitude. z is front and back. x is left or right
    object_list = []
    object_list2 = []

    # Cubes
    object_data = Mesh.create_instance(m, 0, tr_translation_y, -5, 0.2)
    object_list.append(object_data)
    object_data = Mesh.create_instance(m, 0, tr_translation_y, 0, 0.2)
    object_list.append(object_data)
    object_data = Mesh.create_instance(m, 5, tr_translation_y, 0, 0.2)
    object_list.append(object_data)
    object_data = Mesh.create_instance(m, 0, tr_translation_y, 3, 0.2)
    object_list.append(object_data)
    object_data = Mesh.create_instance(m, 5, tr_translation_y, 3, 0.2)
    object_list.append(object_data)
    object_data = Mesh.create_instance(m, 0, tr_translation_y, -3, 0.2)
    object_list.append(object_data)
    object_data = Mesh.create_instance(m, 5, tr_translation_y, -3, 0.2)
    object_list.append(object_data)

    # Spheres
    object_data = Mesh.create_instance(m2, 1, tr_translation_y2, 80, 0.2)
    object_list2.append(object_data)
    object_data = Mesh.create_instance(m2, 1, tr_translation_y2, -80, 0.2)
    object_list2.append(object_data)
    texture = glutils.load_texture('cube.jpg')
    texture2 = glutils.load_texture('sun2.png')
    texture3 = glutils.load_texture('moon2.png')


    number_of_objects = len(object_list)
    number_of_objects2 = len(object_list2)
    for i in range(number_of_objects):
        object_data = object_list[i]
        o = Object3D(object_data[0], nb_triangle, program3d_id, texture, object_data[1])
        viewer.add_object(o)

    for i in range(number_of_objects2):
        object_data = object_list2[i]
        o = Object3D(object_data[0], nb_triangle2, program3d_id, texture2, object_data[1])
        texture2 = texture3
        viewer.add_object(o)
    

    # Circuit mario
    m = Mesh()
    p0, p1, p2, p3 = [-120, 0, -120], [120, 0, -120], [120, 0, 120], [-120, 0, 120]
    n, c = [0, 1, 0], [1, 1, 1]
    t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
    m.vertices = np.array([[p0 + n + c + t0], [p1 + n + c + t1], [p2 + n + c + t2], [p3 + n + c + t3]], np.float32)
    m.faces = np.array([[0, 1, 2], [0, 2, 3]], np.uint32)
    texture = glutils.load_texture('circuit.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D())
    viewer.add_object(o)

    

    #vao = Text.initalize_geometry()
    #texture = glutils.load_texture('fontB.jpg')
    #o = Text('00:00', np.array([0.70, 0.9], np.float32), np.array([0.97, 0.99], np.float32), vao, 2, programGUI_id, texture)
    #viewer.add_object(o)

    #current_time = time.time()
    #glut.glutInit(())
    #glut.glutTimerFunc(1000, main, 0)

    #vao = Text.initalize_geometry()
    #texture = glutils.load_texture('fontB.jpg')
    #new_time = current_time + 1
    #o = Text(str(new_time), np.array([0.70, 0.9], np.float32), np.array([0.97, 0.99], np.float32), vao, 2, programGUI_id, texture)
    #ViewerGL.del_object(viewer)
    #ViewerGL.add_object(viewer, o)
    viewer.run()

    


    


if __name__ == '__main__':
    main()
    def mouse_mvt(self):
        x_offset *= constants.MOUSE_SENSIB
        y_offset *= constants.MOUSE_SENSIB

        self.jaw += x_offset
        self.pitch += y_offset

        if self.pitch > 45:
            self.pitch = 45
        if self.pitch < -45:
            self.pitch = -45
