#!/usr/bin/env python3

import OpenGL.GL as GL
import glfw
import OpenGL.GLUT as glut
import pyrr
import numpy as np
import time
import glutils
from mesh import Mesh
from astre import Astre
from cube import Cube
from cpe3d import Object3D, Camera, Transformation3D, Text
import constants
from math import sqrt

start_time = time.time()
inventaire_state = False


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
        self.planet= []

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

            for object in object_list2 :
                return
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

    def del_object(self, object_type = None):
        if object_type == 'timer' :
            self.objs.pop()
        
    

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
        
        # Création du timer et affichage
        if start_time != 0:
            vao = Text.initalize_geometry()
            texture = glutils.load_texture('fontB.jpg')
            #print('getting into E')
            new_time = time.time()
            o = Text(str(round(-(start_time-new_time),2)), np.array([-0.10, 0.85], np.float32), np.array([0.9, 0.99], np.float32), vao, 2, programGUI_id, texture)
            ViewerGL.del_object(viewer, 'timer')
            ViewerGL.add_object(viewer, o)
        
        # Création de l'inventaire lors de la pression de la touche E
        while glfw.KEY_E in self.touch and self.touch[glfw.KEY_E] > 0:
            if inventaire_state == False:
                vao = Text.initalize_geometry()
                texture = glutils.load_texture('inventory_test.jpg')
                o = Text('B', np.array([-0.10, 0.20], np.float32), np.array([0.80, 0.9], np.float32), vao, 2, programGUI_id, texture)
                ViewerGL.del_object(viewer, 'timer')
                ViewerGL.add_object(viewer, o)
            return

        # Sun's Movement 
        while glfw.KEY_R in self.touch and self.touch[glfw.KEY_R] > 0:
            #print(self.objs)
            self.objs[35].transformation.translation += \
            pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[35].transformation.rotation_euler), pyrr.Vector3([0, 0.2, -0.3]))
            #Sun = Astre(1, tr_translation_y2, 80,sphereMesh, "sun", object_list_astre)
            #object_list_astre.append(Sun)
            #Sun.add_viewer( viewer, object_list_astre, program3d_id)
            return




viewer = ViewerGL()
programGUI_id = glutils.create_program_from_file('gui.vert', 'gui.frag')
program3d_id = glutils.create_program_from_file('shader.vert', 'shader.frag')
object_list2 = []
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

    # Cubes
    cubeMesh = Mesh.load_obj('cube.obj')
    cubeMesh.normalize()
    cubeMesh.apply_matrix(pyrr.matrix44.create_from_scale([1, 1, 1, 1]))
    tr_translation_y = -np.amin(cubeMesh.vertices, axis=0)[1]
    object_list_chara = []
    Chara = Cube(0, tr_translation_y, -5,cubeMesh, object_list_chara)
    Chara2 = Cube(0, tr_translation_y, 0,cubeMesh, object_list_chara)
    Chara3 = Cube(5, tr_translation_y, 0,cubeMesh, object_list_chara)
    Chara4 = Cube(0, tr_translation_y, 3,cubeMesh, object_list_chara)

    # Spheres
    sphereMesh = Mesh.load_obj('sphere.obj')
    sphereMesh.normalize()
    sphereMesh.apply_matrix(pyrr.matrix44.create_from_scale([30, 30, 30, 30]))
    tr_translation_y2 = -np.amin(sphereMesh.vertices, axis=0)[1]
    object_list_astre = []
    Sun = Astre(1, tr_translation_y2, 80,sphereMesh, "sun", object_list_astre)
    Moon = Astre(1, tr_translation_y2, -80,sphereMesh, "moon", object_list_astre)

    # Every chara
    
    object_list_chara.append(Chara)
    object_list_chara.append(Chara2)
    object_list_chara.append(Chara3)
    object_list_chara.append(Chara4)

    # Every astre
    
    object_list_astre.append(Sun)
    object_list_astre.append(Moon)


    # Creates vao ids and coordinates
    # create_instance(object, tr_translation_x, tr_translation_y, tr_translation_z, tr_rotation_center_z)
    # y is the altitude. z is front and back. x is left or right

    

    # Add to viewer
    Chara.add_viewer( viewer, object_list_chara, program3d_id)
    Chara2.add_viewer( viewer, object_list_chara, program3d_id)
    Chara3.add_viewer( viewer, object_list_chara, program3d_id)
    Chara4.add_viewer( viewer, object_list_chara, program3d_id)
    Sun.add_viewer( viewer, object_list_astre, program3d_id)
    Moon.add_viewer( viewer, object_list_astre, program3d_id)
    


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
