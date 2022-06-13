from viewerGL import ViewerGL
import glutils
from mesh import Mesh
from cpe3d import Object3D, Camera, Transformation3D, Text
import numpy as np
import OpenGL.GLUT as glut
import pyrr
import time
viewer = ViewerGL()
programGUI_id = glutils.create_program_from_file('gui.vert', 'gui.frag')
def main():
    

    viewer.set_camera(Camera())
    viewer.cam.transformation.translation.y = 2
    viewer.cam.transformation.rotation_center = viewer.cam.transformation.translation.copy()

    program3d_id = glutils.create_program_from_file('shader.vert', 'shader.frag')
    programGUI_id = glutils.create_program_from_file('gui.vert', 'gui.frag')

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
    m.normalize()
    m.apply_matrix(pyrr.matrix44.create_from_scale([0, 0, 0, 0]))
    nb_triangle = m.get_nb_triangles()
    tr_translation_y = -np.amin(m.vertices, axis=0)[1]
    # Creates vao ids and coordinates
    # create_instance(object, tr_translation_x, tr_translation_y, tr_translation_z, tr_rotation_center_z)
    object_list = []
    object_data = Mesh.create_instance(m, 0, tr_translation_y, -5, 0.2)
    object_list.append(object_data)
    texture = glutils.load_texture('cube.jpg')


    # For now, add_object must be called once for every instance
    number_of_objects = len(object_list)
    for i in range(number_of_objects):
        object_data = object_list[i]
        o = Object3D(object_data[0], nb_triangle, program3d_id, texture, object_data[1])
        viewer.add_object(o)
    
    # Création de la mesh pour les monticules de terre
    montMesh = Mesh.load_obj('monticule.obj')
    montMesh.normalize()
    montMesh.apply_matrix(pyrr.matrix44.create_from_scale([1, 1, 1, 1]))

    # Création de la mesh pour le stage 1 des fleurs
    fleurMesh = Mesh.load_obj('fleur1.obj')
    fleurMesh.normalize()
    fleurMesh.apply_matrix(pyrr.matrix44.create_from_scale([1, 1, 1, 1]))

    # Création de la mesh pour le stage 2 des fleurs
    fleurMesh = Mesh.load_obj('fleur1.obj')
    fleurMesh.normalize()
    fleurMesh.apply_matrix(pyrr.matrix44.create_from_scale([1, 1, 1, 1]))

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

    vao = Text.initalize_geometry()
    texture = glutils.load_texture('fontB.jpg')
    o = Text('00:00', np.array([0.70, 0.9], np.float32), np.array([0.97, 0.99], np.float32), vao, 2, programGUI_id, texture)
    viewer.add_object(o)

    current_time = time.time()
    glut.glutInit()
    glut.glutTimerFunc(1000, main, 0)

    vao = Text.initalize_geometry()
    texture = glutils.load_texture('fontB.jpg')
    new_time = current_time + 1
    o = Text(str(new_time), np.array([0.70, 0.9], np.float32), np.array([0.97, 0.99], np.float32), vao, 2, programGUI_id, texture)
    ViewerGL.del_object(viewer)
    ViewerGL.add_object(viewer, o)
    viewer.run()

    


    


if __name__ == '__main__':
    main()