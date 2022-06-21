import datetime
import random
import constants
from mesh import Mesh
import glutils
from cpe3d import Object3D
import numpy as np

class Fleur():
    def __init__(self, viewer, program3d_id, pos_x, pos_z, montMesh, fMeshStg1, fMeshStg2):

        # Sauvegarde des composants utiles
        self.viewer = viewer
        self.program3d_id = program3d_id

        # Et des constantes utiles
        self.pos_x = pos_x
        self.pos_z = pos_z

        # Mesh pour le monticule de terre
        self.montMesh = montMesh
        self.montMesh_yBase = -np.amin(montMesh.vertices, axis=0)[1]

        # Mesh pour les différents stages des fleurs
        self.fMeshStg1 = fMeshStg1
        self.fMeshStg1_yBase = -np.amin(fMeshStg1.vertices, axis=0)[1]
        self.fMeshStg2 = fMeshStg2
        self.fMeshStg2_yBase = -np.amin(fMeshStg2.vertices, axis=0)[1]

        # Textures
        self.mont_texture = glutils.load_texture('objs/monticule.jpg')
        self.fleur1_texture = glutils.load_texture('objs/fleur1.jpg')
        self.fleur2_texture = glutils.load_texture('objs/fleur2.jpg')

    # Création des instances de monticule et de pousse de fleur
        self.mont_data = Mesh.create_instance(self.montMesh, pos_x, self.montMesh_yBase, pos_z, 0.2)
        self.mont_triangles = self.montMesh.get_nb_triangles()
       #self.mont_texture = glutils.load_texture('objs/monticule.jpg')

        self.fleur_data = Mesh.create_instance(self.fMeshStg1, pos_x, self.fMeshStg1_yBase, pos_z, 0.2)
        self.fleur_triangles = self.fMeshStg1.get_nb_triangles()

    # Et push dans le viewer
        # Le monticule
        self.mont_obj = Object3D(self.mont_data[0], self.mont_triangles, self.program3d_id, self.mont_texture, self.mont_data[1])
        viewer.add_object(self.mont_obj)
        # La fleur
        self.fleur_obj = Object3D(self.fleur_data[0], self.fleur_triangles, self.program3d_id, self.fleur1_texture, self.fleur_data[1])
        viewer.add_object(self.fleur_obj)

        # Détermination des timestamps futurs pour faire pousser la plante
        delay = datetime.timedelta(seconds=random.randint(constants.MIN_SCND_STAGE_TIM, constants.MAX_SCND_STAGE_TIM))
        self.secondStageTime = datetime.datetime.now() + delay

        # Etat de pousse actuel de la plante
        self.stage = 1

    # Fonction utilisée pour faire pousser la plante
    def grow(self):
        if self.stage == 1 and datetime.datetime.now() > self.secondStageTime:
            # Passer la plante au second stage
            self.stage = 2

        # Faire pousser la plante dans le jeu
            # Supprimer l'ancienne plante
            self.viewer.del_object(obj=self.fleur_obj)
            # Ajouter la nouvelle
            self.fleur_data = Mesh.create_instance(self.fMeshStg2, self.pos_x, self.fMeshStg2_yBase, self.pos_z, 0.2)
            self.fleur_triangles = self.fMeshStg2.get_nb_triangles()
            self.fleur_obj = Object3D(self.fleur_data[0], self.fleur_triangles, self.program3d_id, self.fleur2_texture, self.fleur_data[1])
            self.viewer.add_object(self.fleur_obj)

    # Supprimer la plante (récoltée), si elle est prête à l'être
    def harvest(self):
        # Autoriser la collecte au stage 2 final
        if self.stage == 2:
            
            # Enlever le monticule et la plante de la liste d'objets
            self.viewer.del_object(obj=self.mont_obj)
            self.viewer.del_object(obj=self.fleur_obj)

            return True
        
        return False
