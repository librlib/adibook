import datetime
import random
import constants

class Fleur():
    def __init__(self, pos_x, pos_y, montMesh, fMeshStg1, fMeshStg2):

        # Mesh pour le monticule de terre
        self.montMesh = montMesh

        # Mesh pour les différents stages des fleurs
        self.fMeshStg1 = fMeshStg1
        self.fMeshStg2 = fMeshStg2

        # Création des instances de monticule et de pousse de fleur
        self.mont_data = Mesh.create_instance(montMesh, 5, 1, -3, 0.2)
        self.fleur_data = Mesh.create_instance(fleurMesh, 5, 2, -3, 0.2)

        # Détermination des timestamps futurs pour faire pousser la plante
        delay = datetime.timedelta(seconds=random.randint(constants.MIN_SCND_STAGE_TIM, constants.MAX_SCND_STAGE_TIM))
        self.secondStageTime = datetime.now() + delay

        # Etat de pousse actuel de la plante
        self.stage = 1

        # Retourner l'obj_data pour l'ajouter ensuite à object_list
        return [self.mont_data, self.fleur_data]
        
    # Fonction utilisée pour faire pousser la plante
    def grow(self):
        if self.stage == 1 and datetime.now() > self.secondStageTime:
            # Passer la plante au second stage
            self.stage = 2

            # Faire pousser la plante dans le jeu
            
            


    # Supprimer la plante (récoltée), si elle est prête a l'être
    def harvest(self):
        # Autoriser la collecte au stage 2 final
        if self.stage == 2:
            pass

    object_data = Mesh.create_instance(fleurMesh, 5, tr_translation_y, -3, 0.2)
    object_list.append(object_data)
