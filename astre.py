class Astre():
    def __init__(self, pos_x, pos_y, pos_z, sphereMesh):
        # Mesh de l'objet
        self.object_data = Mesh.create_instance(sphereMesh, pos_x, pos_y, pos_z, 0.2)
        
        # Textures de l'objet
        self.sun_texture = glutils.load_texture('sun2.png')
        self.moon_texture = glutils.load_texture('moon2.png')

        # Position de l'objet
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.pos_z = pos_z

        # Centre de rotation, on prend le plan comme référence
        tr.rotation_center.z = tr_rotation_center_z




        pass



    def rotate(self):
        current_time = time.time()
        step = 10
        glLoadIdentity();
        glTranslated(0,0,0);
        glrotated(0.f,0.f, time * step);
        dessineObjetA();
