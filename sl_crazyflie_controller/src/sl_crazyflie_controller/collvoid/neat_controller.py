from collvoid_interface import CollvoidInterface

class NEATController(CollvoidInterface):

    def __init__(self):

        print("NEATController!")

        #Implements interface
        CollvoidInterface.__init__(self)


    def calculate_velocity(self, current_target_velocity):

        print("Calculating velocity")

    def is_active(self):

        print("Is active")

        return True
