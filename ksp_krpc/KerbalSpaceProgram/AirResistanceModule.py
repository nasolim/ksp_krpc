class AirResistance():
    def __init__(self):
        self.area = 0
        self.coefficient_of_drag = 0.82  # coefficient for a cylinder
        self.density_of_fluid = 0
        self.velocity = 0
        self.force_of_drag = 0

    def set_area(self, value):
        '''m^s '''
        self.area = value

    def set_coefficient_of_drag(self, value):
        self.coefficient_of_drag = value

    def set_density_of_fluid(self, value):
        self.density_of_fluid = value

    def set_velocity(self, value):
        ''' m/s^2'''
        self.velocity = value

    def set_force_of_drag(self, value):
        ''' Newtons '''
        self.force_of_drag = value

    def calculate_force(self, inplace=False):
        ''' response in newtons'''
        force_of_drag =  0.5 * self.density_of_fluid * pow(self.velocity,2) * self.coefficient_of_drag * self.area
        force_of_drag = round(force_of_drag,4)
        if inplace:
            self.set_force_of_drag(force_of_drag)
        return force_of_drag

    def calculate_acceleration(self, drag, mass):
        ''' response in m / s^2 '''
        return drag / mass

    def calculate_coefficient_of_drag(self, force_of_drag, density_of_fluid, area, velocity, inplace=False):
        cd = (2 * force_of_drag) / (density_of_fluid * (velocity ** 2) * area)
        cd = round(cd,2)
        if inplace:
            self.set_coefficient_of_drag(cd)
        return cd
