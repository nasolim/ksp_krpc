class Body:
    def __init__(self):
        self.name = 'Test'
        self.information = {}

    def gravitational_constant(self):
        return self.gravitational_constant

    def radius(self):
        return self.information.radius

    def mass(self):
        return self.information.mass

    def has_atm(self):
        return self.information.atm_present

    def atm_height(self):
        if self.has_atm() is not True:
            return None
        return self.information.atm_height

    def atm_density(self):
        return 1

    def terminal_velocity(self
                          , craft_mass
                          , gravitational_acceleration
                          , atm_density
                          , coefficient_of_drag
                          , cross_sectional_area):
        value = (2 * craft_mass * gravitational_acceleration) / (
                atm_density * coefficient_of_drag * cross_sectional_area)
        return pow(value, 0.5)


# CelestialBody
# CelestialBody.mass in kg
# CelestialBody.gravitational_parameter m^3s^-2
# CelestialBody.surface_gravity ms^-2
# CelestialBody.has_atmosphere
# CelestialBody.atmosphere_depth in m
# CelestialBody.temperature_at()
# CelestialBody.density_at(alt)
# CelestialBody.pressure_at(alt)

