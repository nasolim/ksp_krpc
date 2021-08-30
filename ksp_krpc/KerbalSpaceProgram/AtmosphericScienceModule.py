# FDrag = 1/2maximum_drag x mass x density x speed2
import CraftModule
import time
import pickle
import datetime as dt


class AtmosphericSciencePackage():
    def __init__(self):
        self.craft = CraftModule.Craft()
        self.flight = self.craft.connection.space_center.active_vessel.flight(
            self.craft.connection.space_center.active_vessel.orbit.body.reference_frame)
        self.atm_readings = [('altitude(m)'
                              , 'density(kg/m^3)'
                              , 'temperature(K)'
                              , 'pressure(Pascals)'
                              , 'drag(x|N)'
                              , 'drag(y|N)'
                              , 'drag(z|N)')]

    def record(self):
        self.atm_readings.append((self.flight.surface_altitude
                                  , self.flight.atmosphere_density
                                  , self.flight.static_air_temperature
                                  , self.flight.static_pressure
                                  , self.flight.drag[0]
                                  , self.flight.drag[1]
                                  , self.flight.drag[2]))

    def activate(self, planet):
        try:
            while True:
                self.record()
                time.sleep(.5)
        #         the ship crashing does not create an exception
        except:
            with open(planet + '_atm_readings_' + dt.datetime.now().strftime("%m%d%Y_%H%M%S") + '.pkl', 'wb') as f:
                pickle.dump(self.atm_readings, f)
