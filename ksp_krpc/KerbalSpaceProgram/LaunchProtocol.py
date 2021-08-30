import CraftModule
import time
import PIDController as pid


class LaunchProfile(CraftModule.Craft):
    def __init__(self, altitude):
        CraftModule.Craft.__init__(self)
        self.active_vessel()
        self.speed_min = 100
        self.speed_step = 50
        self.pitch_max = 90
        self.pitch_min = 10
        self.pitch_step = 5
        self.pitch = 90
        self.heading = 0
        self.throttle = 0  # this can be collected from the parent
        self.altitude = altitude
        self.timestep = 0.001
        self.launch_velocity = 500
        self.kp = 2000
        self.ki = 2042.5531914894
        self.kd = 750
        self.autopilot = self.vessel.auto_pilot  # i dont think this is a great idea especially if the connection hasnt been made
        self.controller = pid.PIDController(self.kp, self.ki, self.kd, self.launch_velocity, self.timestep,
                                            self.max_thrust(), label='Vertical \nSpeed')

    def set_pitch(self, value):
        self.pitch = value
        return True

    def set_heading(self, value):
        '''
        value between 0 and 360
        N - 0
        E - 90
        S - 180
        W - 270
        '''
        self.heading = value
        return True

    def reposition(self, desired_pitch, desired_heading, stepped=True):
        if stepped:
            for step in range(int(((self.pitch - desired_pitch) / self.pitch_step))):
                self.set_heading(desired_heading)
                self.set_pitch(self.pitch - self.pitch_step)
                self.autopilot.target_pitch_and_heading(self.pitch, self.heading)
                time.sleep(1 / 3)
        if not stepped:
            self.autopilot.target_pitch_and_heading(self.pitch, self.heading)
        return True

    def determine_thrust_pct(self, thrust_in_newtons):
        return (thrust_in_newtons / self.max_thrust()) * 100.000

    def blast_off(self):
        # check if celestial body has atmosphere
        # class CelestialBody has_atmosphere
        self.autopilot.engage()
        # self.autopilot.sas = True
        self.reposition(90, 0, False)  # Initial position is straight up
        # self.set_throttle(1)  # this can be collected from the parent
        self.vessel.control.activate_next_stage()  # release clamps and start engine
        # initial vertical launch
        # need to account for asparagus staging
        # account for TWR
        while self.srf_altitude() < self.altitude:  # this should be mean_altitude
            thrust = 0
            # if self.wait_until('gt_eq', target=300,current=self.vertical_speed()):
            #     self.controller.set_max_thrust(self.max_thrust())
            #     thrust = self.controller.compute(500, self.vertical_speed())
            #     self.set_throttle(self.determine_thrust_pct(thrust))
            if self.srf_altitude() < 1000:
                self.controller.set_max_thrust(self.max_thrust())
                thrust = self.controller.compute(100.9, self.vertical_speed())
                self.set_throttle(self.determine_thrust_pct(thrust))
            if self.srf_altitude() >= 1000 and self.srf_altitude() < 2000:
                self.controller.set_max_thrust(self.max_thrust())
                thrust = self.controller.compute(110.5, self.vertical_speed())
                self.set_throttle(self.determine_thrust_pct(thrust))
            if self.srf_altitude() >= 2000 and self.srf_altitude() < 3000:
                thrust = self.controller.compute(121.9, self.vertical_speed())
                self.set_throttle(self.determine_thrust_pct(thrust))
            if self.srf_altitude() >= 3000 and self.srf_altitude() < 4000:
                thrust = self.controller.compute(134.5, self.vertical_speed())
                self.set_throttle(self.determine_thrust_pct(thrust))
            if self.srf_altitude() >= 4000 and self.srf_altitude() < 5000:
                thrust = self.controller.compute(148.4, self.vertical_speed())
                self.set_throttle(self.determine_thrust_pct(thrust))
            if self.srf_altitude() >= 5000 and self.srf_altitude() < 6000:
                thrust = self.controller.compute(163.7, self.vertical_speed())
                self.set_throttle(self.determine_thrust_pct(thrust))
            if self.srf_altitude() >= 6000 and self.srf_altitude() < 7000:
                thrust = self.controller.compute(180.6, self.vertical_speed())
                self.set_throttle(self.determine_thrust_pct(thrust))
            if self.srf_altitude() >= 7000 and self.srf_altitude() < 8000:
                thrust = self.controller.compute(199.3, self.vertical_speed())
                self.set_throttle(self.determine_thrust_pct(thrust))
            if self.srf_altitude() >= 8000 and self.srf_altitude() < 9000:
                thrust = self.controller.compute(219.9, self.vertical_speed())
                self.set_throttle(self.determine_thrust_pct(thrust))
            if self.srf_altitude() >= 9000 and self.srf_altitude() < 10000:
                thrust = self.controller.compute(242.6, self.vertical_speed())
                self.set_throttle(self.determine_thrust_pct(thrust))
            if self.srf_altitude() >= 10000 and self.srf_altitude() < 12500:
                thrust = self.controller.compute(267.7, self.vertical_speed())
                self.set_throttle(self.determine_thrust_pct(thrust))
            if self.srf_altitude() >= 12500 and self.srf_altitude() < 15000:
                thrust = self.controller.compute(342.4, self.vertical_speed())
                self.set_throttle(self.determine_thrust_pct(thrust))
            if self.srf_altitude() >= 15000 and self.srf_altitude() < 20000:
                thrust = self.controller.compute(437.8, self.vertical_speed())
                self.set_throttle(self.determine_thrust_pct(thrust))
            if self.srf_altitude() >= 20000 and self.srf_altitude() < 32000:
                self.reposition(75, 90)
                thrust = self.controller.compute(716, self.vertical_speed())
                self.set_throttle(self.determine_thrust_pct(thrust))
            if self.srf_altitude() >= 32000 and self.srf_altitude() < 60000:
                thrust = self.controller.compute(2332, self.vertical_speed())
                self.set_throttle(self.determine_thrust_pct(thrust))
            if self.srf_altitude() < 40000 and self.srf_altitude() >= 30000:
                self.reposition(30, 90)
            if self.srf_altitude() >= 60000 and self.vessel.orbit.apoapsis_altitude < self.altitude:
                self.set_throttle(1)
                # full turn and burn
                self.reposition(0, 90)
            if self.vessel.orbit.apoapsis_altitude >= self.altitude:
                self.set_throttle(0)
                break
            if self.vessel.orbit.apoapsis_altitude >= self.altitude:
                self.set_throttle(0)
                break
            print(
                '''Take off... Altitude: {} m | Apoapsis: {} m | Periapsis {} m \n Heading {} | Pitch {} \n Thrust {} | Speed {}'''.format(
                    round(self.srf_altitude()),
                    round(
                        self.vessel.orbit.apoapsis_altitude),
                    round(
                        self.vessel.orbit.periapsis_altitude), self.heading, self.pitch, round(thrust),
                    round(self.vertical_speed()))
                , end="\r")
        self.controller.graph()
        # while self.vessel.orbit.time_to_apoapsis > 20:
        #     pass
        #     print('''Coasting... Altitude: {} m | Apoapsis: {} m | Periapsis {} m | Situation {}'''.format(
        #         round(self.srf_altitude()),
        #         round(
        #             self.vessel.orbit.apoapsis_altitude),
        #         round(
        #             self.vessel.orbit.periapsis_altitude), self.vessel.situation), end="\r")
        # while self.vessel.orbit.periapsis_altitude < self.altitude:
        #     self.reposition(0, 90)
        #     self.set_throttle(1)
        #     if self.vessel.resources_in_decouple_stage(1, cumulative=False).amount('LiquidFuel') == 0:
        #         self.vessel.control.activate_next_stage()
        #     print('''Circularizing... Altitude: {} m | Apoapsis: {} m | Periapsis {} m | Situation {}'''.format(
        #         round(self.srf_altitude()),
        #         round(self.vessel.orbit.apoapsis_altitude),
        #         round(self.vessel.orbit.periapsis_altitude), self.vessel.situation), end="\r")
        #     if self.vessel.orbit.periapsis_altitude >= self.altitude:
        #         self.set_throttle(0)
        #         print('''Complete... Altitude: {} m | Apoapsis: {} m | Periapsis {} m | Situation {}'''.format(
        #             round(self.srf_altitude()),
        #             round(self.vessel.orbit.apoapsis_altitude),
        #             round(self.vessel.orbit.periapsis_altitude), self.vessel.situation), end="\r")
        #         break
        self.disconnect()

# https://www.reddit.com/r/KerbalAcademy/comments/4b878r/how_to_launch_rockets_efficiently_in_ksp_105_and/

# atmosphere_density
# The current density of the atmosphere around the vessel, in kg/m3.
# dynamic_pressure
# The dynamic pressure acting on the vessel, in Pascals. This is a measure of the strength of the aerodynamic forces. It is equal to 12.air density.velocity2. It is commonly denoted Q.

# solar_panels
# Returns whether all solar panels on the vessel are deployed, and sets the deployment state of all solar panels. See SolarPanel.deployed.
# apoapsis_altitude
# The apoapsis of the orbit, in meters, above the sea level of the body being orbited.
# periapsis_altitude
# The periapsis of the orbit, in meters, above the sea level of the body being orbited.
# time_to_apoapsis
# The time until the object reaches apoapsis, in seconds.
# time_to_periapsis
# The time until the object reaches periapsis, in seconds.
# eccentricity
# The eccentricity of the orbit.
# mean_altitude
# The altitude above sea level, in meters. Measured from the center of mass of the vessel.
# surface_altitude
# The altitude above the surface of the body or sea level, whichever is closer, in meters. Measured from the center of mass of the vessel.
# elevation
# The elevation of the terrain under the vessel, in meters. This is the height of the terrain above sea level, and is negative when the vessel is over the sea.
# class VesselSituation
# The situation a vessel is in. See Vessel.situation.
#
# docked
# Vessel is docked to another.
#
# escaping
# Escaping.
#
# flying
# Vessel is flying through an atmosphere.
#
# landed
# Vessel is landed on the surface of a body.
#
# orbiting
# Vessel is orbiting a body.
#
# pre_launch
# Vessel is awaiting launch.
#
# splashed
# Vessel has splashed down in an ocean.
#
# sub_orbital
# Vessel is on a sub-orbital trajectory.


# cycle through engines and check if has fuel is true
#
# Vessel.current_stage Int
# has_resource(name)
#  in_stage(stage)
#  stream?
# test.vessel.parts.in_stage(test.vessel.control.current_stage - 1)[0].engine.has_fuel


# test.vessel.parts.in_stage(test.vessel.control.current_stage - 1)
# Out[9]:
# [<SpaceCenter.Part remote object #10>,
#  <SpaceCenter.Part remote object #11>,
#  <SpaceCenter.Part remote object #12>,
#  <SpaceCenter.Part remote object #13>]
# test.vessel.parts.in_stage(test.vessel.control.current_stage - 1)[0].engine.has_fuel
# Out[16]: True


# dir(test.vessel.parts.in_stage(test.vessel.control.current_stage - 1)[0].engine)
# Out[15]:
# ['__class__',
#  '__delattr__',
#  '__dict__',
#  '__dir__',
#  '__doc__',
#  '__eq__',
#  '__format__',
#  '__ge__',
#  '__getattribute__',
#  '__gt__',
#  '__hash__',
#  '__init__',
#  '__init_subclass__',
#  '__le__',
#  '__lt__',
#  '__module__',
#  '__ne__',
#  '__new__',
#  '__reduce__',
#  '__reduce_ex__',
#  '__repr__',
#  '__setattr__',
#  '__sizeof__',
#  '__str__',
#  '__subclasshook__',
#  '__weakref__',
#  '_add_method',
#  '_add_property',
#  '_add_static_method',
#  '_class_name',
#  '_client',
#  '_object_id',
#  '_service_name',
#  'active',
#  'auto_mode_switch',
#  'available_thrust',
#  'available_torque',
#  'can_restart',
#  'can_shutdown',
#  'gimbal_limit',
#  'gimbal_locked',
#  'gimbal_range',
#  'gimballed',
#  'has_fuel',
#  'has_modes',
#  'kerbin_sea_level_specific_impulse',
#  'max_thrust',
#  'max_vacuum_thrust',
#  'mode',
#  'modes',
#  'part',
#  'propellant_names',
#  'propellant_ratios',
#  'propellants',
#  'specific_impulse',
#  'throttle',
#  'throttle_locked',
#  'thrust',
#  'thrust_limit',
#  'thrusters',
#  'toggle_mode',
#  'vacuum_specific_impulse']


# dir(test.vessel.parts.in_stage(test.vessel.control.current_stage - 1)[0])
# Out[14]:
# ['__class__',
#  '__delattr__',
#  '__dict__',
#  '__dir__',
#  '__doc__',
#  '__eq__',
#  '__format__',
#  '__ge__',
#  '__getattribute__',
#  '__gt__',
#  '__hash__',
#  '__init__',
#  '__init_subclass__',
#  '__le__',
#  '__lt__',
#  '__module__',
#  '__ne__',
#  '__new__',
#  '__reduce__',
#  '__reduce_ex__',
#  '__repr__',
#  '__setattr__',
#  '__sizeof__',
#  '__str__',
#  '__subclasshook__',
#  '__weakref__',
#  '_add_method',
#  '_add_property',
#  '_add_static_method',
#  '_class_name',
#  '_client',
#  '_object_id',
#  '_service_name',
#  'add_force',
#  'antenna',
#  'axially_attached',
#  'bounding_box',
#  'cargo_bay',
#  'center_of_mass',
#  'center_of_mass_reference_frame',
#  'children',
#  'control_surface',
#  'cost',
#  'crossfeed',
#  'decouple_stage',
#  'decoupler',
#  'direction',
#  'docking_port',
#  'dry_mass',
#  'dynamic_pressure',
#  'engine',
#  'experiment',
#  'fairing',
#  'fuel_lines_from',
#  'fuel_lines_to',
#  'highlight_color',
#  'highlighted',
#  'impact_tolerance',
#  'inertia_tensor',
#  'instantaneous_force',
#  'intake',
#  'is_fuel_line',
#  'launch_clamp',
#  'leg',
#  'light',
#  'mass',
#  'massless',
#  'max_skin_temperature',
#  'max_temperature',
#  'modules',
#  'moment_of_inertia',
#  'name',
#  'parachute',
#  'parent',
#  'position',
#  'radially_attached',
#  'radiator',
#  'rcs',
#  'reaction_wheel',
#  'reference_frame',
#  'resource_converter',
#  'resource_harvester',
#  'resources',
#  'rotation',
#  'sensor',
#  'shielded',
#  'skin_temperature',
#  'solar_panel',
#  'stage',
#  'tag',
#  'temperature',
#  'thermal_conduction_flux',
#  'thermal_convection_flux',
#  'thermal_internal_flux',
#  'thermal_mass',
#  'thermal_radiation_flux',
#  'thermal_resource_mass',
#  'thermal_skin_mass',
#  'thermal_skin_to_internal_flux',
#  'title',
#  'velocity',
#  'vessel',
#  'wheel']


# engine
# An Engine if the part is an engine, otherwise None.
#
# Attribute:	Read-only, cannot be set
# Return type:	Engine
# Game Scenes:	All


# Part.stage and Part.decouple_stage respectively. For parts that are not activated by staging, Part.stage returns -1. For parts that are never decoupled, Part.decouple_stage



def main():
    ship = LaunchProfile(80000)
    ship.blast_off()


if __name__ == "__main__":
    main()