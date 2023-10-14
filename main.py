import krpc
from time import sleep
from math import sqrt

from Vector import Vector3

from PID import PIDController

class LandingBurn:
    def __init__(self):
        self.conn = krpc.connect("Starship")
        self.drawing = self.conn.drawing
        self.space_center = self.conn.space_center
        self.vessel = self.space_center.active_vessel
        self.control = self.vessel.control
        self.auto_pilot = self.vessel.auto_pilot
        self.body = self.vessel.orbit.body
        
        self.body_ref = self.body.reference_frame
        self.vessel_ref = self.vessel.reference_frame
        self.surface_ref = self.vessel.surface_reference_frame
        self.hybrid_ref = self.space_center.ReferenceFrame.create_hybrid(position=self.body_ref, rotation=self.surface_ref)

        self.flight_body = self.vessel.flight(self.body_ref)
        self.flight_surface = self.vessel.flight(self.surface_ref)
        self.flight_hybrid = self.vessel.flight(self.hybrid_ref)

        # GET WINGS
        self.wing_ur, self.wing_ul, self.wing_dr, self.wing_dl = self.get_wings()

        # PID
        self.wings_controller = PIDController()
        self.wings_controller.adjust_pid(0.1, 0.01, 0.1)
        self.wings_controller.limit_output(0, 1)

        # Streams
        self.stream_mass = self.conn.add_stream(getattr, self.vessel, "mass")
        self.stream_av_thrust = self.conn.add_stream(getattr, self.vessel, "available_thrust")
        self.stream_vel = self.conn.add_stream(getattr, self.flight_hybrid, "velocity")
        self.stream_surface_altitude = self.conn.add_stream(getattr, self.flight_body, "surface_altitude")
        self.stream_pitch = self.conn.add_stream(getattr, self.flight_surface, "pitch")
        self.stream_situation = self.conn.add_stream(getattr, self.vessel, "situation")

        # ===================

        self.land_func = lambda self: (
            self.progressive_engine_cut(),
            self.sas_aim_up(),
            self.finish())
        
        # Params
        self.max_twr = 4
        self.eng_threshold = .8
        self.final_speed = -1
        self.final_altitude = 10
        self.gear_delay = 4
        self.flip_delay = .5

        # Consts
        self.a_g = self.body.surface_gravity
        self.landed_situation = self.vessel.situation.landed
        self.splashed_situation = self.vessel.situation.splashed
        self.vf_2 = self.final_speed*abs(self.final_speed)

        # FTS - Get parts
        fts_modules = []
        for part in [p for p in self.vessel.parts.all if (p.engine is None and p.fairing is None)]:
            for m in part.modules:
                if m.name.lower().startswith("tac"):
                    fts_modules.append(m)
                    break
   
        print(f"FTS Count: {len(fts_modules)}")

        # Initializing
        self.control.throttle = 0
        self.control.brakes = False
        self.control.rcs = True

        # Reset Wings
        self.wing_ul.target_angle = 180
        self.wing_ur.target_angle = 180
        self.wing_dl.target_angle = 180
        self.wing_dr.target_angle = 180

        # Auto Pilot
        self.auto_pilot.reference_frame = self.surface_ref
        self.auto_pilot.stopping_time = (.5, .5, .5)
        self.auto_pilot.deceleration_time = (5, 5, 5)
        self.auto_pilot.engage()
        
        # Ascent phase
        if self.stream_situation() == self.vessel.situation.landed or self.stream_situation() == self.vessel.situation.pre_launch:
            self.auto_pilot.target_direction = (1, 0, 0)
            self.auto_pilot.target_roll = 0
            
            target_altitude = 5000
            v_max = 50

            self.control.throttle = 1
            sleep(1)
            while self.stream_vel()[0] < 2:
                self.control.activate_next_stage()
                sleep(2)
                
            while True:
                mass = self.stream_mass()
                av_thrust = self.stream_av_thrust()
                alt = self.stream_surface_altitude()
                v_x = self.stream_vel()[0]

                dy = target_altitude - alt
                if dy<0: break

                a_eng = av_thrust / mass
                a_net = a_eng - self.a_g
                v_target = a_net * sqrt(dy/a_net)
                v_target = min(v_max, v_target)
                delta_vel = v_target - v_x
                self.control.throttle = (delta_vel*2 + self.a_g) / a_eng
                sleep(0.1)

        # DESCENT PHASE
        self.accelerating = False
        self.control.throttle = 0
        self.auto_pilot.target_pitch = 0
        self.auto_pilot.target_heading = 0
        self.auto_pilot.target_roll = 0

        while True:
            # Get Streams
            vel = Vector3(self.stream_vel())
            alt = self.get_altitude()
            mass = self.stream_mass()
            av_thrust = self.stream_av_thrust()

            mag_speed = vel.magnitude()
            
            a_eng = av_thrust / mass
            a_eng_l = a_eng * self.eng_threshold * min(self.max_twr * self.a_g / a_eng, 1)

            a_net = max(a_eng_l - self.a_g, .1) # used to calculate v_target

            burn_altitude = abs((mag_speed**2 + self.vf_2 + 2*self.a_g*alt) / (2 * a_eng_l))

            v_2 = vel.x*abs(vel.x) # auxiliar math variable
            t_free_fall = (vel.x + sqrt(max(0, 2*self.a_g*alt - v_2))) / self.a_g
            t_to_burn = (vel.x + sqrt(max(0, 2*self.a_g*(alt - burn_altitude) - v_2))) / self.a_g
            t_burning = sqrt((2*burn_altitude) / a_net)
            t_fall = t_to_burn + t_burning

            '''
            burn_altitude = (mag_speed**2 - self.final_speed**2 + 2*self.a_g*alt) / (2 * a_eng_l)
            t_free_fall = (vel.x + sqrt(vel.x**2 + 2*self.a_g*alt)) / self.a_g
            t_to_burn = (vel.x + sqrt(max(0, vel.x**2 + 2*self.a_g*(alt - burn_altitude)))) / self.a_g
            t_burning = sqrt(2*abs(burn_altitude) / a_net)
            t_fall = t_to_burn + t_burning
            '''

            if not self.accelerating:
                if t_to_burn <= self.flip_delay: # EXEC BELLY FLOP
                    self.wing_ul.target_angle = 180
                    self.wing_ur.target_angle = 180
                    self.wing_dl.target_angle = 90
                    self.wing_dr.target_angle = 90
                    self.accelerating = True
                    self.auto_pilot.reference_frame = self.body_ref
                    self.auto_pilot.target_roll = -90
                else:
                    pitch = self.stream_pitch()
                    ctrl = self.wings_controller.calc_pid(pitch, 0)

                    up = 90 * (ctrl + 1)
                    down = 270 - up
                   
                    self.wing_ul.target_angle = up
                    self.wing_ur.target_angle = up
                    self.wing_dl.target_angle = down
                    self.wing_dr.target_angle = down

                    #print(f'Pitch: {pitch:.2f} | PID: {ctrl:.2f} | TToBurn: {t_to_burn:.2f}')

            else: # THROTTLING
                target_dir = -vel
                target_dir.x = abs(target_dir.x)

                # Throttle Controller
                dist = Vector3(-alt, vel.y*t_free_fall, vel.z*t_free_fall).magnitude()

                if alt > self.final_altitude:
                    target_speed = sqrt(max(0, 2*a_net*(dist-self.final_altitude) - self.vf_2))
                    delta_speed = mag_speed - target_speed
                    throttle = (delta_speed*5 + self.a_g) / a_eng
                else: # Final Burn
                    target_dir.x *= 15
                    delta_speed = self.final_speed - vel.x
                    throttle = (delta_speed*2 + self.a_g) / a_eng

                    # Check Landed
                    situation = self.stream_situation()
                    if situation == self.landed_situation or situation == self.splashed_situation:
                        self.land_func(self)
                        break

                self.control.throttle = throttle
    
                # fts
                if delta_speed > 20 and len(fts_modules) != 0:
                    self.vessel.control.throttle = 0
                    for m in fts_modules: m.trigger_event("Self Destruct!")
                    print("FTS - ACTIVATED!")
                    break

                # Check Gears
                if t_fall < self.gear_delay: self.control.gear = True
        
                # Aim Vessel
                self.auto_pilot.target_direction = self.space_center.transform_direction(target_dir, self.surface_ref, self.body_ref)

            sleep(0.05)

    def progressive_engine_cut(self):
        a_eng = self.stream_av_thrust() / self.stream_mass()

        throttle = min((self.a_g / a_eng) *.8, 1)
        total_time = 1
        dt = 0.1
        
        delta_throttle = (throttle * dt) / total_time

        while throttle > 0:
            throttle -= delta_throttle
            self.control.throttle = throttle
            sleep(dt)

        self.control.throttle = 0

    def sas_aim_up(self):
        self.auto_pilot.disengage()
        self.control.rcs = True
        self.control.sas = True
        sleep(0.1)
        try:
            self.control.sas_mode = self.control.sas_mode.radial
        except:
            self.control.sas = False
            self.auto_pilot.engage()
            self.auto_pilot.target_direction = self.space_center.transform_direction((1, 0, 0), self.surface_ref, self.body_ref)
            input("Press enter to finish program.")
            self.auto_pilot.disengage()

    def finish(self):
        self.control.rcs = True
        self.control.brakes = False
        print(f"{self.vessel.name} has landed!")
        self.conn.close()

    def get_altitude(self):
        return max(0, self.stream_surface_altitude() + self.vessel.bounding_box(self.surface_ref)[0][0])

    def get_wings(self):
        hinges = self.vessel.parts.robotic_hinges
        hinges.sort(key=lambda v: v.part.position(self.vessel_ref)[1], reverse=True)

        wings_up = hinges[:2]
        wings_up.sort(key=lambda v: v.part.position(self.vessel_ref)[0], reverse=True)

        wings_down = hinges[2:]
        wings_down.sort(key=lambda v: v.part.position(self.vessel_ref)[0], reverse=True)

        return wings_up + wings_down

if __name__ == "__main__":
    LandingBurn()