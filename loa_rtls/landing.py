import krpc
import time
import numpy as np
import asyncio
from math import degrees, radians, atan2, sin, cos, tan, asin, atan, sqrt
import matplotlib.pyplot as plt

conn = krpc.connect(name="Loa booster descent guidance")
vessel = conn.space_center.active_vessel
earth = vessel.orbit.body
orbit_frame = earth.non_rotating_reference_frame
smart_ass = conn.mech_jeb.smart_ass
auto_pilot = vessel.auto_pilot

# get lz coordinates 

lz = [v for v in conn.space_center.vessels if v.name == "Landing Zone"][0]
landing_target = np.array(lz.position(earth.reference_frame))

print(earth.latitude_at_position(landing_target,earth.reference_frame),".",earth.longitude_at_position(landing_target,earth.reference_frame))

# exit()

# add streams for telemetry values
speed_orbit = conn.add_stream(getattr, vessel.flight(orbit_frame), 'speed')
speed_surface = conn.add_stream(getattr, vessel.flight(earth.reference_frame), 'speed')
velocity_surface = conn.add_stream(getattr, vessel.flight(earth.reference_frame), 'velocity')
vertical_speed = conn.add_stream(getattr, vessel.flight(earth.reference_frame), 'vertical_speed')
horizontal_speed = conn.add_stream(getattr, vessel.flight(earth.reference_frame), 'horizontal_speed')
surface_altitude = conn.add_stream(getattr, vessel.flight(earth.reference_frame), 'surface_altitude')

# sleep for time, accounting for timewarp
async def sleep(duration):
    await asyncio.sleep(duration/conn.space_center.warp_rate)

booster_engines_outer = vessel.parts.with_tag("booster")
booster_engines_inner = vessel.parts.with_tag("booster_c")


def find_impact():
    target_radius = earth.equatorial_radius
    altitude_at = 1000  # arbitrary positive value
    impact_time = conn.space_center.ut
    increment = 16  # for binary search

    while abs(altitude_at) >= 100:
        altitude_at = vessel.orbit.radius_at(impact_time) - target_radius
        if np.sign(altitude_at) == -np.sign(increment):
            increment *= -0.5
        impact_time += increment
        # print(impact_time, increment)

    uncorrected_impact = vessel.orbit.position_at(impact_time, earth.reference_frame)
    lat = earth.latitude_at_position(uncorrected_impact, earth.reference_frame)
    uncorrected_lon = earth.longitude_at_position(uncorrected_impact, earth.reference_frame)
    lon = uncorrected_lon - degrees((impact_time - conn.space_center.ut) * earth.rotational_speed)

    return earth.msl_position(lat, lon, earth.reference_frame) 

# solve second order ode
# fun(p, p`)
# t is values to solve over, linspace is ideal here
def solve_o2(fun, t_vals, y_con, dy_con):
    y = y_con  # start at starting condition
    dy = dy_con  # derivative

    y_predicted = [y]
    dy_predicted = [dy]

    for t0, t1 in zip(t_vals[:-1],t_vals[1:]):

        d2y = fun(y, dy)  # second derivative
        
        # values for our quadratic approximation
        a = d2y/2
        b = dy - d2y*t0
        c = y - a*t0**2 - b*t0

        y = a*t1**2 + b*t1 + c
        dy = 2*a*t1 + b

        y_predicted.append(y) 
        dy_predicted.append(dy)

    return y_predicted, dy_predicted

async def main():
    try:

        vessel.auto_pilot.disengage()

        part = vessel.parts.with_title('Avionics [Procedural]')[0]
        vessel.parts.controlling = part

        for engine in booster_engines_outer:
            engine.engine.thrust_limit = 1
            engine.engine.active = False


        for engine in booster_engines_inner:
            engine.engine.thrust_limit = 1
            engine.engine.active = True


        if surface_altitude() > 100000:  # entry, disable for debug

            # point retrograde
            retrograde = conn.space_center.transform_direction((0, -1, 0), vessel.orbital_reference_frame, vessel.surface_reference_frame)
            heading = degrees(atan2(retrograde[2], retrograde[1])) + 360
       
            smart_ass.autopilot_mode = smart_ass.autopilot_mode.surface
            # smart_ass.force_roll = True
            smart_ass.surface_heading = heading
            smart_ass.surface_pitch = 10        
            smart_ass.update(False)
            vessel.control.throttle = 0.5

            print("TURNAROUND FOR BOOSTBACK")

            while vessel.flight().heading < 180 or vessel.flight().pitch > 30:
                await sleep(0.5)

            print("THROTTLE UP BOOSTBACK")

            vessel.control.throttle = 1

            behind = False  # impact is behind target

            while True:
                impact = np.array(find_impact())
                dist = np.linalg.norm(landing_target-impact)
                # print(dist)

                # calculate perpendicular deflection
                # plane passing through vessel, target, and center of earth
                P = np.cross(vessel.position(earth.reference_frame), landing_target)
                # distance from plane
                deflection = sum(impact*P) / np.linalg.norm(P)
                print(dist, sqrt(dist**2 - deflection**2))

                if sqrt(dist**2 - deflection**2) > 28000:
                    break

                if dist < 60000:
                    vessel.control.throttle = 0.3
                    if deflection < -100:
                        smart_ass.surface_heading = heading - 6
                    elif deflection > 100:
                        smart_ass.surface_heading = heading + 6
                    else:
                        smart_ass.surface_heading = heading
                    smart_ass.update(False)

                elif dist < 400000:
                    if deflection < -1000:
                        smart_ass.surface_heading = heading - 6
                    elif deflection > 1000:
                        smart_ass.surface_heading = heading + 6
                    else:
                        smart_ass.surface_heading = heading
                    smart_ass.update(False)

                await sleep(0.5)
            

            # print(dist, deflection)

        vessel.control.throttle = 0
        print("BOOSTBACK BURN COMPLETE")
        print("ORIENTING FOR RE-ENTRY")
        
        vessel.control.rcs = True

        smart_ass.surface_vel_yaw = 0
        smart_ass.force_roll = True
        smart_ass.surface_vel_roll = 0
        smart_ass.surface_vel_pitch = -50
        smart_ass.autopilot_mode = smart_ass.autopilot_mode.surface_retrograde
        smart_ass.update(False)

        # make sure we're in the atmosphere
        while speed_surface() > 1700 or surface_altitude() > 70000:
            await sleep(0.5)        

        vessel.control.rcs = False

        while speed_surface() > 400:
            await sleep(0.5)

        smart_ass.autopilot_mode = smart_ass.autopilot_mode.kill_rot
        smart_ass.update(False)

        vessel.control.pitch = -1

        profile_defl = [-30,-15,-5, 0, 5, 15, 30]
        profile_roll = [-15,-10,-2, 0, 2, 10, 15]

        profile_alt = [-1000,-100, 100, 200]
        profile_slope = [0.5, 0, 0, -0.1]

        target_distance = 100000  # arbitrarily large value

        prev_slope_e = 0
        sum_slope_e = 0

        # while True:
        while surface_altitude() > 1000 and target_distance > 1200:
            #    calculate compenent of the velocity normal to target (deflection)
            # this projects the vessels position onto the earth's surface
            # i know there's a better way to do it
            # i do not care
            surface_pos = earth.surface_position(
                earth.latitude_at_position(vessel.position(earth.reference_frame), earth.reference_frame),
                earth.longitude_at_position(vessel.position(earth.reference_frame), earth.reference_frame),
                earth.reference_frame
                )
            surface_pos = np.array(surface_pos)
            # plane passing through vessel, target, and center of earth
            P = np.cross(surface_pos, landing_target)
            # projection of velocity normal to plane
            deflection = np.dot(velocity_surface(), P) / np.linalg.norm(P)
            # total surface distance from target
            target_distance = np.linalg.norm(surface_pos - landing_target)

            target_roll = np.interp(deflection, profile_defl, profile_roll)

            roll_error = target_roll - vessel.flight().roll

            if roll_error > 1:
                vessel.control.roll = +0.1
            
            if roll_error < -1:
                vessel.control.roll = -0.1 
            
            # calculating the target pitch to get on nominal trajectory
            altitude = surface_altitude()
            nominal_slope = 2/3
            nominal_altitude = nominal_slope * target_distance + 2000

            target_slope = nominal_slope+np.interp(nominal_altitude - altitude, profile_alt, profile_slope)

            slope_error = target_slope - abs(vertical_speed() / horizontal_speed())
            d_slope_e = slope_error - prev_slope_e
            sum_slope_e += slope_error

            prev_slope_e = slope_error


            print(target_slope, slope_error, sum_slope_e, d_slope_e)
 
            kp = 2
            ki = 0
            kd = -10

            vessel.control.pitch = min(kp*slope_error + ki*sum_slope_e + kd*d_slope_e, -0.1)



            # if slope_error > 0 and vessel.control.pitch < -0.1:
            #     vessel.control.pitch += 0.005
            # if slope_error < 0:
            #     vessel.control.pitch -= 0.01

            # vessel.control.pitch = np.interp(target_slope, profile_slope, profile_pitch)

            # print(deflection, target_distance, target_angle)

            await sleep(0.1)

            vessel.control.roll = 0
            # vessel.control.pitch = 0

            # horizontal acceleration
            # accel = (vessel.available_thrust/vessel.mass)*(horizontal_speed() / np.linalg.norm((horizontal_speed() * vertical_speed())))
            # stopping_distance = horizontal_speed()**2 / 2*accel
            # print(target_distance, stopping_distance)
            # if stopping_distance > target_distance:
            #     break

            # y = np.array(vessel.position(earth.reference_frame))  # start at starting condition
            # dy = np.array(vessel.velocity(earth.reference_frame))  # derivative

            # y_predicted = [y]
            # dy_predicted = [dy]

            # timestep = 0.1
            # t0 = 0
            # t1 = timestep

            # while np.linalg.norm(dy[1:]) > 20 and t0 < 30:

            #     d2y = -(vessel.available_thrust/vessel.mass) * (dy/np.linalg.norm(dy)) - np.array([-9.8,0,0]) # second derivative
                
            #     # values for our quadratic approximation
            #     a = d2y/2
            #     b = dy - d2y*t0
            #     c = y - a*t0**2 - b*t0

            #     y = a*t1**2 + b*t1 + c
            #     dy = 2*a*t1 + b

            #     y_predicted.append(y) 
            #     dy_predicted.append(dy)

            #     t0 += timestep
            #     t1 += timestep

            # print(y, dy)

        print("INITIATE LANDING BURN")

        vessel.control.pitch = 0
        vessel.control.rcs = True
        # auto_pilot.disengage()
        # smart_ass.autopilot_mode = smart_ass.autopilot_mode.surface_retrograde
        # smart_ass.force_pitch = True
        # smart_ass.surface_vel_pitch = 0
        # smart_ass.surface_vel_yaw = 0
        # # smart_ass.surface_vel_roll = 0
        # smart_ass.update(False)

        vessel.control.throttle = 1

        # while speed_surface() > 60:
        #     await sleep(0.5)
        
        # for engine in booster_engines_inner[1:]:
        #     engine.engine.active = False
        # landing_engine = booster_engines_inner[0].engine

        # # smart_ass.surface_vel_pitch = 5

        # while speed_surface() > 30:
        #     await sleep(0.5)


        # smart_ass.autopilot_mode = smart_ass.autopilot_mode.vertical_plus            smart_ass.autopilot_mode = smart_ass.autopilot_mode.surface

        # while speed_surface() > 60:

        # auto_pilot.engage()

        smart_ass.autopilot_mode = smart_ass.autopilot_mode.surface_retrograde
        smart_ass.force_roll = False
        smart_ass.surface_vel_pitch = 0
        smart_ass.surface_vel_yaw = 0        
        smart_ass.update(False)

        while speed_surface() > 50:
            await sleep(0.5)
        
        vessel.control.throttle = 0.01
        smart_ass.autopilot_mode = smart_ass.autopilot_mode.vertical_plus

        while speed_surface() > 15:
            await sleep(0.5)

        smart_ass.autopilot_mode = smart_ass.autopilot_mode.surface
        controlling_frame = vessel.parts.controlling.reference_frame

        for engine in booster_engines_inner[1:]:
            engine.engine.active = False
        landing_engine = booster_engines_inner[0].engine

        vessel.control.gear = True

        while True:
            relative_position = vessel.position(lz.surface_reference_frame)
            h_displacement = np.linalg.norm(relative_position[1:])

            if h_displacement < 15 and relative_position[0] < 25:
                transl_vel = 0
            else:
                transl_vel = -h_displacement/8

            if relative_position[0] > 300:
                target_speed = -30
            elif h_displacement > relative_position[0] / 2 and h_displacement > 15:
                target_speed = 0
            elif relative_position[0] > 22:
                target_speed = -10
            else:
                target_speed = -5


            vspeed = vertical_speed()
            hspeed = horizontal_speed()
            # print(vspeed)
            weight = vessel.mass * earth.surface_gravity
            throttle_abs = weight / vessel.available_thrust

            retrograde = np.array(conn.space_center.transform_direction((0, -speed_surface(), 0), vessel.surface_velocity_reference_frame, vessel.surface_reference_frame))
            
            h_correction = transl_vel * np.array(relative_position[1:]) / h_displacement
            retrograde[1:] += h_correction
                

            if np.linalg.norm(retrograde[1:]) > 3:
                pitch_angle = radians(87)
            else:
                pitch_angle = radians(90-np.linalg.norm(retrograde[1:]))
            # pitch_angle = radians(90)

            point_direction = np.array([sin(pitch_angle), *(cos(pitch_angle) * retrograde[1:] / np.linalg.norm(retrograde[1:]))])

            r_point_direction = np.array(conn.space_center.transform_direction(point_direction, vessel.surface_reference_frame, controlling_frame))
            
            z_angle = tan(r_point_direction[2] / np.linalg.norm(r_point_direction[:2]))
            z_angle += radians(5)
            r_point_direction[:2] = cos(z_angle) * (r_point_direction[:2] / np.linalg.norm(r_point_direction[:2]))
            r_point_direction[2] = sin(z_angle)
            
            point_direction = np.array(conn.space_center.transform_direction(r_point_direction, controlling_frame, vessel.surface_reference_frame))

            heading = degrees(atan2(point_direction[2], point_direction[1])) % 360
            pitch = degrees(asin(point_direction[0]))
            roll = (heading - 270) % 360

            smart_ass.surface_heading = heading
            smart_ass.surface_pitch = pitch
            smart_ass.surface_roll = roll
            smart_ass.update(False)

            if relative_position[0] < 17:
                break

            if vspeed < target_speed:
                throttle_abs += 0.15
            if vspeed > target_speed + 5:
                throttle_abs -= 0.05

            vessel.control.throttle = throttle_abs - .19


            print(h_displacement)

        vessel.control.throttle = 0
        smart_ass.surface_pitch = 90
        smart_ass.force_roll = False
        smart_ass.update(False)
        # smart_ass.autopilot_mode = smart_ass.autopilot_mode.off
        print("WELCOME HOME")

    except Exception as e:
        print("ERROR",str(e))



asyncio.run(main())
