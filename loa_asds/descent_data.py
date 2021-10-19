import krpc
import time
import numpy as np
import asyncio
from math import degrees, radians, atan2, sin, cos, tan, asin

conn = krpc.connect(name="Loa booster descent guidance")
vessel = conn.space_center.active_vessel
earth = vessel.orbit.body
orbit_frame = earth.non_rotating_reference_frame
smart_ass = conn.mech_jeb.smart_ass
auto_pilot = vessel.auto_pilot

# get asds coordinates 

asds = [v for v in conn.space_center.vessels if v.name == "ASDS"][0]
landing_target = np.array(asds.position(earth.reference_frame))

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

async def main():
    while True:
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


            if False:  # entry, disable for debug

                # point retrograde
                retrograde = conn.space_center.transform_direction((0, -1, 0), vessel.orbital_reference_frame, vessel.surface_reference_frame)
                heading = degrees(atan2(retrograde[2], retrograde[1])) + 360
           
                smart_ass.autopilot_mode = smart_ass.autopilot_mode.surface
                # smart_ass.force_roll = True
                smart_ass.surface_heading = heading
                smart_ass.surface_pitch = 10        
                smart_ass.update(True)
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
                    print(dist)

                    # calculate perpendicular deflection
                    # plane passing through vessel, target, and center of earth
                    P = np.cross(vessel.position(earth.reference_frame), landing_target)
                    # distance from plane
                    deflection = sum(impact*P) / np.linalg.norm(P)


                    if dist > 29000 and behind:
                        break
                    if dist < 10000:
                        behind = True

                    if dist < 60000:
                        vessel.control.throttle = 0.3
                        if deflection < -100:
                            smart_ass.surface_heading = heading - 1
                        elif deflection > 100:
                            smart_ass.surface_heading = heading + 1
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


            while vertical_speed() < -50:
                await sleep(0.5)

            surface_pos = earth.surface_position(
                earth.latitude_at_position(vessel.position(earth.reference_frame), earth.reference_frame),
                earth.longitude_at_position(vessel.position(earth.reference_frame), earth.reference_frame),
                earth.reference_frame
                )
            surface_pos = np.array(surface_pos)
            target_distance = np.linalg.norm(surface_pos - landing_target)

            print(target_distance, surface_altitude())

            conn.space_center.quickload()

        except Exception as e:
            print("ERROR",str(e))

asyncio.run(main())
