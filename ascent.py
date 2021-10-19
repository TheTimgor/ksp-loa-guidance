# MAX PAYLOAD: 30T

import krpc
import time
import numpy as np
import asyncio
from threading import Thread # i need both of these for some reason
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
from math import degrees, atan2

conn = krpc.connect(name="Loa ascent guidance")
vessel = conn.space_center.active_vessel
orbit_frame = vessel.orbit.body.non_rotating_reference_frame

# add streams for telemetry values
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
speed_orbit = conn.add_stream(getattr, vessel.flight(orbit_frame), 'speed')
vertical_speed = conn.add_stream(getattr, vessel.flight(orbit_frame), 'vertical_speed')
periapsis = conn.add_stream(getattr, vessel.orbit, 'periapsis_altitude')
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
pitch = conn.add_stream(getattr, vessel.control, 'pitch')

# sleep for time, accounting for timewarp
async def sleep(duration):
    await asyncio.sleep(duration/conn.space_center.warp_rate)

booster_engines_outer = vessel.parts.with_tag("booster")
booster_engines_inner = vessel.parts.with_tag("booster_c")


async def pitch_program():
    print("INITIATE PITCH PROGRAM")
    profile_alt = [200,     300,    5000,   10000,  15000,  20000,  30000,  140000]
    profile_pit = [90,      90,     80,     60,     55,     45,     35,     25]

    alt = 0

    # smooth gravity turn
    while alt < profile_alt[-1]:
        alt = altitude()
        pit = np.interp(alt, profile_alt, profile_pit)
        vessel.auto_pilot.target_pitch = pit
        await sleep(0.1)

async def throttle_program():
    print("INITIATE THROTTLE PROGRAM")

    # keep vectoring within safe range for maneuvering

    throttle = 1
    while throttle > 0:
        if pitch() < -0.80:
            throttle -= 0.01
            for engine in booster_engines_outer:
                engine.engine.thrust_limit = throttle
        await sleep(0.1)

    print("SECONDARY BOOSTER ENGINE CUTOFF")
    for engine in booster_engines_outer:
        engine.engine.active = False

    throttle = 1
    while throttle > 0:
        if pitch() < -0.80:
            throttle -= 0.01
            for engine in booster_engines_inner:
                engine.engine.thrust_limit = throttle
        await sleep(0.1)

async def main():

    vessel.auto_pilot.target_pitch_and_heading(90, 90)
    vessel.auto_pilot.target_roll = 0
    vessel.auto_pilot.engage()

    await sleep(2)

    if True:  # can be disabled for debug

        print("MAIN ENGINE START")
        vessel.control.throttle = 1
        vessel.control.activate_next_stage()

        # wait for spool up
        await sleep(3)

        print("LIFTOFF")
        vessel.control.activate_next_stage()

        await sleep(5)

        pitch_task = asyncio.create_task(pitch_program())
        throttle_task = asyncio.create_task(throttle_program())

        while speed_orbit() < 3200:  # stage separation speed
            await sleep(0.5)

        pitch_task.cancel()
        throttle_task.cancel()

        await sleep(0.5)
        print("STAGE SEPARATION + BECO")
        vessel.control.activate_next_stage()
        vessel.control.rcs = True
        for engine in booster_engines_inner:
            engine.engine.active = False

        await sleep(2)
        vessel.auto_pilot.target_pitch -= 1
        await sleep(8)

        print("GO FOR INSERTION")
    
    vessel.auto_pilot.target_pitch = 20
        
    v_speed_previous = 10000  # arbitrarily large value
    time_delta = 0.1  # seconds
    pitch_rate = 1  # degrees/second

    while periapsis() < 0:
        # manage vertical speed
        v_speed = vertical_speed()
        v_accel = (v_speed - v_speed_previous)/time_delta
        # print(v_accel)
        if (
            v_accel > -8
            and v_speed > 0 
            and vessel.auto_pilot.target_pitch > 0
           ):
            vessel.auto_pilot.target_pitch -= pitch_rate*time_delta
        v_speed_previous = v_speed

        # keep heading prograde to not waste burning normal
        prograde = conn.space_center.transform_direction((0, 1, 0), vessel.orbital_reference_frame, vessel.surface_reference_frame)
        heading = degrees(atan2(prograde[2], prograde[1]))
        vessel.auto_pilot.target_heading = heading

        await sleep(time_delta)

    vessel.control.throttle = 0.1

    while periapsis() < 200000:
        pass

    vessel.control.throttle = 0

    print("ORBIT CONFIRMED")

# visualizer_process = Thread(target=visualizer)
# visualizer_process.start()

asyncio.run(main())
