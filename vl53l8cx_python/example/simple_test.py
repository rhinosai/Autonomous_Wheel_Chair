import time

from vl53l8cx.vl53l8cx import VL53L8CX

driver = VL53L8CX(bus_id=0)

alive = driver.is_alive()
if not alive:
    raise IOError("VL53L8CX Device is not alive")

print("Initialising...")
t = time.time()
driver.init()
print(f"Initialised ({time.time() - t:.1f}s)")

# python test code




# Ranging:

res = 8
frequency = 15
mode = 1

driver.set_resolution(res*res)
# frequency needs to be set after setting the resolution
frequency = min(frequency, 15) if res == 8 else min(frequency, 60)
driver.set_ranging_frequency_hz(frequency)
driver.set_ranging_mode(mode)

driver.start_ranging()

previous_time = 0
loop = 0
start_time = time.time()
delay_ms = 100
print(f"Delay : {delay_ms:.1f}ms")
while True:
    if driver.check_data_ready():
        print(f"Loop : {loop: >3d} ({time.time() - start_time:.1f}s)")
        ranging_data = driver.get_ranging_data()

        # As the sensor is set in 4x4 mode by default, we have a total 
        # of 16 zones to print. For this example, only the data of first zone are 
        # print
        now = time.time()
        if previous_time != 0:
            time_to_get_new_data = now - previous_time
            print(f"Print data no : {driver.streamcount: >3d} ({time_to_get_new_data * 1000:.1f}ms)")
        else:
            print(f"Print data no : {driver.streamcount: >3d}")

        for i in range(res*res):
            print(f"Zone : {i: >3d}, "
                  f"Status : {ranging_data.target_status[driver.nb_target_per_zone * i]: >3d}, "
                  f"Distance : {ranging_data.distance_mm[driver.nb_target_per_zone * i]: >4.0f} mm")

        print("")

        previous_time = now
        loop += 1
        time.sleep(delay_ms/1000)
    else :
        print(f"data is not ready : {loop: >3d} ({time.time() - start_time:.1f}s)")
        time.sleep(1000)
    
#    time.sleep(delay_ms/1000)

driver.stop_ranging()
