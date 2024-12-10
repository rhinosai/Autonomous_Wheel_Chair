import time
import logging

from vl53l8cx.vl53l8cx import VL53L8CX

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

driver = VL53L8CX(bus_id=0)

def init_driver():
    alive = driver.is_alive()
    if not alive:
        raise IOError("VL53L8CX Device is not alive")

    logging.debug("Initialising...")
    t = time.time()
    driver.init()
    logging.debug(f"Initialised ({time.time() - t:.1f}s)")

    # Ranging:

    res = 8
    frequency = 15
    mode = 3

    driver.set_resolution(res*res)
    # frequency needs to be set after setting the resolution
    frequency = min(frequency, 15) if res == 8 else min(frequency, 60)
    driver.set_ranging_frequency_hz(frequency)
    driver.set_ranging_mode(mode)
    driver.start_ranging()

def main():
    init_driver()
    previous_time = 0
    loop = 0
    start_time = time.time()
    delay_ms = 100
    error_count = 0
    logging.debug(f"Delay : {delay_ms:.1f}ms")
    resolution = driver.get_resolution()
    position = resolution - 1
    while True:
        if driver.check_data_ready():
            # logging.debug(f"Loop : {loop: >3d} ({time.time() - start_time:.1f}s)")
            ranging_data = driver.get_ranging_data()

            # As the sensor is set in 4x4 mode by default, we have a total 
            # of 16 zones to print. For this example, only the data of first zone are 
            # print
            # now = time.time()
            # if previous_time != 0:
            #     time_to_get_new_data = now - previous_time
            #     logging.debug(f"Print data no : {driver.streamcount: >3d} ({time_to_get_new_data * 1000:.1f}ms)")
            # else:
            #     logging.debug(f"Print data no : {driver.streamcount: >3d}")

            # for i in range(resolution):
            #     print(f"Zone : {i: >3d}, "
            #         f"Status : {ranging_data.target_status[driver.nb_target_per_zone * i]: >3d}, "
            #         f"Distance : {ranging_data.distance_mm[driver.nb_target_per_zone * i]: >4.0f} mm")

            # print("")

            logging.debug(f"Loop :  {loop: >3d} ({time.time() - start_time:.1f}s), "
                f"Zone : {position: >3d}, "
                f"Status : {ranging_data.target_status[driver.nb_target_per_zone * position]: >3d}, "
                f"Distance : {ranging_data.distance_mm[driver.nb_target_per_zone * position]: >4.0f} mm, "
                f"No : {driver.streamcount: >3d}"
                )


            # previous_time = now
            loop += 1
            error_count = 0
        else:
            error_count += 1
            logging.debug(f"data is not ready(error_count={error_count}) : {loop: >3d} ({time.time() - start_time:.1f}s)")
            
        time.sleep(delay_ms/1000)

        if (error_count > 10):
            logging.debug(f"reset driver : {loop: >3d} ({time.time() - start_time:.1f}s)")
            init_driver()
            error_count = 0


    driver.stop_ranging()

if __name__ == "__main__":
    main()