Based on popular 3 cup anemometer available from multiple suppliers such as the Renke Polycarbonate Wind Sensor.

As is, the anemometer draws too much current for battery operation. The optoelectronic rotation sensor and encoding board are replaced with a low current tunnel magnetoristance sensor, XIAO nrf2840 board and SX1262 radio. This gives a mean current consumption of less than 20  microamps, giving years of life on 3 AA batteries and several hundred metres range. Peak power consumption is 45 milliamps for 4.75 millseconds every 2 minutes.

Every 2 minutes, the anemometer broadcasts the mean wind speed and maximum 3 second gust of the previous 10 minutes. Uses a semaphore to sleep the controller.

![anem1](https://github.com/user-attachments/assets/9ae9a2b2-2650-4923-806c-26bd9a84a536)
