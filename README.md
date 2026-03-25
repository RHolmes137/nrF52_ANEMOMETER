Based on popular 3 cup anemometer available from multiple suppliers such as the Renke Polycarbonate Wind Sensor.

As is, the anemometer draws too much current for battery operation. The optoelectronic rotation sensor and encoding board are replaced with a low current tunnel magnetoristance sensor, XIAO nrf2840 board and SX1262 radio. This gives a mean current consumption of less than 20  microamps, giving years of life on 3 AA batteries and several hundred metres range. Peak power consumption is 45 milliamps for 4.75 millseconds every 2 minutes.

Every 2 minutes, the anemometer broadcasts the mean wind speed and maximum 3 second gust of the previous 10 minutes. The code is written in the Arduino framework, itself based on FreeRTOS - semaphores are used to sleep and wake the MCU, giving rapid response.

![anem1](https://github.com/user-attachments/assets/9ae9a2b2-2650-4923-806c-26bd9a84a536)

The optoelectronic sensor and board are removed.

![anem2](https://github.com/user-attachments/assets/242cbb31-57e4-4b81-8c79-f3b3ec141660)

![anem3](https://github.com/user-attachments/assets/3c123b2d-ef9c-4a58-83c2-64ffa8c85e6a)

The vanes removed and replaced with a pair of small magnets.

![anem4](https://github.com/user-attachments/assets/d5053fc5-cec7-407a-a4d8-20f7161b364a)

The TMR sensor is mounted to a replacement board.

![anem5](https://github.com/user-attachments/assets/0f27e053-b342-4b78-b9dc-45d2b77f5b27)

![anem6](https://github.com/user-attachments/assets/1c1d042b-4d71-4722-afbe-abe45ce6164a)

The anemometer cups balanced with a small blob of epoxy.

![anem7](https://github.com/user-attachments/assets/ded33a6b-a91e-4736-9d27-89d726424b1a)

The nRF52840 board, batteries and SX1262 LoRa transceiver placed in an IP69 enclosure.

![anem8](https://github.com/user-attachments/assets/d0ae4a19-e53d-4798-b66a-9c130ec595a2)









