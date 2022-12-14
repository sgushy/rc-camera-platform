# rc-camera-platform

<p>This is a wirelessly controlled camera platform suitable for a smartphone, involving mechanical, electrical, and firmware elements, designed and built by a group of 2 for the Mechatronics Design class at UC Berkeley. The firmware was written in C++ using the Arduino platform, and the device was made using 2 ESP-32 microcontrollers, some RC car transmission gears, skateboard bearings, metal rods, a handful of $1 MPU-6050 and MPU-9250 IMU boards, and some 3d printed plastic parts.</p>

<video src="https://user-images.githubusercontent.com/113747791/191147041-62e3f0af-c7d9-48a0-b825-159e07531564.mp4"></video>

<p>Below are a few images of the (from left to right) assembled camera platform, tilt sensing remote control (with camera platform internals visible in background), and remote control internals.</p> 

<img src="https://user-images.githubusercontent.com/113747791/191139676-475fb703-f1bf-4415-8959-d996843d879b.jpg" width="200"> <img src="https://user-images.githubusercontent.com/113747791/191139634-77254d5a-5a11-4a9b-b076-ee95fbe13af0.jpg" width="200"> <img src="https://user-images.githubusercontent.com/113747791/191140378-631eb1d3-ed77-4696-b6f8-71539e5e10d9.jpg" width="200">

<p>The device features fully wireless operation, with the remote control sending actuation signals to the platform. The camera platform will rotate and elevate/depress according to a comparison of its own state and that of the remote control. Communication is performed using the WLAN antennas on the ESP32, using HTTP to send messages. Multithreading is used to prevent the WLAN operation from blocking the internal state machine.</p>

<p>The state estimation and feedback control of the system relied entirely on a pair of inertial measurement units (IMUs), consisting of an MPU-9250 on the platform and an MPU-6050 on the remote controller. This was perhaps a bit of a risky choice, as these inexpensive IMU chips do suffer a large amount of background noise in their measurements. However, our signal filter and feedback controller implementation was robust enough to effectively control the system.</p>

<p>The following show the wiring of the device:</p>
<img src="https://user-images.githubusercontent.com/113747791/191141739-d6079753-5cf6-444b-a487-4d5f07f1bf83.png" width = 825> <img src="https://user-images.githubusercontent.com/113747791/191142824-2658a921-ddc9-46d6-b5b2-4f4b4b2608fb.png" width = 825>

<p>Firmware was implemented using the following state machines:</p>
<img src="https://user-images.githubusercontent.com/113747791/191142084-667ed929-38ce-4a0d-be6b-3150c12fb496.png" width=650>

<p>We had initially also planned to incorporate motion tracking by the camera platform via GPS, but the GPS antenna was damaged and the IMUs were too noisy to effectively function on their own in an integrator (and especially a double-integrator!), so this was not implemented. Relics of these unimplemented features can be found, however, within the code!</p>
