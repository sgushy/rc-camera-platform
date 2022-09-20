# rc-camera-platform

<p>This is a wirelessly controlled camera platform suitable for a smartphone, involving mechanical, electrical, and firmware elements, designed and built by a group of 2 for the Mechatronics Design class at UC Berkeley. The firmware was written in C++ using the Arduino platform, and the device was made using 2 ESP-32 microcontrollers, some RC car transmission gears, skateboard bearings, metal rods, a handful of $1 MPU-6050 and MPU-9250 IMU boards, and some 3d printed plastic parts.</p>

<p>Below are a few images of the (from left to right) assembled camera platform, tilt sensing remote control (with camera platform internals visible in background), and remote control internals.</p> 

<img src="https://user-images.githubusercontent.com/113747791/191139676-475fb703-f1bf-4415-8959-d996843d879b.jpg" width="200"> <img src="https://user-images.githubusercontent.com/113747791/191139634-77254d5a-5a11-4a9b-b076-ee95fbe13af0.jpg" width="200"> <img src="https://user-images.githubusercontent.com/113747791/191140378-631eb1d3-ed77-4696-b6f8-71539e5e10d9.jpg" width="200">

<p>The device features fully wireless operation, with the remote control sending actuation signals to the platform. The camera platform will rotate and elevate/depress according to a comparison of its own state and that of the remote control.</p>

<p>The state estimation and feedback control of the system relied entirely on a pair of inertial measurement units (IMUs), consisting of an MPU-9250 on the platform and an MPU-6050 on the remote controller. This was perhaps a bit of a risky choice, as these inexpensive IMU chips do suffer a large amount of background noise in their measurements. However, our signal filter and feedback controller implementation was robust enough to effectively control the system.</p>

<p>The wiring of the device are as follows:</p>
<img src="https://user-images.githubusercontent.com/113747791/191141739-d6079753-5cf6-444b-a487-4d5f07f1bf83.png" width = 250> <img src="https://user-images.githubusercontent.com/113747791/191141815-7eddf583-0896-4579-b71c-ace7b58204ff.png" width = 275>


