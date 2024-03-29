Disclaimer

Do not have permission to clone PX4/PX4-Autopilot.git. Hence, is why a separate git repository had to be created. PLease note, for all PX4 files go to https://github.com/PX4/PX4-Autopilot.


Project Title

A simulated approach to active noise cancellation onboard a drone



Abstract

Most drones come pre-built with camera(s) and can capture videos in high quality, however with no current means to effectively record sound from such drones, due to the noise generated by the rotors, the full capabilities of these drones are not utilized. To combat this, noise cancellation techniques can be used. This project aims to provide the groundwork for such by analyzing key data from within a flight controller to estimate the noise generated by the drone. This will be done by using a Pixhawk simulation environment, utilizing pre-existing communication networks and modules to write the sound modelling system. The noise will then be predicted at a microphone array located beneath the drone, where it is outputted as a polar complex waveform with the calculations written in a C file. The results of such showing that the estimation of sound was a success considering the assumptions made and simulated context of the project. Thus, highlighting the feasibility of an active noise cancelling system on board a real drone.



Installation

The steps for installation are laid out very clearly on the PX4 user guide site (in the development section). Hence, follow the steps from this site and the program should work. Note when choosing a simulator use Gazebo with Software-In-The-Loop (SITL).

PX4 user guide link: https://docs.px4.io/master/en/development/development.html



Explanation

Will explain each section and how they work together, then an explanation of each individual files that were created or modified for the project. Of all the five files listed, four of them are responsible for the communication of information whereupon the last one is used for calculations. The first and second file is for topic creation which acts as the medium for information sharing. The third and fourth files are for the transmission of information and the fifth file is for the attainment and calculations of the information.

Each file will contain the full location in PX4-Autopilot.

    msg/raw_PWM.msg (created)

Is a simple file that creates the uORB topic. Contains a timestep integer and a 4 input float array called “info” containing PWM information for each motor.

    msg/CMakeLists.txt (modified)

Only contains a single additional line “raw_PWM.msg” which tells CMake to build the topic.

    src/modules/simulator/simulator.h (modified)

Only contains a single additional include statement to the raw_PWM topic. “#include <uORB/topics/raw_PWM.h>”

    src/modules/simulator/simulator_mavlink.cpp (modified)

This pre-existing file is a key module in the real-time processing of the drone in a simulated system. It also contains the PWM information needed for the noise prediction. Thus, the added lines of code in this file (in total 7 lines) is only to export this content. The user code is at the start of this large file and can be easily seen due to the obnoxious commenting stating the start and end of two chunks of user code. The only thing to note regarding this file is the advertise and publish uORB commands used.

    src/examples/px4_simple_app/px4_simple_app.c (modified)

This is the key file and although is stated to be modified there is no remaining code that was used in the original file. Hence, all aspects of this file is completely user defined. The step by step function of this file can be seen in the subsequent “A simulated approach to active noise cancellation onboard a drone” report. The behavior of all the features in this file is commented well, however a high-level view will be described.

Before attainment of the PWM information via uORB communication the distance and delay between each motor and microphone point is found. After which the PWM is converted to RPM and the magnitude and phase at each motor found. Then a memory buffer is applied to save the data for a certain amount of timesteps. The output from this buffer will be calculated to find the phase distortion and magnitude loss over. Finally, this will be superimposed with all other motors to find the net sound acoustics at each microphone point.



How to Use?

User defined constants include the location of motor and microphone array points. This includes other key information regarding the drone wanting to be simulated. i.e blade diameter. The running time of the application can also be altered. These are commented sufficiently throughout the px4_simple_app.c file so the specific variables will not be mentioned here.

To obtain a useful output, simply follow the PX4 guide to build the code for Gazebo. Specifically calling  “make px4_sitl gazebo” to start the build and use the environment. Then using commands to control the drone the “px4_simple_app” application is called and the output will simply appear on screen.



Credits

The use of the PX4 open-source code was essential in the development of this code. Hence a shoutout goes out to the amazing Pixhawk team who has created this platform.
