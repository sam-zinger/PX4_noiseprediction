

/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <complex.h>

#include <uORB/uORB.h>
#include <uORB/topics/raw_PWM.h>

//export main file
__EXPORT int px4_simple_app_main(int argc, char *argv[]);

//*******************************************************************************************************************************
// global initilisation
//*******************************************************************************************************************************

//must be initilaised here for pointer decleration
double result_mag, result_phase;

//pointer decleration for passing to function
double* mag_sum = &result_mag;
double* phase_sum = &result_phase;

//complex addition function init
void complex_addition(double mag1, double phase1, double mag2, double phase2);

//main loop
int px4_simple_app_main(int argc, char *argv[])
{
	//PX4_INFO is equivalent to printf
	PX4_INFO("Application starting");

       //*******************************************************************************************************************************
       // local paramater initilisation
       //*******************************************************************************************************************************


        struct noise_info {
              //5 length array for 5 frequancies (can be changed)
              double mag[5], phase[5];
        };

        //structure declerisation containing magnitude and phase infomation for each microphone point
        struct noise_info point1 = { .mag = {0}, .phase = {0} };
        struct noise_info point2 = { .mag = {0}, .phase = {0} };
        struct noise_info point3 = { .mag = {0}, .phase = {0} };
        struct noise_info point4 = { .mag = {0}, .phase = {0} };
        struct noise_info point5 = { .mag = {0}, .phase = {0} };

        //length from one motor to another
        double airframe_l = 0.45;
        //length of propeller blade
        double D_blade = 0.5;

        //arbitary xyz coordinate of test(microphone) points
         double test_point_xyz[5][3] = {
        {0, 0, -2},
        {1, 0, -2},
        {-0.5, -0.866, -2},
        {-0.5, 0.866, -2},
        {0, 0, -4}
        };

        //xyz coordinates of motors
        double airframe_motors_xyz[4][3] = {
        {airframe_l, airframe_l, 0},
        {-1 * airframe_l, airframe_l, 0},
        {-1 * airframe_l, -1 * airframe_l, 0},
        {airframe_l, -1 * airframe_l, 0}
        };

        //speed of sound
        double c = 343;
        //freq
        double f[5] = {100, 200, 300, 400, 500};
        //angular freq w
        double w[5] = { (2 * M_PI * f[0]), (2 * M_PI * f[1]), (2 * M_PI * f[2]), (2 * M_PI * f[3]), (2 * M_PI * f[4]) } ;
        //scalar wavenumber k
        double k[5] = { (w[0] / c), (w[1] / c), (w[2] / c), (w[3] / c), (w[4] / c) };

        //scaler value required due to noise calc assumptions
        double scale_noise = 1921083679;

        /*** Following initilisations are for basic place holder variables, helping in simplicity of coding and readability ***/
        double noise_at_point_per_motor = 0;
        int current_delay;
        double current_distance;
        double current_motor_noise;
        double phase_at_point;
        double time_of_flight;
        double current_mag;
        double current_phase;
        double current_motor_phase;
        double rotation;
        double saved_motor_phase = 0;
        double rawmotor[4];

	//gives current position of each plade in terms of phase shift
        double blade_phase[4] = { 0 };

        //distance matrix - shows distance between all pairs of motor/microphone points
        double distance_matrix[4][5];

        //delay matrix - shows delay between all pairs of motor/microphone points in multiples of sampling time
        int delay_matrix[4][5] = { 0 };

        //3-D memory matrix. [required_TS][motor][point]. hence will show the calculated noise for each motor-point pair.
        //required_TS can be any number above the max delay in delay_matrix
        double memory_matrix_mag[20][4][5] = { 0 };
        double memory_matrix_phase[20][4][5] = { 0 };

        //init to save raw data
	double save[4] = {0};

        //error counter if info isnt recieved two timesteps in a row
        int no_data_twice = 0;

        //TS value (st 1kHz sampling)
        double TS_value = 0.001;

        //distance travelled for one TS for speed of sound
        double sound_dis_1TS = c * TS_value;

        //how many times do we want the program to run and provide output
        int amount_timesteps = 1200;

        //how often we want new data (in ms) - note done decrease below 1ms otherwise serious error will occur
	int time_update_period = 1;


        //*******************************************************************************************************************************
        // one time initilisation for each motor-microphone distance and time delay calculation
        //*******************************************************************************************************************************
        //looping over every motor and microphone point
        for (int point_index = 0; point_index < 5; point_index++) {
                for (int motor_index = 0; motor_index < 4; motor_index++) {
			//calculating distance in 3-D euclideon space
                        distance_matrix[motor_index][point_index] = sqrt(pow((airframe_motors_xyz[motor_index][0] - test_point_xyz[point_index][0]), 2) + pow((airframe_motors_xyz[motor_index][1] - test_point_xyz[point_index][1]), 2) + pow((airframe_motors_xyz[motor_index][2] - test_point_xyz[point_index][2]), 2));
                        //testing if distance is greater then multiples of the distance travelled by sound in a timestep, amount of iterations will be delay
                        int i = 1;
                        while (distance_matrix[motor_index][point_index] > i * sound_dis_1TS) {
                                i++;
                        }
                        delay_matrix[motor_index][point_index] = i - 1;
                }
        }
        //*******************************************************************************************************************************
	//Initialisation and polling for sensor subscription
	//*******************************************************************************************************************************

	/* subscribe to sensor_combined topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(raw_PWM));
	/* limit the update rate to (1000/time_update_period) Hz */
	orb_set_interval(sensor_sub_fd, time_update_period);

        //wait for topic to update
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
	};

        /*** the main for loop which goes through a set amount of timesteps ***/
	for (int timesteps = 0; timesteps < amount_timesteps; timesteps++) {

                /*** following lines are vital for polling and error messages ***/
		//wait for sensor update of 1 file descriptor for 100ns
                // data recieved, poll_ret = 1
                // data not recieved, poll_ret = 0
                // error, poll_ret < 0
		int poll_ret = px4_poll(fds, 1, 1.0001*time_update_period);

                //we didnt recieve data in time
                if (poll_ret == 0) {
                         //if this occurs twice in a row create error and leave modue
                        if (no_data_twice == 2) {
                                PX4_ERR("Didnt recieve data twice in a row");
                                break;
                        }

                        //when no data recieved, use last timesteps data. (note occurs very rarely and for error control, so to not halt system)
                        rawmotor[0] = save[0];
			rawmotor[1] = save[1];
			rawmotor[2] = save[2];
			rawmotor[3] = save[3];

                        no_data_twice += 1;
                }

                else if (poll_ret < 0) {
			//is extremely bad
			PX4_ERR("ERROR return value from poll(): %d", poll_ret);
                        break;
                }

                else if (fds[0].revents & POLLIN) {
			//obtained data for the first file descriptor
                        struct raw_PWM_s raw;
			//copy sensors raw data into local buffer
			orb_copy(ORB_ID(raw_PWM), sensor_sub_fd, &raw);

                        //save input data
			save[0] = raw.info[0];
			save[1] = raw.info[1];
			save[2] = raw.info[2];
			save[3] = raw.info[3];

                        //set motor as input data
			rawmotor[0] = raw.info[0];
			rawmotor[1] = raw.info[1];
			rawmotor[2] = raw.info[2];
			rawmotor[3] = raw.info[3];

                        //hard double error
                        no_data_twice = 0;
                }


                //*******************************************************************************************************************************
	        //Noise calculation
          	//*******************************************************************************************************************************

                /*  Loop over for each motor */
                for (int motor_index = 0; motor_index < 4; motor_index++) {

                        //convert PWM to RPM
                        rawmotor[motor_index] = 6 * 1000 * rawmotor[motor_index];

                        /* following three lines to find the blades current orientation (phase) */
                        rotation = rawmotor[motor_index]/(60*1000*time_update_period); //how far blade turn in a TS (note is never larger then a full revolution)
                        blade_phase[motor_index] += 2 * M_PI * rotation; //convert this remainder to phase

                        /* Normalise phase to be between 0 < phase < 2pi */
                        if (blade_phase[motor_index] > 2 * M_PI) {
                                blade_phase[motor_index] -= 2 * M_PI;
                        }

                        //convert RPM to noise
                        current_motor_noise = (pow(D_blade, 7) * pow(rawmotor[motor_index], 5)) / scale_noise;
                        current_motor_phase = blade_phase[motor_index];

                        /* Loop over for each microphone array point */
                        for (int point_index = 0; point_index < 5; point_index++) {

                                // simple placeholders
                                current_distance = distance_matrix[motor_index][point_index];
                                current_delay = delay_matrix[motor_index][point_index];

                                /* Think of the following for statement as a shift register, whereupon the delay for a certain motor/microphone pair is
                                found and its contents in the memory matrix shifted down with the new value placed at the top and the oldest (bottom value)
                                being pushed out and used for the current timesteps calculations   */
                                for (int current_delay_index = 0; current_delay_index <= current_delay; current_delay_index++) {
                                        if (current_delay_index == current_delay) {
                                                if (current_delay_index != 0) {
                                                        //store the most current data into the top of the memory buffer
                                                        memory_matrix_mag[current_delay_index - 1][motor_index][point_index] = current_motor_noise / (4 * M_PI * current_distance);
                                                        memory_matrix_phase[current_delay_index - 1][motor_index][point_index] = current_motor_phase;
                                                }
                                                else {
                                                        //in the event there is no needed delay
                                                        noise_at_point_per_motor = current_motor_noise / (4 * M_PI * current_distance);
                                                        saved_motor_phase = current_motor_phase;
                                                }
                                        }
                                        else if (current_delay_index != 0) {
                                                //shit values down memory buffer
                                                memory_matrix_mag[current_delay_index - 1][motor_index][point_index] = memory_matrix_mag[current_delay_index][motor_index][point_index];
                                                memory_matrix_phase[current_delay_index - 1][motor_index][point_index] = memory_matrix_phase[current_delay_index][motor_index][point_index];
                                        }
                                        else {
                                                //user value in lowest memory index
                                                noise_at_point_per_motor = memory_matrix_mag[0][motor_index][point_index];
                                                saved_motor_phase = memory_matrix_phase[0][motor_index][point_index];
                                        }


                                }

                                // time of flight will be delay value + 1 multiplied by timestep value
                                time_of_flight = (current_delay + 1) * TS_value;
                                current_mag = noise_at_point_per_motor;

                                /*** now we loop through all frequancies ***/
                                for (int freq_index = 0; freq_index < 5; freq_index++) {
                                        // Using sound propagation modelling equation:
                                        phase_at_point = saved_motor_phase + (k[freq_index] * current_distance) - (w[freq_index] * time_of_flight);

                                        /* Normalise phase to be between 0 < phase < 2pi */
                                        while ((-M_PI <= phase_at_point && phase_at_point <=  M_PI) == 0) {
                                                if (phase_at_point > M_PI) {
                                                        phase_at_point -= 2 * M_PI;
                                                }
                                                else {
                                                        phase_at_point += 2 * M_PI;
                                                }
                                        }

                                        current_phase = phase_at_point;

                                        /** switch statement will execute the current microphone array **/
                                        switch (point_index) {
                                        case(0):
                                                //add current motor with all previous motor points for this timesstep
                                                complex_addition(point1.mag[freq_index], point1.phase[freq_index], current_mag, current_phase);
                                                //update the structure with summed values
                                                point1.mag[freq_index] = *mag_sum;
                                                point1.phase[freq_index] = *phase_sum;
                                                break;
                                        case(1):
                                                complex_addition(point2.mag[freq_index], point2.phase[freq_index], current_mag, current_phase);
                                                point2.mag[freq_index] = *mag_sum;
                                                point2.phase[freq_index] = *phase_sum;
                                                break;
                                        case(2):
                                                complex_addition(point3.mag[freq_index], point3.phase[freq_index], current_mag, current_phase);
                                                point3.mag[freq_index] = *mag_sum;
                                                point3.phase[freq_index] = *phase_sum;
                                                break;
                                        case(3):
                                                complex_addition(point4.mag[freq_index], point4.phase[freq_index], current_mag, current_phase);
                                                point4.mag[freq_index] = *mag_sum;
                                                point4.phase[freq_index] = *phase_sum;
                                                break;
                                        case(4):
                                                complex_addition(point5.mag[freq_index], point5.phase[freq_index], current_mag, current_phase);
                                                point5.mag[freq_index] = *mag_sum;
                                                point5.phase[freq_index] = *phase_sum;
                                                break;
                                        default:
                                                break;
                                        }
                                }
                        }
                }

                /*** iterate for each frequancy ***/
                for (int i=0; i<5; i++) {

                       //so not to cause math error when using log function i.e log(0)
                        if (point1.mag[i] <= 1) {
                                point1.mag[i] = 1;
                        }
                        if (point2.mag[i] <= 1) {
                                point2.mag[i] = 1;
                        }
                        if (point3.mag[i] <= 1) {
                                point3.mag[i] = 1;
                        }
                        if (point4.mag[i] <= 1) {
                                point4.mag[i] = 1;
                        }
                        if (point5.mag[i] <= 1) {
                                point5.mag[i] = 1;
                        }

                        //output
                        PX4_INFO("frequancy = %d ", ((i + 1) * 100) );
                        PX4_INFO("%d mag1 = %f    phase1 = %f ", 1, 10*log10(point1.mag[i]), point1.phase[i]);
                        PX4_INFO("%d mag2 = %f    phase2 = %f ", 2, 10*log10(point2.mag[i]), point2.phase[i]);
                        PX4_INFO("%d mag3 = %f    phase3 = %f ", 3, 10*log10(point3.mag[i]), point3.phase[i]);
                        PX4_INFO("%d mag4 = %f    phase4 = %f ", 4, 10*log10(point4.mag[i]), point4.phase[i]);
                        PX4_INFO("%d,mag5 = %f    phase5 = %f \n", 5, 10*log10(point5.mag[i]), point5.phase[i]);

                        //reset values
                        point1.mag[i] = 0;
		        point1.phase[i] = 0;
		        point2.mag[i] = 0;
		        point2.phase[i] = 0;
		        point3.mag[i] = 0;
		        point3.phase[i] = 0;
		        point4.mag[i] = 0;
		        point4.phase[i] = 0;
		        point5.mag[i] = 0;
		        point5.phase[i] = 0;
                }
        }
	PX4_INFO("exiting");
	return 0;
}

/*** complex addition function ***/
void complex_addition(double mag1, double phase1, double mag2, double phase2) {
    double dummy_x, dummy_y;

    //convert to rectangular complex form
    dummy_x = (mag1 * cos(phase1)) + (mag2 * cos(phase2));
    dummy_y = (mag1 * sin(phase1)) + (mag2 * sin(phase2));

    //convert back to polar complx form
    result_mag = sqrt(pow(dummy_x, 2) + pow(dummy_y, 2));
    //to ensure no math error when possibly diciding by 0
    if ((dummy_x <= 0.000000001) && (dummy_x >= -0.000000001)) {
        dummy_x = 0.0000000001;
	dummy_y = 0.0;
    }
    result_phase = atan(dummy_y / dummy_x);

    //normalise phase to between -pi and pi
    while ((-M_PI <= result_phase && result_phase <= M_PI) == 0) {
        if (result_phase > M_PI) {
            result_phase -= 2 * M_PI;
        }
        else {
            result_phase += 2 * M_PI;
        }
    }

     //output
     mag_sum = &result_mag;
     phase_sum = &result_phase;

    return;
}

