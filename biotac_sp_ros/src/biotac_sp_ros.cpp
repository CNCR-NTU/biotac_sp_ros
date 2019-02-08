/*=========================================================================
| (c) 2011-2012  SynTouch LLC
|--------------------------------------------------------------------------
| Project : BioTac C Library for Cheetah
| File    : example.c
| Authors : Gary Lin (gary.lin@syntouchllc.com)
|			Tomonori Yamamoto (tomonori.yamamoto@syntouchllc.com)
|			Jeremy Fishel (jeremy.fishel@syntouchllc.com)
|--------------------------------------------------------------------------
| Function: Main function to run an example program
|--------------------------------------------------------------------------
| Redistribution and use of this file in source and binary forms, with
| or without modification, are permitted.
|
| THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
| "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
| LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
| FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
| COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
| INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
| BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
| LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
| CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
| LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
| ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
| POSSIBILITY OF SUCH DAMAGE.
 ========================================================================*/

//=========================================================================
// INCLUDES
//=========================================================================
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>

extern "C"{
    #include <biotac_sp_ros/cheetah.h>
    
}
extern "C"{
    #include <biotac_sp_ros/biotac.h>
}

using namespace std;
//=========================================================================
// MAIN PROGRAM
//=========================================================================
int main(int argc,char **argv)
{
	/****************************/
	/* --- Define variables --- */
	/****************************/
	bt_info biotac;
    bt_property biotac_property[MAX_BIOTACS_PER_CHEETAH];
	bt_data *data;
	BioTac bt_err_code;
	Cheetah ch_handle;
    
    ros::init(argc, argv,"biotac_sp_ros");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    ros::Publisher biotac_sp_pub = n.advertise<std_msgs::String>("biotac_sp_ros", 1000);
    
	int i;
	int length_of_data_in_second;
	int number_of_samples;
	int number_of_loops;

    /**************************************************************************/
	/* --- Initialize BioTac settings (only default values are supported) --- */
    /**************************************************************************/
	biotac.spi_clock_speed = BT_SPI_BITRATE_KHZ_DEFAULT;
	biotac.number_of_biotacs = 0;
	biotac.sample_rate_Hz = BT_SAMPLE_RATE_HZ_DEFAULT;
	biotac.frame.frame_type = 0;
	biotac.batch.batch_frame_count = BT_FRAMES_IN_BATCH_DEFAULT;
	biotac.batch.batch_ms = BT_BATCH_MS_DEFAULT;

	// Set the duration of the run time
	length_of_data_in_second = 3;
	number_of_samples = (int)(BT_SAMPLE_RATE_HZ_DEFAULT); // * length_of_data_in_second)

	// Check if any initial settings are wrong
	if (MAX_BIOTACS_PER_CHEETAH != 3 && MAX_BIOTACS_PER_CHEETAH != 5)
	{
		bt_err_code = BT_WRONG_MAX_BIOTAC_NUMBER;
		bt_display_errors(bt_err_code);
		exit(1);
	}

    /******************************************/
	/* --- Initialize the Cheetah devices --- */
    /******************************************/
	ch_handle = bt_cheetah_initialize(&biotac);

	/*********************************************************/
	/* --- Get and print out properties of the BioTac(s) --- */
	/*********************************************************/
	for (i = 0; i < MAX_BIOTACS_PER_CHEETAH; i++)
	{
		bt_err_code = bt_cheetah_get_properties(ch_handle, i+1, &(biotac_property[i]));

		if (biotac_property[i].bt_connected == YES)
		{
			(biotac.number_of_biotacs)++;
		}

		if (bt_err_code)
		{
			bt_display_errors(bt_err_code);
			exit(1);
		}
	}

	if (biotac.number_of_biotacs == 0)
	{
		bt_err_code = BT_NO_BIOTAC_DETECTED;
		bt_display_errors(bt_err_code);
		return bt_err_code;
	}
	else
	{
		printf("\n%d BioTac(s) detected.\n\n", biotac.number_of_biotacs);
	}
	static int results[4][162]; 
    static int results_vec[162];

    while (ros::ok())
    {
        std::stringstream s_results;
        std_msgs::String msg;
        /*************************************/
        /* --- Configure the save buffer --- */
        /*************************************/
        //printf("Number of samples: %d\n",number_of_samples);
        data = bt_configure_save_buffer(number_of_samples);

        /*******************************/
        /* --- Configure the batch --- */
        /*******************************/
        bt_err_code = bt_cheetah_configure_batch(ch_handle, &biotac, number_of_samples);
        if (bt_err_code < 0)
        {
            bt_display_errors(bt_err_code);
            exit(1);
        }
        else
        {
            //printf("\nConfigured the batch\n");
        }

        /**********************************************************/
        /* --- Collect 4 samples in a second and publish data --- */
        /**********************************************************/
        number_of_loops=1;
        //printf("Start collecting BioTac...\n");
        
        memset(results, 0, sizeof results);

        for (i = 0; i < number_of_loops; i++)

        {
            bt_cheetah_collect_1_batch(ch_handle, &biotac, data, results);
        }

        memset(results_vec, 0, sizeof results_vec);

        for(int k=0; k<4; ++k)
        {
            for (int l=0; l<162;++l)
            {
                results_vec[l]+=results[k][l]/4.;
            }
        }

        s_results<<"[";
        s_results<<ros::Time::now();
        for (int l=0; l<162;++l)
        {
                s_results<<","<<results_vec[l];   
        }

        free(data);
        s_results<<"]";
        msg.data = s_results.str();
        ROS_INFO("%s", msg.data.c_str());
        biotac_sp_pub.publish(msg);
        loop_rate.sleep();
    }
    bt_cheetah_close(ch_handle);

    return 0;
}
