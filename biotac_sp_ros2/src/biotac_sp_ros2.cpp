#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

extern "C" {
    #include "biotac_sp_ros2/cheetah.h"
    #include "biotac_sp_ros2/biotac.h"
}

using namespace std::chrono_literals;

class BioTacSPNode : public rclcpp::Node
{
public:
    BioTacSPNode() : Node("biotac_sp_ros2")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("biotac_sp_ros2", 10);
        timer_ = this->create_wall_timer(10ms, std::bind(&BioTacSPNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Biotac SP ROS 2 node started. Publishing on /biotac_sp_ros2");

        // --- Define BioTac configuration ---
        biotac_.spi_clock_speed = BT_SPI_BITRATE_KHZ_DEFAULT;
        biotac_.number_of_biotacs = 0;
        biotac_.sample_rate_Hz = BT_SAMPLE_RATE_HZ_DEFAULT;
        biotac_.frame.frame_type = 0;
        biotac_.batch.batch_frame_count = BT_FRAMES_IN_BATCH_DEFAULT;
        biotac_.batch.batch_ms = BT_BATCH_MS_DEFAULT;

        number_of_samples_ = BT_SAMPLE_RATE_HZ_DEFAULT;

        if (MAX_BIOTACS_PER_CHEETAH != 3 && MAX_BIOTACS_PER_CHEETAH != 5)
        {
            bt_display_errors(BT_WRONG_MAX_BIOTAC_NUMBER);
            rclcpp::shutdown();
        }

        // --- Initialise Cheetah ---
        ch_handle_ = bt_cheetah_initialize(&biotac_);

        for (int i = 0; i < MAX_BIOTACS_PER_CHEETAH; ++i)
        {
            BioTac err = bt_cheetah_get_properties(ch_handle_, i + 1, &biotac_property_[i]);
            if (biotac_property_[i].bt_connected == YES)
            {
                biotac_.number_of_biotacs++;
            }

            if (err)
            {
                bt_display_errors(err);
                rclcpp::shutdown();
            }
        }

        if (biotac_.number_of_biotacs == 0)
        {
            bt_display_errors(BT_NO_BIOTAC_DETECTED);
            rclcpp::shutdown();
        }
        else
        {
            std::cout << "BioTac(s) detected: " << biotac_.number_of_biotacs << std::endl;
        }
    }

    ~BioTacSPNode()
    {
        bt_cheetah_close(ch_handle_);
    }

private:
    void timer_callback()
    {
        static int results[4][162] = {0};
        static int results_vec[162] = {0};
        std::stringstream s_results;

        // --- Buffer and batch config ---
        data_ = bt_configure_save_buffer(number_of_samples_);
        BioTac err = bt_cheetah_configure_batch(ch_handle_, &biotac_, number_of_samples_);
        if (err < 0)
        {
            bt_display_errors(err);
            rclcpp::shutdown();
        }

        memset(results, 0, sizeof results);
        for (int i = 0; i < 1; ++i)
        {
            bt_cheetah_collect_1_batch(ch_handle_, &biotac_, data_, results);
        }

        memset(results_vec, 0, sizeof results_vec);
        for (int k = 0; k < 4; ++k)
        {
            for (int l = 0; l < 162; ++l)
            {
                results_vec[l] += results[k][l] / 4.0;
            }
        }

        s_results << this->now().seconds();
        for (int l = 0; l < 162; ++l)
        {
            s_results << "," << results_vec[l];
        }

        auto msg = std_msgs::msg::String();
        msg.data = s_results.str();
        publisher_->publish(msg);

        free(data_);
    }

    // ROS 2
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // BioTac/Cheetah
    bt_info biotac_;
    bt_property biotac_property_[MAX_BIOTACS_PER_CHEETAH];
    bt_data *data_;
    Cheetah ch_handle_;
    int number_of_samples_;
};

//=========================================================================
// MAIN
//=========================================================================
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BioTacSPNode>());
    rclcpp::shutdown();
    return 0;
}
