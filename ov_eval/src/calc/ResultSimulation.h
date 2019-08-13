#ifndef OV_EVAL_SIMULATION_H
#define OV_EVAL_SIMULATION_H

#include <fstream>
#include <sstream>
#include <random>
#include <string>
#include <unordered_map>

#include <ros/ros.h>
#include <Eigen/Eigen>

#include "utils/Statistics.h"
#include "utils/Math.h"
#include "utils/Loader.h"

#ifdef HAVE_PYTHONLIBS

// import the c++ wrapper for matplot lib
// https://github.com/lava/matplotlib-cpp
// sudo apt-get install python-matplotlib python-numpy python2.7-dev
#include "plot/matplotlibcpp.h"

#endif


namespace ov_eval {



    /**
     * @brief A single simulation run (the full state not just pose)
     */
    class ResultSimulation {

    public:


        /**
         * @brief Default constructor that will load our data from file
         * @param path_est Path to the estimate text file
         * @param path_std Path to the standard deviation file
         * @param path_gt Path to the groundtruth text file
         */
        ResultSimulation(std::string path_est, std::string path_std, std::string path_gt);


        /**
         * @brief Will plot the state error and its three sigma bounds
         * @param doplotting True if you want to display the plots
         * @param max_time Max number of second we want to plot
         */
        void plot_state(bool doplotting, double max_time=INFINITY);

        /**
         * @brief Will plot the state imu camera offset and its sigma bound
         * @param doplotting True if you want to display the plots
         * @param max_time Max number of second we want to plot
         */
        void plot_timeoff(bool doplotting, double max_time=INFINITY);

        /**
         * @brief Will plot the camera calibration intrinsics
         * @param doplotting True if you want to display the plots
         * @param max_time Max number of second we want to plot
         */
        void plot_cam_instrinsics(bool doplotting, double max_time=INFINITY);

        /**
         * @brief Will plot the camera calibration extrinsic transform
         * @param doplotting True if you want to display the plots
         * @param max_time Max number of second we want to plot
         */
        void plot_cam_extrinsics(bool doplotting, double max_time=INFINITY);


    protected:

        // Trajectory data (loaded from file and timestamp intersected)
        std::vector<Eigen::VectorXd> est_state, gt_state;
        std::vector<Eigen::VectorXd> state_cov;

        // Calculated error values
        Statistics error_ori[3], error_pos[3], error_vel[3], error_bg[3], error_ba[3]; // state
        Statistics error_time; // timeoff
        std::vector<std::vector<Statistics>> error_cam_k, error_cam_d; // cameras intrinsics
        std::vector<std::vector<Statistics>> error_cam_ori, error_cam_pos; // cameras extrinsics


#ifdef HAVE_PYTHONLIBS

    /**
     * @brief Plots three different statistic values and sigma bounds
     * @param sx X-axis error
     * @param sy Y-axis error
     * @param sz Z-axis error
     * @param color_err MATLAB color string for error line (blue, red, etc.)
     * @param color_std MATLAB color string for deviation (blue, red, etc.)
     */
    void plot_3errors(ov_eval::Statistics sx, ov_eval::Statistics sy, ov_eval::Statistics sz, std::string color_err, std::string color_std) {

        // Zero our time arrays
        double starttime1 = (sx.timestamps.empty())? 0 : sx.timestamps.at(0);
        double endtime1 = (sx.timestamps.empty())? 0 : sx.timestamps.at(sx.timestamps.size()-1);
        for(size_t i=0; i<sx.timestamps.size(); i++) {
            sx.timestamps.at(i) -= starttime1;
        }
        double starttime2 = (sy.timestamps.empty())? 0 : sy.timestamps.at(0);
        double endtime2 = (sy.timestamps.empty())? 0 : sy.timestamps.at(sy.timestamps.size()-1);
        for(size_t i=0; i<sy.timestamps.size(); i++) {
            sy.timestamps.at(i) -= starttime2;
        }
        double starttime3 = (sz.timestamps.empty())? 0 : sz.timestamps.at(0);
        double endtime3 = (sz.timestamps.empty())? 0 : sz.timestamps.at(sz.timestamps.size()-1);
        for(size_t i=0; i<sz.timestamps.size(); i++) {
            sz.timestamps.at(i) -= starttime3;
        }

        // Parameters that define the line styles
        std::map<std::string, std::string> params_value, params_bound;
        //params_value.insert({"label","error"});
        params_value.insert({"linestyle","-"});
        params_value.insert({"color",color_err});
        //params_bound.insert({"label","3 sigma bound"});
        params_bound.insert({"linestyle","--"});
        params_bound.insert({"color",color_std});

        // Plot our error value
        matplotlibcpp::subplot(3,1,1);
        matplotlibcpp::plot(sx.timestamps, sx.values, params_value);
        if(!sx.values_bound.empty()) {
            matplotlibcpp::plot(sx.timestamps, sx.values_bound, params_bound);
            for(size_t i=0; i<sx.timestamps.size(); i++) {
                sx.values_bound.at(i) *= -1;
            }
            matplotlibcpp::plot(sx.timestamps, sx.values_bound, params_bound);
        }
        matplotlibcpp::xlim(0.0,endtime1-starttime1);

        // Plot our error value
        matplotlibcpp::subplot(3,1,2);
        matplotlibcpp::plot(sy.timestamps, sy.values, params_value);
        if(!sy.values_bound.empty()) {
            matplotlibcpp::plot(sy.timestamps, sy.values_bound, params_bound);
            for(size_t i=0; i<sy.timestamps.size(); i++) {
                sy.values_bound.at(i) *= -1;
            }
            matplotlibcpp::plot(sy.timestamps, sy.values_bound, params_bound);
        }
        matplotlibcpp::xlim(0.0,endtime2-starttime2);

        // Plot our error value
        matplotlibcpp::subplot(3,1,3);
        matplotlibcpp::plot(sz.timestamps, sz.values, params_value);
        if(!sz.values_bound.empty()) {
            matplotlibcpp::plot(sz.timestamps, sz.values_bound, params_bound);
            for(size_t i=0; i<sz.timestamps.size(); i++) {
                sz.values_bound.at(i) *= -1;
            }
            matplotlibcpp::plot(sz.timestamps, sz.values_bound, params_bound);
        }
        matplotlibcpp::xlim(0.0,endtime3-starttime3);

    }

        /**
         * @brief Plots three different statistic values and sigma bounds
         * @param sx Error one
         * @param sy Error two
         * @param sz Error three
         * @param sk Error four
         * @param color_err MATLAB color string for error line (blue, red, etc.)
         * @param color_std MATLAB color string for deviation (blue, red, etc.)
         */
        void plot_4errors(ov_eval::Statistics sx, ov_eval::Statistics sy, ov_eval::Statistics sz, ov_eval::Statistics sk, std::string color_err, std::string color_std) {

            // Zero our time arrays
            double starttime1 = (sx.timestamps.empty())? 0 : sx.timestamps.at(0);
            double endtime1 = (sx.timestamps.empty())? 0 : sx.timestamps.at(sx.timestamps.size()-1);
            for(size_t i=0; i<sx.timestamps.size(); i++) {
                sx.timestamps.at(i) -= starttime1;
            }
            double starttime2 = (sy.timestamps.empty())? 0 : sy.timestamps.at(0);
            double endtime2 = (sy.timestamps.empty())? 0 : sy.timestamps.at(sy.timestamps.size()-1);
            for(size_t i=0; i<sy.timestamps.size(); i++) {
                sy.timestamps.at(i) -= starttime2;
            }
            double starttime3 = (sz.timestamps.empty())? 0 : sz.timestamps.at(0);
            double endtime3 = (sz.timestamps.empty())? 0 : sz.timestamps.at(sz.timestamps.size()-1);
            for(size_t i=0; i<sz.timestamps.size(); i++) {
                sz.timestamps.at(i) -= starttime3;
            }
            double starttime4 = (sk.timestamps.empty())? 0 : sk.timestamps.at(0);
            double endtime4 = (sk.timestamps.empty())? 0 : sk.timestamps.at(sk.timestamps.size()-1);
            for(size_t i=0; i<sk.timestamps.size(); i++) {
                sk.timestamps.at(i) -= starttime4;
            }

            // Parameters that define the line styles
            std::map<std::string, std::string> params_value, params_bound;
            //params_value.insert({"label","error"});
            params_value.insert({"linestyle","-"});
            params_value.insert({"color",color_err});
            //params_bound.insert({"label","3 sigma bound"});
            params_bound.insert({"linestyle","--"});
            params_bound.insert({"color",color_std});

            // Plot our error value
            matplotlibcpp::subplot(4,1,1);
            matplotlibcpp::plot(sx.timestamps, sx.values, params_value);
            if(!sx.values_bound.empty()) {
                matplotlibcpp::plot(sx.timestamps, sx.values_bound, params_bound);
                for(size_t i=0; i<sx.timestamps.size(); i++) {
                    sx.values_bound.at(i) *= -1;
                }
                matplotlibcpp::plot(sx.timestamps, sx.values_bound, params_bound);
            }
            matplotlibcpp::xlim(0.0,endtime1-starttime1);

            // Plot our error value
            matplotlibcpp::subplot(4,1,2);
            matplotlibcpp::plot(sy.timestamps, sy.values, params_value);
            if(!sy.values_bound.empty()) {
                matplotlibcpp::plot(sy.timestamps, sy.values_bound, params_bound);
                for(size_t i=0; i<sy.timestamps.size(); i++) {
                    sy.values_bound.at(i) *= -1;
                }
                matplotlibcpp::plot(sy.timestamps, sy.values_bound, params_bound);
            }
            matplotlibcpp::xlim(0.0,endtime2-starttime2);

            // Plot our error value
            matplotlibcpp::subplot(4,1,3);
            matplotlibcpp::plot(sz.timestamps, sz.values, params_value);
            if(!sz.values_bound.empty()) {
                matplotlibcpp::plot(sz.timestamps, sz.values_bound, params_bound);
                for(size_t i=0; i<sz.timestamps.size(); i++) {
                    sz.values_bound.at(i) *= -1;
                }
                matplotlibcpp::plot(sz.timestamps, sz.values_bound, params_bound);
            }
            matplotlibcpp::xlim(0.0,endtime3-starttime3);

            // Plot our error value
            matplotlibcpp::subplot(4,1,4);
            matplotlibcpp::plot(sk.timestamps, sk.values, params_value);
            if(!sk.values_bound.empty()) {
                matplotlibcpp::plot(sk.timestamps, sk.values_bound, params_bound);
                for(size_t i=0; i<sk.timestamps.size(); i++) {
                    sk.values_bound.at(i) *= -1;
                }
                matplotlibcpp::plot(sk.timestamps, sk.values_bound, params_bound);
            }
            matplotlibcpp::xlim(0.0,endtime4-starttime4);

        }

#endif


    };


}



#endif //OV_EVAL_SIMULATION_H