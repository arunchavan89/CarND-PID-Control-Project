#include "PID.h"
#include <limits>       // std::numeric_limits
#include <iostream>
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID()
{

}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_)
{
    /**
     * Initialize PID coefficients (and errors, if needed)
     */
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
    prev_cte = 0.0;

#if flag_Twiddle    
    step = 0;
    best_err = std::numeric_limits<double>::max();
    err = 0.0;
    first_iteration = true;
    second_iteration = false;
    third_iteration = false;
    first = true;
    index = 0;
    iteration = 0;
    p = { 0.05, 0.0001, 1.5 };
    dp = { 0.01, 0.0001, 0.3 };
#endif // flag_Twiddle
}

void PID::UpdateError(double cte)
{
    /**
     * Update PID errors based on cte.
     */
    p_error = cte;
    i_error += cte;
    d_error = cte - prev_cte;
    prev_cte = cte;
}

double PID::TotalError()
{
    /**
     * Calculate and return the total error
     */
    double steer_value = -Kp * p_error - Ki * i_error - Kd * d_error;

    return steer_value;
}

void PID::Twiddle()
{
#if flag_Twiddle  
    err += p_error * p_error;

    if (((step % 100) == 0) && (step != 0))
    {
        step = 0;
        if (iteration == 0)
        {
            first_iteration = true;
        }
        else if (iteration == 1)
        {
            second_iteration = true;
        }
        else if (iteration == 2)
        {
            third_iteration = true;
            index = (index + 1) % 3;
        }

        if ((dp[0] + dp[1] + dp[2]) > 0.2)
        {
            if (first)
            {
                best_err = err / 500;
            }

            if (first_iteration)
            {
                p[index] += dp[index];
                first_iteration = false;
                std::cout << "first_iteration: " << std::endl;
            }

            if (second_iteration)
            {
                err = err / 500;
                if (err < best_err)
                {
                    best_err = err;
                    dp[index] *= 1.1;
                }
                else
                {
                    p[index] -= 2.0*dp[index];
                }
                second_iteration = false;
                std::cout << "second_iteration: " << std::endl;
            }

            if (third_iteration)
            {
                err = err / 500;
                if (err < best_err)
                {
                    best_err = err;
                    dp[index] *= 1.1;
                }
                else
                {
                    p[index] += dp[index];
                    dp[index] *= 0.9;
                }
                third_iteration = false;
                std::cout << "third_iteration: " << std::endl;
            }
        }
        else
        {
            std::cout << "Best: " << Kp << "," << Ki << "," << Kd << std::endl;
        }

        iteration = (iteration + 1) % 3;

        Kp = p[0];
        Ki = p[1];
        Kd = p[2];
        std::cout << p[0] << "," << p[1] << "," << p[2] << std::endl;
    }

    step++;
#endif // flag_Twiddle
}
