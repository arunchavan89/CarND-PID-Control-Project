#ifndef PID_H
#define PID_H

#include <vector>
#define flag_Twiddle 0

class PID {
public:
    /**
     * Constructor
     */
    PID();

    /**
     * Destructor.
     */
    virtual ~PID();

    /**
     * Initialize PID.
     * @param (Kp_, Ki_, Kd_) The initial PID coefficients
     */
    void Init(double Kp_, double Ki_, double Kd_);

    /**
     * Update the PID error variables given cross track error.
     * @param cte The current cross track error
     */
    void UpdateError(double cte);

    /**
     * Calculate the total PID error.
     * @output The total PID error
     */
    double TotalError();

    /**
    * Twiddlw algorithm.
    * @output  PID coefficients
    */
    void Twiddle();

private:
    /**
     * PID Errors
     */
    double p_error;
    double i_error;
    double d_error;

    /**
     * PID Coefficients
     */
    double Kp;
    double Ki;
    double Kd;

    //previous cte error
    double prev_cte;

#if flag_Twiddle

    double best_err;
    double err;

    bool first_iteration;
    bool second_iteration;
    bool third_iteration;
    bool first;

    long int step;
    int index;
    int iteration;

    std::vector<double> p;
    std::vector<double> dp;

#endif // flag_Twiddle
};

#endif  // PID_H