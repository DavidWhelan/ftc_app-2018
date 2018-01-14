package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

/**
 * Created by David on 10/29/2017.
 */

public class PID
{
    public double kp, ki, kd, bias;
    public double previous_error = 0, error_sum = 0;
    public double setpoint;

    public ElapsedTime timer = new ElapsedTime();

    public PID()
    {
        kp = 0;
        ki = 0;
        kd = 0;
        setpoint = 0;
        timer.reset();
    }

    public PID(double kp, double ki, double kd, double setpoint)
    {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.setpoint = setpoint;
        timer.reset();
    }

    public double calculate(double actual)
    {
        double elapsed_iteration = timer.time();

        timer.reset();

        double error = setpoint - actual;

        error_sum += error* elapsed_iteration;

        double d = (error - previous_error)/elapsed_iteration;

        double out = kp * error + ki * error_sum + kd * d + bias;
        previous_error = error;
        return out;
    }

    public void set_tunings(double kp, double ki, double kd)
    {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public void reset_PID()
    {
        timer.reset();
    }

}
