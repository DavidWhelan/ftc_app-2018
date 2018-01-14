package org.firstinspires.ftc.teamcode.Diva;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by David on 11/26/2017.
 */

public class DivaDrive
{
    //This ensures that the methods can acess the motors you have on your own bot.
    public DcMotor front_left;
    public DcMotor back_left;
    public DcMotor front_right;
    public DcMotor back_right;
    //Checks to ensure that the Turn PID is reset
    private boolean turn_reset = false;
    //Create a new PID to handle the turns
    DivaPID turn_control = new DivaPID();
    //Gyro To Read From
    ModernRoboticsI2cGyro gyro;
    //Input the motors in the correct place. This is to ensure that this class can acess the motors and gyro
    public DivaDrive(DcMotor front_left, DcMotor back_left, DcMotor front_right, DcMotor back_right, ModernRoboticsI2cGyro gyro)
    {
        this.front_left = front_left;
        this.front_right = front_right;
        this.back_left = back_left;
        this.back_right = back_right;
        this.gyro = gyro;

    }
    //Required by turn_to_heading to easily stop motors
    public void stop()
    {
        front_left.setPower(0);
        front_right.setPower(0);
        back_right.setPower(0);
        back_left.setPower(0);
    }
    //Takes in the power, heading, and error and turns the robot to the desired absolute heading
    public boolean turn_to_heading(double power, double heading, double error)
    {
        //Reverse heading because it is reversed on modernrobotics gyro
        heading = -heading;
        //Ensure that the timer in PID is reset
        if(!turn_reset)
        {
            turn_control.reset_PID();
            turn_reset = true;
        }
        //Set the setpoint as the heading
        turn_control.setpoint = heading;
        //These are the PID Values. DO NOT USE ANY I. It is currently 0. If it stops early lower the third vale or increase the first. If it stops to late lower the forst values or increase the third value
        turn_control.set_tunings(.03, 0, .0032);
        //Pass in where we are at and the PID does the calculations for the amount it needs to change
        double to_change = turn_control.calculate(gyro.getIntegratedZValue());
        //Apply yhe Change to the Motors
        front_left.setPower(-(power * to_change));
        back_left.setPower(-(power * to_change));

        front_right.setPower((power * to_change));
        back_right.setPower((power * to_change));
        //Check to see if we made it and if we have then stop and return true
        boolean correct_angle = Math.abs(gyro.getIntegratedZValue() - turn_control.setpoint) < error;

        if(correct_angle)
        {
            turn_reset = false;
            stop();
            return true;
        }
        return false;
    }
}
