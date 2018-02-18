package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by David on 10/22/2017.
 */

public class Drive
{
    RobotHardware robot;
    PID drive_control = new PID();
    PID turn_control = new PID();

    double absolute_drive;

    private final double ticks_per_revolution = 1000;
    private final double drive_gear_teeth = 24;
    private final double wheel_gear_teeth = 32;

    private boolean drive_reset = false;
    private boolean turn_reset = false;
    public boolean pid_reset = false;

    String output = "unchanged";

    ElapsedTime drive_timer = new ElapsedTime();

    public Drive(RobotHardware robot)
    {
        this.robot = robot;
    }

    public void stop()
    {
        robot.front_left.setPower(0);
        robot.front_right.setPower(0);
        robot.back_right.setPower(0);
        robot.back_left.setPower(0);
    }

    public boolean turn_to_heading(double power, double heading, double error)
    {
        heading = -heading;
        if(!turn_reset)
        {
            turn_control.reset_PID();
            turn_reset = true;
        }

        turn_control.setpoint = heading;

        turn_control.set_tunings(.03, 0, .0032);

        double to_change = turn_control.calculate(robot.gyro.getIntegratedZValue());

        robot.front_left.setPower(-(power * to_change));
        robot.back_left.setPower(-(power * to_change));

        robot.front_right.setPower((power * to_change));
        robot.back_right.setPower((power * to_change));

        boolean correct_angle = Math.abs(robot.gyro.getIntegratedZValue() - turn_control.setpoint) < error;

        if(correct_angle)
        {
            turn_reset = false;
            stop();
            absolute_drive = heading;
            return true;
        }
        return false;
    }

    public boolean drive_straight_forward(double power, double angle_to_drive, boolean stop)
    {
        if(!pid_reset)
        {
            drive_control.reset_PID();
            pid_reset = true;
        }

        drive_control.setpoint = absolute_drive + angle_to_drive;

        drive_control.set_tunings(0.028, 0.015, 0.0);

        double angle  = robot.gyro.getIntegratedZValue();

        double to_change = drive_control.calculate(angle);

        robot.front_left.setPower(power - (power * to_change));
        robot.back_left.setPower(power - (power * to_change));

        robot.front_right.setPower(power + (power * to_change));
        robot.back_right.setPower(power + (power * to_change));

        if(stop)
        {
            pid_reset = false;
            stop();
        }
        return stop;
    }

    public boolean drive_straight_backward(double power, double angle_to_drive, boolean stop)
    {
        if(!pid_reset)
        {
            drive_control.reset_PID();
            pid_reset = true;
        }
        power = -power;
        drive_control.setpoint = absolute_drive + angle_to_drive;

        drive_control.set_tunings(0.028, 0.015, 0.0);

        double angle  = robot.gyro.getIntegratedZValue();

        double to_change = drive_control.calculate(angle);

        robot.front_left.setPower(power + (power * to_change));
        robot.back_left.setPower(power + (power * to_change));

        robot.front_right.setPower(power - (power * to_change));
        robot.back_right.setPower(power - (power * to_change));

        if(stop)
        {
            pid_reset = false;
            stop();
        }
        return stop;
    }

    public boolean distance_drive_forward(double power, double distance)
    {
        if(!drive_reset)
        {
            if(reset_encoders())
            {
                run_with_encoders();
                drive_reset = true;
            }

            else
            {
                return false;
            }
        }

        double wheel_rotation_per_turn = drive_gear_teeth/wheel_gear_teeth;

        double distance_per_turn = (Math.PI * 6) * wheel_rotation_per_turn;

        double total_wheel_turns = distance/distance_per_turn;

        double total_number_encoder_ticks = ticks_per_revolution * total_wheel_turns;

        double new_power = power - power*(robot.front_right.getCurrentPosition()/total_number_encoder_ticks) + 0.15;

        boolean correct_distance = robot.front_right.getCurrentPosition() >= total_number_encoder_ticks;

        if(drive_straight_forward(new_power, 0, correct_distance))
        {
            drive_reset = false;
            return true;
        }
        return false;

    }

    public boolean distance_drive_backward(double power, double distance)
    {
        distance = -distance;
        if(!drive_reset)
        {
            if(reset_encoders())
            {
                run_with_encoders();
                drive_reset = true;
            }

            else
            {
                return false;
            }
        }

        double wheel_rotation_per_turn = drive_gear_teeth/wheel_gear_teeth;

        double distance_per_turn = (Math.PI * 6) * wheel_rotation_per_turn;

        double total_wheel_turns = distance/distance_per_turn;

        double total_number_encoder_ticks = -ticks_per_revolution * total_wheel_turns;

        double new_power = power - power*(Math.abs(robot.front_right.getCurrentPosition())/total_number_encoder_ticks) + 0.15;

        boolean correct_distance = robot.front_right.getCurrentPosition() <= -total_number_encoder_ticks;

        if(drive_straight_backward(new_power, 0, correct_distance))
        {
            drive_reset = false;
            return true;
        }
        return false;

    }

    public boolean reset_encoders()
    {
        robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(robot.front_left.getCurrentPosition() == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public void run_with_encoders()
    {
        robot.front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void run_without_encoders()
    {
        robot.front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean score(String color, double red, double blue)
    {
        if(color.equals("blue"))
        {
            if((blue + 20) >= red)
            {
                robot.left_flicker_right();
            }
            else
            {
                robot.left_flicker_left();
            }
            return true;
        }
        else if(color.equals("red"))
        {
            if(red > (blue + 20))
            {
                robot.left_flicker_right();
            }
            else
            {
                robot.left_flicker_left();
            }
            return true;
        }
        else return false;

    }

    public boolean new_drive_forward(double distance)
    {
        if(!drive_reset)
        {
            if(reset_encoders())
            {
                run_with_encoders();
                drive_reset = true;
            }

            else
            {
                return false;
            }
        }

        double wheel_rotation_per_turn = drive_gear_teeth/wheel_gear_teeth;

        double distance_per_turn = (Math.PI * 6) * wheel_rotation_per_turn;

        double total_wheel_turns = distance/distance_per_turn;

        double total_number_encoder_ticks = ticks_per_revolution * total_wheel_turns;

        double new_power = new_power(robot.front_right.getCurrentPosition()/total_number_encoder_ticks);

        boolean correct_distance = robot.front_right.getCurrentPosition() >= total_number_encoder_ticks;

        if(drive_straight_forward(new_power, 0, correct_distance))
        {
            drive_reset = false;
            return true;
        }
        return false;
    }

    public boolean new_drive_backward(double distance)
    {
        distance = -distance;
        if(!drive_reset)
        {
            if(reset_encoders())
            {
                run_with_encoders();
                drive_reset = true;
            }

            else
            {
                return false;
            }
        }

        double wheel_rotation_per_turn = drive_gear_teeth/wheel_gear_teeth;

        double distance_per_turn = (Math.PI * 6) * wheel_rotation_per_turn;

        double total_wheel_turns = distance/distance_per_turn;

        double total_number_encoder_ticks = -ticks_per_revolution * total_wheel_turns;

        double new_power = new_power(Math.abs(robot.front_right.getCurrentPosition()/total_number_encoder_ticks));

        boolean correct_distance = robot.front_right.getCurrentPosition() <= -total_number_encoder_ticks;

        if(drive_straight_backward(new_power, 0, correct_distance))
        {
            drive_reset = false;
            return true;
        }
        return false;

    }

    public double new_power(double drive_proportion)
    {
        return Math.pow(Math.E, (-Math.pow(drive_proportion-0.5,2))/(2*Math.pow(.25, 2)));
    }
}
