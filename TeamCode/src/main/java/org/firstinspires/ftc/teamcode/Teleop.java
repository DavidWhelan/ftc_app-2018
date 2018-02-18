

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name="Teleop", group="Tech Hogs")

public class Teleop extends OpMode
{


    boolean flag = false;

    boolean flag_servo = false;

    double pos=0.5;

    RobotHardware robot = new RobotHardware();
    Drive drive = new Drive(robot);



    @Override
    public void init()
    {
        robot.init_hardware(hardwareMap, false);
        drive.run_without_encoders();
    }


    @Override
    public void init_loop()
    {

    }


    @Override
    public void start()
    {

    }


    @Override
    public void loop()
    {
        double left_2_X = gamepad2.left_stick_x;
        double left_2_Y = gamepad2.left_stick_y;

        double left_X = gamepad1.left_stick_x;
        double left_Y = gamepad1.left_stick_y;

        /////////////////////////////////////////////////////////////
        if(left_X + left_Y > .05 || left_X + left_Y < -.05)
        {
            robot.front_left.setPower(new_power(-left_Y + left_X));
            robot.back_left.setPower(new_power(-left_Y + left_X));
            robot.front_right.setPower(new_power(-left_Y - left_X));
            robot.back_right.setPower(new_power(-left_Y - left_X));
        }

        else if(left_2_X + left_2_Y > .05 || left_2_X + left_2_Y < -.05)
        {
            robot.front_left.setPower(new_power_2(-left_2_Y + left_2_X));
            robot.back_left.setPower(new_power_2(-left_2_Y + left_2_X));
            robot.front_right.setPower(new_power_2(-left_2_Y - left_2_X));
            robot.back_right.setPower(new_power_2(-left_2_Y - left_2_X));
        }

        else
        {
            drive.stop();
        }

        /////////////////////////////////////////////////////////////
        double lift_power = -gamepad1.right_stick_y;

        if(lift_power <= -.01 && !robot.bottom_lift.getState())
        {
            robot.lift.setPower(lift_power);
        }
        else if(lift_power >= .01 && !robot.top_lift.getState())
        {
            robot.lift.setPower(lift_power);
        }
        else
        {
            robot.lift.setPower(0);
        }
        /////////////////////////////////////////////////////////////


        //////////////////////////////////////////////////////////////

        if(gamepad1.right_trigger > .01)
        {
            robot.right_roller.setPower(gamepad1.right_trigger);
        }
        else if(gamepad1.right_bumper)
        {
            robot.right_roller.setPower(-1);
        }
        else
        {
            robot.right_roller.setPower(0);
        }

        //////////////////////////////////////////////////////////////////

        if(gamepad1.left_trigger > .01)
        {
            robot.left_roller.setPower(gamepad1.left_trigger);
        }
        else if(gamepad1.left_bumper)
        {
            robot.left_roller.setPower(-1);
        }
        else
        {
            robot.left_roller.setPower(0);
        }

        if(gamepad2.dpad_up && !flag_servo)
        {
            flag_servo = true;
            pos += 0.02;
        }
        else if(gamepad2.dpad_down && !flag_servo)
        {
            flag_servo = true;
            pos -= 0.02;
        }
        robot.extension.setPosition(pos);
        if(!gamepad2.dpad_down && !gamepad2.dpad_up)
        {
            flag_servo = false;
        }

        if(gamepad2.a)
        {
            flag = true;
            //robot.belt.setPower(1);

        }
        else if(gamepad2.right_stick_y > 0.05 || gamepad2.right_stick_y < -0.05)
        {
            flag = false;
            robot.belt.setPower(-gamepad2.right_stick_y);
        }
        else if(flag == false)
        {
            robot.belt.setPower(0);
        }
        //////////////////////////////////////////////////////////////

        robot.arm.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

        if(flag)
        {
            robot.belt.setPower(-1+((robot.force.getVoltage()-0.5)/robot.force.getMaxVoltage()));
        }

        telemetry.addData("FL", robot.front_left.getCurrentPosition());
        telemetry.addData("BL", robot.back_left.getCurrentPosition());
        telemetry.addData("FR", robot.front_right.getCurrentPosition());
        telemetry.addData("BR", robot.back_right.getCurrentPosition());

        telemetry.addData("Top Switch", robot.top_lift.getState());
        telemetry.addData("Bottom Switch", robot.bottom_lift.getState());

        telemetry.addData("Lift Encoder", robot.lift.getCurrentPosition());

        telemetry.addData("Flag", flag);
        telemetry.addData("Force Sensor", robot.force.getVoltage());
    }


    @Override
    public void stop()
    {
        drive.stop();
    }

    public double new_power(double power)
    {
        return Math.pow(power, 3) + (Math.abs(power)/power) * .1;
    }

    public double new_power_2(double power)
    {
        return Math.pow(power, 11) + (Math.abs(power)/power) * .25;
    }

}
