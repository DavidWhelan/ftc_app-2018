

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
    RobotHardware robot = new RobotHardware();
    Drive drive = new Drive(robot);

    @Override
    public void init()
    {
        robot.init_hardware(hardwareMap, false);
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
        double right_X = gamepad1.right_stick_x;
        double right_Y = gamepad1.right_stick_y;

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

        //////////////////////////////////////////////////////////////

        if(gamepad2.right_bumper)
        {
            robot.claw.setPosition(1);
        }
        else if(gamepad2.left_bumper)
        {
            robot.claw.setPosition(0);
        }

        robot.tape_measure.setPower(gamepad2.left_stick_y * .75);
        robot.tape_adjustor.setPower(gamepad2.right_stick_y);

        telemetry.addData("FL", robot.front_left.getCurrentPosition());
        telemetry.addData("BL", robot.back_left.getCurrentPosition());
        telemetry.addData("FR", robot.front_right.getCurrentPosition());
        telemetry.addData("BR", robot.back_right.getCurrentPosition());

        telemetry.addData("Top Switch", robot.top_lift.getState());
        telemetry.addData("Bottom Switch", robot.bottom_lift.getState());

        telemetry.addData("Lift Encoder", robot.lift.getCurrentPosition());
    }


    @Override
    public void stop()
    {
        drive.stop();
    }

    public double new_power(double power)
    {
        if(power == 0)
        {

        }
        return Math.pow(power, 3) + (Math.abs(power)/power) * .1;
    }

}
