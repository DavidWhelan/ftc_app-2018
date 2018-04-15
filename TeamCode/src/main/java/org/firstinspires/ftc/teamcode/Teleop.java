

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name="Teleop", group="Tech Hogs")

public class Teleop extends OpMode
{


    boolean flag = false;

    boolean balance_flag = false;

    boolean toggle = false;
    boolean second_toggle = false;

    boolean button_check = false;
    boolean button_check2= false;

    double inital = 0;

    double initial_x = 0;
    double initial_y = 0;

    RobotHardware robot = new RobotHardware();
    Drive drive = new Drive(robot);

    ElapsedTime test_timer = new ElapsedTime();

    PID arm_controller = new PID();

    @Override
    public void init()
    {

        robot.init_hardware(hardwareMap, false);
        drive.run_without_encoders();
        inital = robot.force.getVoltage();
        arm_controller.set_tunings(0.01, 0.0, 0);

        //initial_x = robot.tilt_sensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
        //initial_y = robot.tilt_sensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
    }


    @Override
    public void init_loop()
    {

    }


    @Override
    public void start()
    {
        arm_controller.setpoint=robot.arm.getCurrentPosition();
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
            balance_flag = false;
        }

        else if(!balance_flag)
        {
            robot.front_left.setPower(0);
            robot.front_right.setPower(0);
            robot.back_right.setPower(0);
            robot.back_left.setPower(0);
        }

            robot.wrist.setPower(left_2_Y);

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
        if(gamepad1.right_stick_button && ! button_check2 && !second_toggle)
        {
            button_check2 = true;
            second_toggle = true;
        }
        else if(gamepad1.right_stick_button && second_toggle && !button_check2)
        {
            button_check2 = true;
            second_toggle = false;
        }
        else if(!gamepad1.right_stick_button)
        {
            button_check2 = false;
        }
        //////////////////////////////////////////////////////////////

        if(gamepad1.right_trigger > .01)
        {
            robot.right_roller.setPower(gamepad1.right_trigger);
            toggle = false;
            //toggled on is true so tru tru = false false 0/0 = 1
        }
        else if(gamepad1.right_bumper)
        {
            robot.right_roller.setPower(-1);
        }
        else if(toggle)
        {
            robot.right_roller.setPower(1);
        }
        else
        {
            robot.right_roller.setPower(0);
        }

        //////////////////////////////////////////////////////////////////

        if(gamepad1.left_trigger > .01)
        {
            robot.left_roller.setPower(gamepad1.left_trigger);
            toggle = false;
        }
        else if(gamepad1.left_bumper)
        {
            robot.left_roller.setPower(-1);
        }
        else if(toggle)
        {
            robot.left_roller.setPower(1);
        }
        else
        {
            robot.left_roller.setPower(0);
        }
        /////////////////////////////////////////////////////////////////////////////
        if((toggle || (gamepad1.left_trigger > 0.01 && gamepad1.right_trigger > .01))&& ! second_toggle)
        {
            robot.lift_belt2.setPower(1);
            robot.lift_belt.setPower(1);
        }
        else
        {
            robot.lift_belt2.setPower(0);
            robot.lift_belt.setPower(0);
        }
///////////////////////////////////////////////////////////////////////////////////////
        if(gamepad2.dpad_up)
        {
            robot.left_arm.setPosition(0.2);
            robot.extension.setPower(-1);
        }
        else if(gamepad2.dpad_down)
        {
            robot.left_arm.setPosition(0.2);
            robot.extension.setPower(1);
        }
        else
        {
            robot.extension.setPower(-0.05);
        }

        if(gamepad1.left_stick_button && !button_check && !toggle)
        {
            button_check = true;
            toggle = true;
        }
        else if(gamepad1.left_stick_button && toggle && !button_check)
        {
            button_check = true;
            toggle = false;
        }
        else if(!gamepad1.left_stick_button)
        {
            button_check = false;
        }


        if(gamepad2.a)
        {
            flag = true;
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

        if(flag)
        {
            robot.belt.setPower(-scale_value(robot.force.getVoltage()));
        }
        //////////////////////////////////////////////////////////////


        if(gamepad2.right_trigger < .01 && gamepad2.left_trigger < .01)
        {
           double power_set = arm_controller.calculate(robot.arm.getCurrentPosition());
           robot.arm.setPower(power_set);
        }
        else
        {
            arm_controller.setpoint = robot.arm.getCurrentPosition();
            robot.arm.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
            robot.left_arm.setPosition(0.2);
        }
        //TODO set position for shutter
        if(robot.block_bottom.getState())
        {
            robot.bottom_block.setPosition(1);
        }
        else
        {
            robot.bottom_block.setPosition(0);
        }


        telemetry.addData("Force Sensor", robot.force.getVoltage());
        /*double y = robot.tilt_sensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle + (initial_y * -1);

        double x = robot.tilt_sensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle + (initial_x * -1);

        telemetry.addData("X", x);
        telemetry.addData("Y", y);
        telemetry.addData("Z", robot.gyro.getIntegratedZValue() * (Math.PI/180));
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


        double[] coordinate = Polar_Cartesian.polar(x, y);

        telemetry.addData("Radius", coordinate[0]);
        telemetry.addData("Theta", coordinate[1]);
        telemetry.addData("Balance Flag", balance_flag);

        telemetry.addData("Is Calibrated?", robot.tilt_sensor.isSystemCalibrated());
        telemetry.addData("Is Calibrated Gyro?", robot.tilt_sensor.isGyroCalibrated());
        telemetry.addData("Is Calibrated? Accel", robot.tilt_sensor.isAccelerometerCalibrated());

        if(gamepad1.right_stick_button)
        {
            balance_flag = true;
        }

        if(balance_flag)
        {
            if(drive.balance(.5, x, y))
            {
                balance_flag = false;
            }

        }
        else
        {   //TODO Reset the heading
            robot.gyro.resetZAxisIntegrator();
        }
        */



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
        return Math.pow(power, 11) + (Math.abs(power)/power) * .1;
    }

    public double scale_value(double value)
    {
        double scaled = ((1-0)*(robot.force.getVoltage() - inital))/(robot.force.getMaxVoltage()-inital) + inital;
        return scaled;
    }

}
