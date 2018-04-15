/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Template", group="Template")
@Disabled
public class AutonomousTemplate extends OpMode
{
    // Declare OpMode members.
    RobotHardware robot;
    Drive drive;
    ElapsedTime case_timer = new ElapsedTime();
    int case_switch = 0;
    double last_heading = 0;
    double added_degrees = 0;

    double red = 0;
    double blue = 0;

    boolean pressed= false;

    String color = "blue";

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init()
    {
        robot = new RobotHardware();
        robot.init_hardware(hardwareMap, true);
        drive = new Drive(robot);
        robot.reset_z();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop()
    {
        telemetry.addData("Motor Power", robot.back_left.getPower());
        telemetry.addData("Boolean", drive.pid_reset);
        telemetry.addData("Gyro", robot.gyro.getIntegratedZValue());
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start()
    {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop()
    {
        switch(case_switch)
        {
            case 0:
            {
                robot.read_left();
                case_timer.reset();
                case_switch++;
                break;
            }

            case 1:
            {
                if(case_timer.time() > 1)
                {
                    red = robot.color_left.red();
                    blue = robot.color_left.blue();
                    if(gamepad1.b)
                    {
                        robot.left_arm_down();
                        case_timer.reset();
                        case_switch++;
                    }
                }
                break;
            }

            case 2:
            {
                if(case_timer.time() > .5)
                {
                    drive.score(color, red, blue);
                    case_timer.reset();
                    case_switch++;
                }
                break;
            }

            case 3:
            {
                if(case_timer.time() > .6)
                {
                    robot.left_flicker_center();
                }
                if(case_timer.time() > .8)
                {
                    robot.reset_left();
                    case_timer.reset();
                    case_switch++;
                }
                break;
            }
        }
        if(gamepad1.x && !pressed)
        {
            case_switch = 0;
            pressed = true;
        }
        else
        {
            pressed = false;
        }

        if(gamepad1.a)
        {
            color = "red";
        }
        if(gamepad1.y)
        {
            color = "blue";
        }
        telemetry.addLine("Press X to re-run, Press A to select red and press Y to select blue");
        telemetry.addLine("To continue press B");
        telemetry.addData("Red", robot.color_left.red());
        telemetry.addData("Blue", robot.color_left.blue());
        telemetry.addData("Green", robot.color_left.green());
        telemetry.addData("What Side Color", color);
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}