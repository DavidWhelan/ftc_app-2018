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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

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

@Autonomous(name="BlueBackGoal", group="Blue")
//@Disabled
public class BlueBackGoal extends OpMode
{
    // Declare OpMode members.
    RobotHardware robot;
    Drive drive;
    ElapsedTime case_timer = new ElapsedTime();
    int case_switch = 0;

    double left_distance = 5;
    double center_distance = 10;
    double right_distance= 18.75;

    double distance_travel = 0;

    double red = 0;
    double blue= 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init()
    {
        robot = new RobotHardware();
        robot.init_hardware(hardwareMap, true );
        drive = new Drive(robot);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop()
    {
        telemetry.addData("Motor Power", robot.back_left.getPower());
        telemetry.addData("Boolean", drive.pid_reset);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start()
    {
        robot.reset_z();

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
                    robot.left_arm_down();
                    case_timer.reset();
                    case_switch++;
                }
                break;
            }

            case 2:
            {
                if(case_timer.time() > .5)
                {
                    drive.score("blue", red, blue);
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

            case 4:
            {
                if(robot.identify_position() != RelicRecoveryVuMark.UNKNOWN || case_timer.time() > 10)
                {
                    if(robot.identify_position() == RelicRecoveryVuMark.LEFT)
                    {
                        distance_travel = left_distance;
                    }
                    else if(robot.identify_position() == RelicRecoveryVuMark.CENTER)
                    {
                        distance_travel = center_distance;
                    }
                    else
                    {
                        distance_travel = right_distance;
                    }
                    case_switch++;
                }
                break;
            }

            case 5:
            {
                if(drive.distance_drive_forward(.33, 24))
                {
                    case_switch++;
                }
                break;
            }

            case 6:
            {
                if(drive.turn_to_heading(.33, 90, .5))
                {
                    case_switch++;
                }
                break;
            }

            case 7:
            {
                if(drive.distance_drive_forward(.33, distance_travel))
                {
                    case_switch++;
                }
                break;
            }

            case 8:
            {
                if(drive.turn_to_heading(.33, 0, .5))
                {
                    case_switch++;
                }
                break;
            }

            case 9:
            {
                if(drive.distance_drive_forward(.33, 5))
                {
                    case_timer.reset();
                    robot.eject_block();
                    case_switch++;
                }
                break;
            }

            case 10:
            {
                if(case_timer.time() > 1.5)
                {
                    case_switch++;
                }
                break;
            }

            case 11:
            {
                if(drive.distance_drive_backward(.33, 2))
                {
                    robot.stop_ejector();
                    case_switch++;
                }
                break;
            }
        }

        telemetry.addData("Setpoint", drive.drive_control.setpoint);
        telemetry.addData("Motor Power", robot.back_left.getPower());
        telemetry.addData("Time", drive.drive_control.timer.time());
        telemetry.addData("Encoder", robot.front_left.getCurrentPosition());
        telemetry.addData("Case", case_switch);
        telemetry.addData("Vumark", robot.identify_position());

        telemetry.addData("Lift Encoder", robot.lift.getCurrentPosition());

        telemetry.addData("Gyro", robot.gyro.getIntegratedZValue());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
