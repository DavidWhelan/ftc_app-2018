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

package org.firstinspires.ftc.teamcode.Diva;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name="Basic: Iterative OpMode", group="Iterative Opmode")
@Disabled
public class Divaexample extends OpMode
{

    //THIS CODE IS HIGHLY EXPERIMENTAL. TESTING HAS NOT BEEN COMPLETED ON ANY OF THE CODE AND TECHHOGS ROBOTICS IS NOT LIABLE FOR ANY ACCIDENTS. HAVE A NICE DAY.

    //Please note do not use this specifically. Please use your own motor and gyro objects
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;

    ModernRoboticsI2cGyro gyro;
    //This is essential for your program. This must be defined at the top
    DivaDrive drive;

    int case_switch = 0;


    @Override
    public void init()
    {
        //I am just defining the names for stuf. This should reflect your own configuration
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");

        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");

        //This has to be done like this. the diva drive class must have access to the motors and gyro and this is the easiest way.
        //Please use your correct motors and gyro in place of these placeholders
        drive = new DivaDrive(frontLeft, backLeft, frontRight, backRight, gyro);
        //You Must calibrate your gyro
        gyro.calibrate();
    }

    @Override
    public void init_loop() {
    }


    @Override
    public void start()
    {
        //You must also 0 the gyro. Please note that the gyro values are based off of this 0.
        //A heading of 90 refers to 90 degrees from the direction you started. 0 always refers to the starting direction and -90 always refers to 90 degrees left from where you started.
        gyro.resetZAxisIntegrator();
    }


    @Override
    public void loop()
    {
        switch (case_switch)
        {
            case 0:
            {
                //Turn to the 90 degrees right from where the robot started the autonomous period
                if(drive.turn_to_heading(.33, 90, .5))
                {
                    case_switch++;
                }
                break;
            }
            case 1:
            { //Turn to the direction the robot started at in the beggning of autonomous
                if(drive.turn_to_heading(.33, 0, .5))
                {
                    case_switch++;
                }
                break;
            }
            case 2:
            {
                //Turn 90 degrees to the left from where the robot started at in the beggning of autonomous
                if(drive.turn_to_heading(.33, -90, .5))
                {
                    case_switch++;
                }
                break;
            }
        }
    }


    @Override
    public void stop() {
    }

}
