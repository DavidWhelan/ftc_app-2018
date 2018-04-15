package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by David on 10/22/2017.
 */

public class RobotHardware
{
    public DcMotor
            front_left = null,
            front_right = null,
            back_left = null,
            back_right = null,
            lift = null,
            left_roller = null,
            right_roller = null,
            arm = null;

    public DigitalChannel
            top_lift = null,
            bottom_lift = null,
            block_bottom;

    ModernRoboticsI2cGyro gyro = null;


    public ColorSensor color_left = null;

    public Servo left_arm = null;
    public Servo left_flicker = null;
    public CRServo wrist = null;
    public Servo bottom_block;


    public CRServo extension= null;
    public CRServo belt = null;
    public CRServo lift_belt = null;
    public CRServo lift_belt2 = null;

    public AnalogInput force = null;

    //public BNO055IMU tilt_sensor;

    HardwareMap hwmap;

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    VuforiaTrackable relicTemplate;

    VuforiaTrackables relicTrackables;

    public RobotHardware()
    {

    }

    public void init_hardware(HardwareMap ahwmap, boolean camera)
    {
        hwmap = ahwmap;
        //////////////////////////////////////////////////////////////////////

        color_left = hwmap.colorSensor.get("colorLeft");

        ///////////////////////////////////////////////////////////////////////

        left_arm = hwmap.servo.get("leftArm");
        left_flicker = hwmap.servo.get("leftFlicker");
        left_arm.setDirection(Servo.Direction.REVERSE);


        //////////////////////////////////////////////////////////////////////

        lift = hwmap.dcMotor.get("lift");

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        arm = hwmap.dcMotor.get("arm");


        //////////////////////////////////////////////////////////////////////

        front_left = hwmap.dcMotor.get("frontLeft");
        back_left = hwmap.dcMotor.get("backLeft");
        front_right = hwmap.dcMotor.get("frontRight");
        back_right = hwmap.dcMotor.get("backRight");

        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);

        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        bottom_block = hwmap.servo.get("sbb");

        wrist = hwmap.crservo.get("wrist");

        wrist.setPower(0);

        bottom_block.setPosition(0);

        ////////////////////////////////////////////////////////////////////////

        right_roller = hwmap.dcMotor.get("rightRoller");
        left_roller = hwmap.dcMotor.get("leftRoller");

        left_roller.setDirection(DcMotorSimple.Direction.REVERSE);

        ////////////////////////////////////////////////////////////////////////
        top_lift = hwmap.get(DigitalChannel.class, "topLift");
        bottom_lift = hwmap.get(DigitalChannel.class, "bottomLift");

        block_bottom = hwmap.get(DigitalChannel.class, "bb");

        top_lift.setMode(DigitalChannel.Mode.INPUT);
        bottom_lift.setMode(DigitalChannel.Mode.INPUT);

        block_bottom.setMode(DigitalChannel.Mode.INPUT);


        ////////////////////////////////////////////////////////////////////////

        gyro = hwmap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro.calibrate();
        force = hwmap.analogInput.get("force");

        //////////////////////////////////////////////////////////////////////////

        left_arm_up();
        left_flicker_left();

        belt = hwmap.crservo.get("belt");
        extension = hwmap.crservo.get("ext");
        lift_belt = hwmap.crservo.get("liftBelt");
        lift_belt2 = hwmap.crservo.get("liftBelt2");
        lift_belt2.setDirection(DcMotorSimple.Direction.REVERSE);

        lift_belt.setPower(0);
        belt.setPower(0);
        extension.setPower(0);

        //////////////////////////////////////////////////////////////////////////
        /*
        BNO055IMU.Parameters tlt_parameters = new BNO055IMU.Parameters();
        tlt_parameters.mode                = BNO055IMU.SensorMode.IMU;
        tlt_parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        tlt_parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        tlt_parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        tlt_parameters.loggingEnabled      = true;
        tlt_parameters.loggingTag          = "IMU";
        tlt_parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        tilt_sensor = hwmap.get(BNO055IMU.class, "tilt");

        tilt_sensor.initialize(tlt_parameters);

        tilt_sensor.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        */


        if(camera)
        {

            int cameraMonitorViewId = hwmap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwmap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

            parameters.vuforiaLicenseKey = "AWyc+zL/////AAAAGW3GVoQIxUxVm+/S9WJS8ZSF2/u9HEb/PlyEfbsNzIHuUSO/w/UUSMn5kvY37iVId9MTYY8+l2wxfVsmWhNrW02Q8k9LItE/t1Oi0GTPrFWUnKrOicNQIUdwYKJBvqGR60q8rmSpqMuMoGFzgwACY7sfPQrmoRX2el3zoRYNC6lPVY+8C3xhI6tN9iLfEAMJtSFzitdyhJZdtYV00KsO8OkG9cFP/FMwDSAsdHS/fdjGYqs3JlV/wurGNgaa+9QeK3S8Ll06ZKj6CBo1wCf7h1PSi7auC8wDjtumEj9qa0NEW3gk1J102hWwPoy/D0h1RJfWqJK1mHEweKTHABK3LF3W9ld/jUtGZNKuM2WZo2pK";

            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
            this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

            relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
            relicTemplate = relicTrackables.get(0);
            relicTemplate.setName("relicVuMarkTemplate");

            relicTrackables.activate();
        }
    }


    RelicRecoveryVuMark identify_position()
    {
        return RelicRecoveryVuMark.from(relicTemplate);
    }

    public void eject_block()
    {
        right_roller.setPower(-1);
        left_roller.setPower(-1);
    }

    public void gather_block()
    {
        right_roller.setPower(1);
        left_roller.setPower(1);
    }

    public void stop_ejector()
    {
        right_roller.setPower(0);
        left_roller.setPower(0);
    }
    ////////////////////////////////////////////////////////////////////////////
    public void left_flicker_left()
    {
        left_flicker.setPosition(0);
    }

    public void left_flicker_right()
    {
        left_flicker.setPosition(1);
    }

    public void left_flicker_center()
    {
        left_flicker.setPosition(.35);
    }
    //////////////////////////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////////////////////////////

    public void left_arm_up()
    {
        left_arm.setPosition(0);
    }

    public void left_arm_down()
    {
        left_arm.setPosition(0.85);
    }

    /////////////////////////////////////////////////////////////////////////////////



    /////////////////////////////////////////////////////////////////////////////////


    /////////////////////////////////////////////////////////////////////////////////
    public void read_left()
    {
        left_arm.setPosition(.85);
        left_flicker_center();
    }

    public void reset_left()
    {
        left_arm_up();
        left_flicker_left();
    }

    public void reset_z()
    {
        gyro.resetZAxisIntegrator();
    }

}
