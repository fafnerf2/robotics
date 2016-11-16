package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *  @author Jochen Fischer
 *  @version 1.0
 *
 *  Hardware definition for the Elon Drivebot.
 *
 *  The robot has a simple drive base with 2 motors:
 *      motorLeft
 *      motorRight
 *
 */
public class HardwareDriveBot
{
    /* Public OpMode members. */

    //~~~~~~~~~~ motors ~~~~~~~~~~
    public DcMotor  motorLeft   = null;
    public DcMotor  motorRight  = null;

    // hardware specific constants:
    public static final int ENC_ROTATION = 1120;
    public static final double WHEEL_DIAMETER = 3.9;
    public static final double WHEEL_BASE = 10.5;

    // useful constants:
    public static final double STOP       =  0.0;
    public static final double SLOW_POWER =  0.3;
    public static final double STD_POWER  =  0.5;
    public static final double MAX_POWER  =  1.0;

    //~~~~~~~~~~ sensors ~~~~~~~~~~
    public ColorSensor sensorColor = null;
    public TouchSensor sensorTouch = null;
    public GyroSensor sensorGyro = null;
//    public TouchSensor sensorLegoTouch = null;           // note the same type TouchSensor
//    public LightSensor sensorLegoLight = null;
//    public UltrasonicSensor sensorUltrasonic = null;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareDriveBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motorLeft   = hwMap.dcMotor.get("motorLeft");
        motorRight  = hwMap.dcMotor.get("motorRight");
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        // robot uses encoders on the drive motors:
        resetEncoders();


        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Map and initialize the sensors:
        sensorColor = hwMap.colorSensor.get("sensorColor");
        sensorTouch = hwMap.touchSensor.get("sensorTouch");
        sensorGyro = hwMap.gyroSensor.get("sensorGyro");
//        sensorLegoTouch = hwMap.touchSensor.get("sensorLegoTouch");
//        sensorLegoLight = hwMap.lightSensor.get("sensorLegoLight");
//        sensorUltrasonic = hwMap.ultrasonicSensor.get("sensorUltrasonic");

        // turn the LED on, we want an active sensor:
        sensorColor.enableLed(true);

        // Lego light sensor is also active:
//        sensorLegoLight.enableLed(true);
    }

    /**
     * resetEncoders - stops the robot and resets the encoder for the left and right motor
     *
     * @author Jochen Fischer
     * @version 1.0 - 10/4/2016
     */
    public void resetEncoders() {
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * start, spin, stop - convenience functions to run, turn and stop the robot
     *
     * @author Jochen Fischer
     * @version 1.1 - 10/4/2016
     *
     * @param speed    motor power between -1.0...1.0
     */
    public void start(double speed) {
        motorLeft.setPower(speed);
        motorRight.setPower(speed);
    }

    public void spin(double speed) {
        motorLeft.setPower(-speed);   // left motor turns backward for CW turn
        motorRight.setPower(speed);   // right motor turns forward for CW turn
    }

    public void stop() {
        motorLeft.setPower(STOP);
        motorRight.setPower(STOP);
    }


    /**
     * convertInchesToTicks - convert a distance given in inches to motor encoder ticks
     *
     * @author Jochen Fischer
     * @version 1.0 - 10/1/2016
     *
     * @param inches  distance in inches
     * @return ticks  encoder ticks
     */
    public static int convertInchesToTicks(double inches) {

        // translate the distance in inches to encoder ticks:
        double wheelRotations = inches / (Math.PI * HardwareDriveBot.WHEEL_DIAMETER);
        int encoderTicks = (int)(wheelRotations * HardwareDriveBot.ENC_ROTATION);

        return encoderTicks;
    }

    /**
     * convertTicksToInches - convert number of encoder ticks to inches traveled by the robot
     *
     * @author Jochen Fischer
     * @version 1.0 - 10/1/2016
     *
     * @param encoderTicks  number of encoder ticks the robot has driven
     * @return inches       distance in inches
     */
    public static double convertTicksToInches(int encoderTicks) {
        double wheelRotations = (double) encoderTicks / HardwareDriveBot.ENC_ROTATION;
        double inches = wheelRotations * (Math.PI * HardwareDriveBot.WHEEL_DIAMETER);

        return inches;
    }

    /**
     * convertDegreesToTicks - convert turn angle to encoder ticks
     *
     * @author Jochen Fischer
     * @version 1.0 - 10/5/2016
     *
     * @param degrees  turn angle of the robot, positive values are clockwise
     * @return
     */
    public static int convertDegreesToTicks(double degrees) {
        double wheelRotations = (degrees / 360.0) * Math.PI * HardwareDriveBot.WHEEL_BASE
                / (Math.PI * HardwareDriveBot.WHEEL_DIAMETER);
        int encoderTarget = (int)(wheelRotations * HardwareDriveBot.ENC_ROTATION);

        return encoderTarget;
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

