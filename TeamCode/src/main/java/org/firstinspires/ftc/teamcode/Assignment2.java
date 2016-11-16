package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Assignment 2 - Move robot in polygon
 *
 * @author Jochen Fischer
 * @version 1.0, 9/18/2016
 *
 *
 */
@Autonomous(name="Assignment2", group="ElonDev")
public class Assignment2 extends LinearOpMode {

    // define the robot hardware:
    HardwareDriveBot robot   = new HardwareDriveBot();   // Use the Drivebot hardware

    @Override
    public void runOpMode() throws InterruptedException {

        // initialize the robot:
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the user to press the PLAY button on the DS:
        waitForStart();

        // move robot in polygon:
        double speed = HardwareDriveBot.SLOW_POWER;

        moveRobot(speed,  72.0);    // Go forward by 72” (6ft or 3 tiles)
        moveRobot(speed, -24.0);    // Go backward by 24”
        turnRobot(speed,  90.0);    // Turn left (counterclockwise) by 90°
        moveRobot(speed,  72.0);    // Go forward by 72”
        turnRobot(speed, 120.0);    // Turn left by 120°
        moveRobot(speed,  55.4);    // Go forward by 55.4”
        turnRobot(speed,  60.0);    // Turn left by 60°
        moveRobot(speed,  48.0);    // Go forward by 48”
        turnRobot(speed,-270.0);    // Turn right (clockwise) by 270°
    }

    /**
     * @author Jochen Fischer
     * @version 1.0 - 9/25/2016 - inital version, works only going forward
     * @version 1.1 - 9/26/2016 - added code to go in both directions
     * @version 2.0 - 10/4/2016 - extensive use of functions
     *
     * moveRobot - Moves the robot by a given distance and speed.
     *
     * @param speed    robot speed between -1.0 ... 1.0
     * @param inches   driving distance in inces
     * @throws InterruptedException
     */
    private void moveRobot(double speed, double inches) throws InterruptedException {

        // determine if the robot moves forward or backward:
        double direction = Math.signum(speed * inches);

        // sanity check: don't do anything if either speed or inches is zero
        if(direction == 0.0) return;

        // since we know in which direction the robot moves,
        // we can use absolute (=positive) values for both the encoder value and target

        // translate the distance in inches to encoder ticks:
        int encoderTarget = HardwareDriveBot.convertInchesToTicks( Math.abs(inches) );

        // move the desired distance:
        robot.resetEncoders();
        robot.start(Math.abs(speed) * direction);
        while (Math.abs(robot.motorLeft.getCurrentPosition()) < encoderTarget) {
            idle();
        }
        robot.stop();
    }

    /**
     * @author Jochen Fischer
     * @version 1.0 - 9/25/2016 - turns robot in both direcions
     * @version 2.0 - 10/4/2016 - extensive use of functions
     *
     * turnRobot - Turns the robot by a given angle in degrees.
     *
     * @param speed    robot speed between -1.0 ... 1.0
     * @param degrees  turn angle in degrees
     * @throws InterruptedException
     */
    private void turnRobot(double speed, double degrees) throws InterruptedException {

        // determine if the robot moves left (1.0) or right (-1.0):
        double direction = Math.signum(speed * degrees);

        // sanity check: don't do anything if either speed or inches is zero
        if(direction == 0.0) return;

        // since we know in which direction the robot moves,
        // we can use absolute (=positive) values for both the encoder value and target

        // translate the degrees into encoder ticks:
        int encoderTarget = HardwareDriveBot.convertDegreesToTicks( Math.abs(degrees) );

        // move the desired distance:
        robot.resetEncoders();
        robot.spin(Math.abs(speed) * direction);
        while (Math.abs(robot.motorLeft.getCurrentPosition()) < encoderTarget) {
            idle();
        }
        robot.stop();
    }
}
