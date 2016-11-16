
package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Assignment 3 - counts lines as driven over by
 * robot and displays distance and lines on phone and console
 *
 * @author Tanner McIntyre
 * @author  Colin Hiriak
 *
 */
@TeleOp(name="Assignment 3", group="Team1")
// @Disabled
public class Assignment3 extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private boolean pressed = false;

    // define the robot hardware:
    HardwareDriveBot robot   = new HardwareDriveBot();   // Use the Drivebot hardware


    private double power = 0.1;

    /**
     * @author Jochen Fischer
     * moves robot at desired speed and distance in inches
     * @param speed
     * @param inches
     */
    private void moveRobot(double speed, double inches) throws InterruptedException {

        // determine if the robot moves forward or backward:
        double direction = Math.signum(speed * inches);

        // sanity check: don't do anything if either speed or inches is zero
        if(direction == 0.0) return;

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
     * @author Tanner McIntyre
     * @author  Colin Hiriak
     * counts white lines and measures distance
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        //create variables to be used later
        int pos = 0;
        int brightness = 0;
        int lines = 0;

        //min and max white values set in code
        int maxWhite = Integer.MIN_VALUE;
        int minWhite = Integer.MAX_VALUE;

        // initialize the hardware
        // including the use of encoders
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // reset encoders
        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set all motors to run with encoders.
        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set motor power
        robot.motorLeft.setPower(power);
        robot.motorRight.setPower(power);

        // run until button is pressed
        while (pressed == false) {


            //ends the loop first if the robot hits a wall
            pressed = robot.sensorTouch.isPressed();

            //gets ticks as robot moves
            pos = robot.motorRight.getCurrentPosition();
            double inches = robot.convertTicksToInches(pos);

            // read the color sensor
            int white = robot.sensorColor.alpha();

            //calibration for first 12 inches
            if(inches < 12){
                if(white > maxWhite){
                    maxWhite = white;
                    Log.i ("MAX", "New maxWhite value = " + maxWhite);
                }
                if(white < minWhite){
                    minWhite = white;
                    Log.i ("MIN", "New minWhite value = " + minWhite);
                }

            } else {

                //counts a line if brightness close to calibrated white is detected
                if (brightness == minWhite && white > maxWhite - 10) {
                    brightness = white;
                    lines++;
                }

                //resets the current brightness so that a new line can be counted
                if (white < minWhite + 5) {
                    brightness = minWhite;
                }

                // adds line count after 12 second calibration period
                telemetry.addData("Lines", "" + lines);
            }
                // adds distance to telemetry + update
                telemetry.addData("Distance", "" + inches);
                telemetry.update();


            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop

            }
        // final telemetry data when button hits the wall
            double inches = robot.convertTicksToInches(pos);
            telemetry.addData("Distance", "" + inches);
            telemetry.addData("Lines", "" + lines);
            telemetry.update();

            Log.i("DISTANCE", "Final Distance = " + inches + " inches");

        //reverses the robot back to it's starting position
            moveRobot(robot.SLOW_POWER, -inches);
        }
    }

