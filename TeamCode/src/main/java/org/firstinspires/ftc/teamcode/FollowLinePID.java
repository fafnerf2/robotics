package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * FollowLinePID - Line follower using a basic proportional controller
 *
 * @author Tanner McIntyre
 * @author Colin Hiriak
 * @version 1.0 - 10/10/2016 as shown in class
 *
 *
 */
@Autonomous(name="Follow Line PID", group="ElonDev")
// @Disabled
public class FollowLinePID extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // define the robot hardware:
    HardwareDriveBot robot   = new HardwareDriveBot();   // Use the Drivebot hardware

    // parameters used by the controller:
    // speed is negative because we flipped our color sensor and needed robot to go backwards
    private double speed = -0.2;
    private double reference;
    private double Kp = 0.004;

    private double maxWhite;
    private double minWhite;

    private double dt = 50.0;  // interval in millisconds
    private double dT = dt/1000.0;   // interval in seconds

    private double Pc = 0.909;

    private double Ki = 2*Kp*dT/Pc;

    private double Kd = Kp*Pc / (8*dT);

    private double prevError = 0;
    private double sumError = 0;
    private int loopCounter = 0;

    /**
     * @author Jochen Fischer
     * @author Colin Hiriak
     * @author Tanner McIntyre
     * @version 1.0 - 9/25/2016 - turns robot in both direcions
     * @version 2.0 - 10/4/2016 - extensive use of functions
     * @version 3.0 - 10/26/2016 - calibrates either Min or Max alpha values
     *
     * turnRobot - Turns the robot by a given angle in degrees.
     *
     * @param speed    robot speed between -1.0 ... 1.0
     * @param degrees  turn angle in degrees
     * @throws InterruptedException
     */
    private double turnRobotCalibration(double speed, double degrees, boolean max)
            throws InterruptedException {

        //creates white value w/ highest or lowest value depending on which direction the user
        //is calibrating for
        double white;
        if(max){
            white = Double.MIN_VALUE;
        } else{
            white = Double.MAX_VALUE;
        }

        // determine if the robot moves left (1.0) or right (-1.0):
        double direction = Math.signum(speed * degrees);

        // sanity check: don't do anything if either speed or inches is zero
        if(direction == 0.0) return 0;

        // since we know in which direction the robot moves,
        // we can use absolute (=positive) values for both the encoder value and target

        // translate the degrees into encoder ticks:
        int encoderTarget = HardwareDriveBot.convertDegreesToTicks( Math.abs(degrees) );

        // move the desired distance:
        // update min or max distance depending on supplied boolean
        robot.resetEncoders();
        robot.spin(Math.abs(speed) * direction);
        while (Math.abs(robot.motorLeft.getCurrentPosition()) < encoderTarget) {
            if(max && robot.sensorColor.alpha() > white){
                white = robot.sensorColor.alpha();
            } else if(!max && robot.sensorColor.alpha() < white){
                white = robot.sensorColor.alpha();
            }
            idle();
        }
        robot.stop();
        return white;
    }
        //sets minWhite and maxWhite
    private double CalibrateColorSensor() {
        try {
            maxWhite = turnRobotCalibration(.05, 20, true);
            minWhite = turnRobotCalibration(.05, -40, false);
            turnRobot(.05, 20);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        return (maxWhite + minWhite)/2;
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
    private void turnRobot(double speed, double degrees)
            throws InterruptedException {

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

    @Override
    public void runOpMode() throws InterruptedException {

        // initialize the hardware
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");




        // calibrate color
         reference = CalibrateColorSensor();
        telemetry.addData("Reference", reference);
        telemetry.addData("Reference", reference);
        telemetry.update();

        // print process parameters in console:
        //Log.i("ROBOT", String.format("ref = %d",reference));

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // read the current light sensor value:
            int brightness = robot.sensorColor.alpha();

            // implementation of P-controller:
            double error = reference - brightness;

            sumError = 0.8 * sumError + error;


            double dError = error - prevError;

            prevError = error;

            //double turn = Kp * error;

            double turn = Kp * error + Ki * sumError + Kd * dError;

            robot.motorLeft.setPower(speed - turn);
            robot.motorRight.setPower(speed + turn);

            String message = String.format("%s %d",
                    runtime.toString(), brightness);
            Log.i("ROBOT", message);

            // wait for the next time slot:
            loopCounter++;
            double nextTimeSlot = loopCounter * dt;
            while (runtime.milliseconds() < nextTimeSlot) {
                idle();
            }
        }
    }
}