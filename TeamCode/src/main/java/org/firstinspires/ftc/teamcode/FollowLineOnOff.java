
package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static android.R.attr.alpha;

/**
 * SensorLoop - reads several sensors and displays their status or values on the DS phone.
 *
 * @author Jochen Fischer
 * @version 1.0 - 9/18/2016
 * @version 1.1 - 10/4/2016 - added legacy sensors (light, touch, ultrasound) and gamepad control.
 *
 */
@TeleOp(name="Follow Line OnOff", group="ElonDev")
// @Disabled
public class FollowLineOnOff extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // define the robot hardware:
    HardwareDriveBot robot   = new HardwareDriveBot();   // Use the Drivebot hardware

    double onSpeed = 0.4;
    double offSpeed = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {

        // initialize the hardware
        // including the use of encoders
        robot.init(hardwareMap);

        // calibrate the gyroscope:
        telemetry.addData("Status", "calibrating gyroscope");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        int threshold = (5+47)/2;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Threshold", threshold);
        telemetry.update();

        Log.i("ROBOT", String.format("%d", threshold));

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            int brightness = robot.sensorColor.alpha();

            if(brightness > threshold) {
                //right turn
                robot.motorLeft.setPower(onSpeed);
                robot.motorRight.setPower(offSpeed);
            } else {
                robot.motorLeft.setPower(offSpeed);
                robot.motorRight.setPower(onSpeed);
            }

            // add telemetry data:
//            telemetry.addData("Color", String.format("red=%3d G=%3d B=%3d A=%3d", red, green, blue, alpha));
//            telemetry.addData("light", String.format("light=%.3f  rawLight=%.3f", light, rawLight));
//            telemetry.update();

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
