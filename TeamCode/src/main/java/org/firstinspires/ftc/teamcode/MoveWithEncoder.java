/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Move2Sec - moves the robot for 2 seconds
 *
 * @autor Jochen Fischer
 */

@Autonomous(name="Move With Encoders", group="Elon")
// @Disabled
public class MoveWithEncoder extends LinearOpMode {

    // define the robot hardware
    HardwareDriveBot robot = new HardwareDriveBot();

    @Override
    public void runOpMode() throws InterruptedException {

        // initialize the robot
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Encoder", robot.motorLeft.getCurrentPosition());
        telemetry.update();

        sleep(2000);  // wait to read the display

        moveRobot(0.3, 24.0);

        sleep(2000);

        //------------------------------------
        // turn robot
        // start both motors

        // reset encoders
        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set all motors to run with encoders.
        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorLeft.setPower(-HardwareDriveBot.SLOW_POWER);
        robot.motorRight.setPower(HardwareDriveBot.SLOW_POWER);

        int encTarget = 1500;

        // wait until we reach our target position
        int pos;
        do {
            pos = robot.motorRight.getCurrentPosition();
            telemetry.addData("Encoder", pos);
            telemetry.update();
        }
        while (pos < encTarget);

        // stop the robot:
        robot.stop();

        telemetry.addData("Encoder", robot.motorRight.getCurrentPosition());
        telemetry.update();

        sleep(4000);
    }

    /**
     * moveRobot - drives robot forward by a given distance at a given speed
     *
     * @author Jochen Fischer
     * @version 1.0 - 9/28/2016
     *
     * @param speed     speed of robot between -1.0 ... 1.0
     * @param inches    distance the robot will travel
     * @throws InterruptedException
     */
    void moveRobot (double speed, double inches) throws InterruptedException {
        // reset encoders
        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set all motors to run with encoders.
        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double rotations = inches / (Math.PI * HardwareDriveBot.WHEEL_DIAMETER);
        int encTarget = (int) (rotations * HardwareDriveBot.ENC_ROTATION);

        robot.motorLeft.setPower(speed);
        robot.motorRight.setPower(speed);

        // wait until we reach our target position
        while (robot.motorLeft.getCurrentPosition() < encTarget) {
            idle();
        }

        /*---- alternate version, as reference -------
        int pos;
        do {
            pos = robot.motorLeft.getCurrentPosition();
            telemetry.addData("Encoder", pos);
            telemetry.update();
        }
        while (pos < encTarget);
        --------------------------------------------*/

        // stop the robot:
        robot.stop();

        System.out.println("Encoder" + robot.motorLeft.getCurrentPosition());
        Log.i("ROBOT", "Encoder" + robot.motorLeft.getCurrentPosition());
    }

}
