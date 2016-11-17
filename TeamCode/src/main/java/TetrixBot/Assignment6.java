
package TetrixBot;

import android.os.Debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * TetrixArm - Robot with 6-axis arm
 *
 * The name "TetrixArm" refers to the instructor's robotic arm built from Tetrix parts.
 * There is no difference in programming on either the Actobotics or Tetrix platform.
 *
 * @author Jochen Fischer
 * @author Colin Hiriak
 * @author Tanner McIntyre
 * @version 1.0, 10/31/2016
 *
 * -------------------------------------------------------------------------
 * Gamepad 1 controls:
 *
 * torso:    X button moves left (CCW), B button right (CW)
 * shoulder: left trigger and left bumper
 *
 * other:
 * reset arm encoders: Start button
 * back to zero position: Y button
 * -------------------------------------------------------------------------
 */
@TeleOp(name="Assignment6", group="Robotic Arms")
public class Assignment6 extends LinearOpMode {

    // define the robot hardware:
    HardwareTetrixBot robot = new HardwareTetrixBot();   // Use the Tetrix arm

    // keep track of the timing throughout the program:
    private ElapsedTime runtime = new ElapsedTime();

    boolean startButtonPressed = false;
    int shoulderFirst = -100;
    int shoulderSecond = -556;
    int elbowDrop = 2656;

    public void initializeRobot() {

        robot.motorShoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorShoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorShoulder.setPower(robot.POWER_SHOULDER/2);
        //pos shoulder power is UP

        while(!robot.sensorShoulder.isPressed()) {
                //robot.motorShoulder.setPower(-HardwareTetrixBot.POWER_SHOULDER / 2);
            }
            robot.motorShoulder.setPower(0);
            robot.resetShoulderEncoder();

            robot.motorElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.motorElbow.setPower(-robot.POWER_ELBOW / 2);
             //neg elbow power is UP

        while(!robot.sensorElbow.isPressed()) {
            //robot.motorElbow.setPower(HardwareTetrixBot.POWER_ELBOW / 2);
        }
            robot.motorElbow.setPower(0);
            robot.resetElbowEncoder();


        robot.motorShoulder.setTargetPosition(robot.INIT_SHOULDER);
        robot.motorElbow.setTargetPosition(robot.INIT_ELBOW);
        robot.motorShoulder.setPower(-robot.POWER_SHOULDER/2);
        robot.motorElbow.setPower(robot.POWER_ELBOW/2);
    }

    public void dropObject(){
        while(robot.posGripper != 0.56) {
            robot.motorShoulder.setTargetPosition(shoulderFirst);
            robot.motorShoulder.setPower(-robot.POWER_SHOULDER / 2);
            while( robot.motorShoulder.getCurrentPosition() != -100) {
            }
            robot.motorElbow.setTargetPosition(elbowDrop);
            robot.motorElbow.setPower(-robot.POWER_ELBOW / 2);
            while(robot.motorElbow.getCurrentPosition() != 2656){
            }

            robot.posTorso = 0.386;
            robot.servoTorso.setPosition(robot.posTorso);
            while(robot.servoTorso.getPosition() != .386) {
            }

            robot.motorShoulder.setTargetPosition(shoulderSecond);
            robot.motorShoulder.setPower(-robot.POWER_SHOULDER / 2);
            while(robot.motorShoulder.getCurrentPosition() != -556){
            }



            robot.posGripper = 0.56;
            robot.servoGripper.setPosition(robot.posGripper);
            robot.motorShoulder.setPower(0);
            robot.motorElbow.setPower(0);
            robot.servoTorso.setPosition(robot.INIT_TORSO);
        }
        robot.servoTorso.setPosition(robot.INIT_TORSO);
        robot.motorElbow.setTargetPosition(robot.INIT_ELBOW);
        robot.motorShoulder.setTargetPosition(robot.INIT_SHOULDER);
        robot.motorShoulder.setPower(-robot.POWER_SHOULDER/2);
        robot.motorElbow.setPower(robot.POWER_ELBOW/2);



    }

    @Override
    public void runOpMode() throws InterruptedException {

        // initialize the hardware
        robot.init(hardwareMap);
        initializeRobot();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //hit limit switch then reset encoder


        // Wait for the user to press the PLAY button on the DS:
        waitForStart();


        while (opModeIsActive()) {

            //------------------------------------------------------------------
            // reset shoulder and elbow encoders by pressing the Start button
            //------------------------------------------------------------------
            if (gamepad1.start && !startButtonPressed) {
                // start button state went from not pressed to pressed:
                startButtonPressed = true;
                robot.resetShoulderEncoder();
                robot.resetElbowEncoder();
                robot.posElbow = 0;
                robot.posShoulder = 0;
                robot.robotIsInitialized = true;
            } else {
                startButtonPressed = false;
            }

            //------------------------------------------------------------------
            // autonomous drop block in bucket when press 'A'
            //------------------------------------------------------------------
            if(gamepad1.a) {
              dropObject();
            }

            //----------------------------------------
            // move shoulder up and down
            //----------------------------------------
            int currentShoulderPosition = robot.motorShoulder.getCurrentPosition();

            if (gamepad1.left_trigger > 0.5) {
                // move shoulder down:
                robot.posShoulder = currentShoulderPosition
                        + HardwareTetrixBot.DELTA_SHOULDER;
            }
            if (gamepad1.left_bumper) {
                // move shoulder up:
                robot.posShoulder = currentShoulderPosition
                        - HardwareTetrixBot.DELTA_SHOULDER;
            }
            if (gamepad1.y) {
                robot.posShoulder = HardwareTetrixBot.INIT_SHOULDER;
                robot.posTorso = HardwareTetrixBot.INIT_TORSO;
            }

            robot.motorShoulder.setTargetPosition(robot.posShoulder);
            robot.motorShoulder.setPower(HardwareTetrixBot.POWER_SHOULDER);
            telemetry.addData("shoulder target", String.format("%d", robot.posShoulder));

            //----------------------------------------
            // move elbow left and right:
            //----------------------------------------

            int currentElbowPosition = robot.motorElbow.getCurrentPosition();

            if (gamepad1.right_bumper) {
                //move elbow down
                robot.posElbow = currentElbowPosition
                        + HardwareTetrixBot.DELTA_ELBOW;
            }
            if (gamepad1.right_trigger > 0.5) {
                //move elbow up
                robot.posElbow = currentElbowPosition
                        - HardwareTetrixBot.DELTA_ELBOW;
            }

            robot.motorElbow.setTargetPosition(robot.posElbow);
            robot.motorElbow.setPower(HardwareTetrixBot.POWER_ELBOW);
            telemetry.addData("Elbow target", String.format("%d", robot.posElbow));

            //----------------------------------------
            // move torso left and right:
            //----------------------------------------
            if (gamepad1.x) {
                // move left, CCW
                robot.posTorso = Range.clip(robot.posTorso + HardwareTetrixBot.DELTA_TORSO,
                        HardwareTetrixBot.MIN_TORSO, HardwareTetrixBot.MAX_TORSO);
            }
            if (gamepad1.b) {
                // move right, CC
                robot.posTorso = Range.clip(robot.posTorso - HardwareTetrixBot.DELTA_TORSO,
                        HardwareTetrixBot.MIN_TORSO, HardwareTetrixBot.MAX_TORSO);
            }
            robot.servoTorso.setPosition(robot.posTorso);
            telemetry.addData("torso", String.format("%.4f", robot.posTorso));

            //----------------------------------------
            // move tilt left and right:
            //----------------------------------------
            if (gamepad1.dpad_left) {
                //move left, CC
                robot.posTilt = Range.clip(robot.posTilt + HardwareTetrixBot.DELTA_TILT,
                        HardwareTetrixBot.MIN_TILT, HardwareTetrixBot.MAX_TILT);
            }
            if (gamepad1.dpad_right) {
                //move right CCW
                robot.posTilt = Range.clip(robot.posTilt - HardwareTetrixBot.DELTA_TILT,
                        HardwareTetrixBot.MIN_TILT, HardwareTetrixBot.MAX_TILT);
            }

            robot.servoTilt.setPosition(robot.posTilt);
            telemetry.addData("tilt", String.format("%.4f", robot.posTilt));

            //----------------------------------------
            // move turn up and down:
            //----------------------------------------
            if (gamepad1.dpad_up) {
                //move up, CC
                robot.posTurn = Range.clip(robot.posTurn + HardwareTetrixBot.DELTA_TURN,
                        HardwareTetrixBot.MIN_TURN, HardwareTetrixBot.MAX_TURN);
            }
            if (gamepad1.dpad_down) {
                //move down, CC
                robot.posTurn = Range.clip(robot.posTurn - HardwareTetrixBot.DELTA_TURN,
                        HardwareTetrixBot.MIN_TURN, HardwareTetrixBot.MAX_TURN);
            }
            robot.servoTurn.setPosition(robot.posTurn);
            telemetry.addData("turn", String.format("%.4f", robot.posTurn));

            //----------------------------------------
            // open and close gripper
            //----------------------------------------
            if (gamepad1.left_stick_x > 0) {
                //move open, CC
                robot.posGripper = Range.clip(robot.posGripper + HardwareTetrixBot.DELTA_GRIPPER,
                        HardwareTetrixBot.MIN_GRIPPER, HardwareTetrixBot.MAX_GRIPPER);
            }
            if (gamepad1.left_stick_x < 0) {
                //move close, CC
                robot.posGripper = Range.clip(robot.posGripper - HardwareTetrixBot.DELTA_GRIPPER,
                        HardwareTetrixBot.MIN_GRIPPER, HardwareTetrixBot.MAX_GRIPPER);
            }
            robot.servoGripper.setPosition(robot.posGripper);
            telemetry.addData("Gripper", String.format("%.4f", robot.posGripper));

            //----------------------------------------
            // stop motors if touch limit switch
            //----------------------------------------
            if(robot.sensorElbow.isPressed()) {
                robot.motorElbow.setPower(robot.STOP);
                robot.resetElbowEncoder();

            }
            if(robot.sensorShoulder.isPressed()) {
                robot.motorShoulder.setPower(robot.STOP);
                robot.resetShoulderEncoder();


            }


            //--------------------------------------------
            // housekeeping at the end of the main loop
            //--------------------------------------------
            // update the telemetry with all data from this loop:
            telemetry.update();

            // run the loop in 50ms increments (and give up the CPU for the rest of a cycle)
            robot.waitForTick(50);
        }
    }



    }

