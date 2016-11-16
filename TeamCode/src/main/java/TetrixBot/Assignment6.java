
package TetrixBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
@TeleOp(name="Tetrix Arm", group="Robotic Arms")
public class Assignment6 extends LinearOpMode {

    // define the robot hardware:
    HardwareTetrixBot robot = new HardwareTetrixBot();   // Use the Tetrix arm

    // keep track of the timing throughout the program:
    private ElapsedTime runtime = new ElapsedTime();

    boolean startButtonPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // initialize the hardware
        robot.init(hardwareMap);
        robot.initializeRobot();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //hit limit switch then reset encoder
       // stopMotors();

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

            //----------------------------------------
            // move shoulder up and down
            //----------------------------------------
            int currentShoulderPosition = robot.motorShoulder.getCurrentPosition();

            if (gamepad1.left_trigger > 0.5) {
                // move shoulder down:
                robot.posShoulder = currentShoulderPosition
                        + robot.DELTA_SHOULDER;
            }
            if (gamepad1.left_bumper) {
                // move shoulder up:
                robot.posShoulder = currentShoulderPosition
                        - robot.DELTA_SHOULDER;
            }
            if (gamepad1.y) {
                robot.posShoulder = 0;
                robot.posTorso = robot.INIT_TORSO;
            }

            robot.motorShoulder.setTargetPosition(robot.posShoulder);
            robot.motorShoulder.setPower(robot.POWER_SHOULDER);
            telemetry.addData("shoulder target", String.format("%d", robot.posShoulder));

            //----------------------------------------
            // move elbow left and right:
            //----------------------------------------

            int currentElbowPosition = robot.motorElbow.getCurrentPosition();

            if (gamepad1.right_bumper) {
                //move elbow down
                robot.posElbow = currentElbowPosition
                        + robot.DELTA_ELBOW;
            }
            if (gamepad1.right_trigger > 0.5) {
                //move elbow up
                robot.posElbow = currentElbowPosition
                        - robot.DELTA_ELBOW;
            }

            robot.motorElbow.setTargetPosition(robot.posElbow);
            robot.motorElbow.setPower(robot.POWER_ELBOW);
            telemetry.addData("Elbow target", String.format("%d", robot.posElbow));

            //----------------------------------------
            // move torso left and right:
            //----------------------------------------
            if (gamepad1.x) {
                // move left, CCW
                robot.posTorso = Range.clip(robot.posTorso + robot.DELTA_TORSO,
                        robot.MIN_TORSO, robot.MAX_TORSO);
            }
            if (gamepad1.b) {
                // move right, CC
                robot.posTorso = Range.clip(robot.posTorso - robot.DELTA_TORSO,
                        robot.MIN_TORSO, robot.MAX_TORSO);
            }
            robot.servoTorso.setPosition(robot.posTorso);
            telemetry.addData("torso", String.format("%.4f", robot.posTorso));

            //----------------------------------------
            // move tilt left and right:
            //----------------------------------------
            if (gamepad1.dpad_left) {
                //move left, CC
                robot.posTilt = Range.clip(robot.posTilt + robot.DELTA_TILT,
                        robot.MIN_TILT, robot.MAX_TILT);
            }
            if (gamepad1.dpad_right) {
                //move right CCW
                robot.posTilt = Range.clip(robot.posTilt - robot.DELTA_TILT,
                        robot.MIN_TILT, robot.MAX_TILT);
            }

            robot.servoTilt.setPosition(robot.posTilt);
            telemetry.addData("tilt", String.format("%.4f", robot.posTilt));

            //----------------------------------------
            // move turn up and down:
            //----------------------------------------
            if (gamepad1.dpad_up) {
                //move up, CC
                robot.posTurn = Range.clip(robot.posTurn + robot.DELTA_TURN,
                        robot.MIN_TURN, robot.MAX_TURN);
            }
            if (gamepad1.dpad_down) {
                //move down, CC
                robot.posTurn = Range.clip(robot.posTurn - robot.DELTA_TURN,
                        robot.MIN_TURN, robot.MAX_TURN);
            }
            robot.servoTurn.setPosition(robot.posTurn);
            telemetry.addData("turn", String.format("%.4f", robot.posTurn));

            //----------------------------------------
            // open and close gripper
            //----------------------------------------
            if (gamepad1.left_stick_x > 0) {
                //move open, CC
                robot.posGripper = Range.clip(robot.posGripper + robot.DELTA_GRIPPER,
                        robot.MIN_GRIPPER, robot.MAX_GRIPPER);
            }
            if (gamepad1.left_stick_x < 0) {
                //move close, CC
                robot.posGripper = Range.clip(robot.posGripper - robot.DELTA_GRIPPER,
                        robot.MIN_GRIPPER, robot.MAX_GRIPPER);
            }
            robot.servoGripper.setPosition(robot.posGripper);
            telemetry.addData("Gripper", String.format("%.4f", robot.posGripper));

            //----------------------------------------
            // stop motors it touch limit switch
            //----------------------------------------
            if(robot.sensorElbow.isPressed()) {
                robot.motorElbow.setPower(0);
            }
            if(robot.sensorShoulder.isPressed()) {
                robot.motorShoulder.setPower(0);
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
    public void stopMotors(String s){
        if(s.equals("sensorShoulder")){
            robot.motorShoulder.setPower(0);
        } else if (s.equals("sensorElbow")){
            robot.motorElbow.setPower(0);
        } else{
            robot.motorShoulder.setPower(0);
            robot.motorElbow.setPower(0);
        }

    }


}