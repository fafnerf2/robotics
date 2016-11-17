package TetrixBot;

import android.text.method.Touch;

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
 *  @author Colin Hiriak
 *  @author Tanner McIntyre
 *  @version 1.0 - 10/31/2016
 *
 *  Hardware definition for the Tetrix arm
 */
public class HardwareTetrixBot
{
    /* Public OpMode members. */

    //--------------------------------------
    // motors
    //--------------------------------------
    public DcMotor  motorShoulder   = null;
    public DcMotor motorElbow = null;

    final static int DELTA_SHOULDER = 40;
    final static int MIN_SHOULDER   = 0;
    final static int MAX_SHOULDER   = 0;
    final static int INIT_SHOULDER  = -242;
    int posShoulder = INIT_SHOULDER;
    final static double POWER_SHOULDER = 0.2;

    final static int DELTA_ELBOW = 40;
    final static int MIN_ELBOW   = 0;
    final static int MAX_ELBOW   = 0;
    final static int INIT_ELBOW  = 4111;
    int posElbow = INIT_ELBOW;
    final static double POWER_ELBOW = 0.2;

    // hardware specific constants:
    public static final int ENC_ROTATION_40 = 1120;
    public static final int ENC_ROTATION_60 = 1680;     // 1120 * 60 / 40

    // useful constants:
    public static final double STOP       =  0.0;

    //--------------------------------------
    // servos
    //--------------------------------------
    Servo servoTorso;

    final static double DELTA_TORSO = 0.01;
    final static double MIN_TORSO   = 0.170;
    final static double MAX_TORSO   = 0.562;
    final static double INIT_TORSO  = 0.486;
    double posTorso = INIT_TORSO;

    Servo servoTilt;

    // TODO: add code for the servo that moves the hand up and down
    final static double DELTA_TILT = 0.01;
    final static double MIN_TILT = -0.562;
    final static double MAX_TILT = 0.562;
    final static double INIT_TILT = -0.114;
    double posTilt = INIT_TILT;

    Servo servoTurn;

    // TODO: add code for the servo that turns the hand

    final static double DELTA_TURN = 0.01;
    final static double MIN_TURN    = 0.05;
    final static double MAX_TURN    = 0.4100;
    final static double INIT_TURN   = 0.050;
    double posTurn = INIT_TURN;

    Servo servoGripper;

    // TODO: add code for the gripper servo
    final static double DELTA_GRIPPER = 0.01;
    final static double MIN_GRIPPER   = 0;
    final static double MAX_GRIPPER    = 0.562;
    final static double INIT_GRIPPER   = 0.3760;
    double posGripper = INIT_GRIPPER;

    //touch sensor elbow and shoulder
    public TouchSensor sensorShoulder = null;
    public TouchSensor sensorElbow = null;




    //--------------------------------------
    // local OpMode members
    //--------------------------------------
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    boolean robotIsInitialized = false;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //--------------------------------------
        // Define and Initialize Motors
        //--------------------------------------
        motorShoulder = hwMap.dcMotor.get("motorShoulder");
        motorShoulder.setDirection(DcMotor.Direction.REVERSE);
        resetShoulderEncoder();

        // TODO: define the motor for the elbow here
        motorElbow = hwMap.dcMotor.get("motorElbow");
        motorElbow.setDirection(DcMotor.Direction.REVERSE);
        resetElbowEncoder();

        //--------------------------------------
        // Define and initialize servos:
        //--------------------------------------
        servoTorso = hwMap.servo.get("servoTorso");
        servoTorso.setPosition(posTorso);

        // TODO: initialize the tilt, turn and gripper servo here
        servoTilt = hwMap.servo.get("servoTilt");
        servoTilt.setPosition(posTilt);

        servoTurn = hwMap.servo.get("servoTurn");
        servoTurn.setPosition(posTurn);

        servoGripper = hwMap.servo.get("servoGripper");
        servoGripper.setPosition(posGripper);

        //--------------------------------------
        // Define and initialize sensors:
        //--------------------------------------
        sensorShoulder = hwMap.touchSensor.get("sensorShoulder");
        sensorElbow = hwMap.touchSensor.get("sensorElbow");
    }

    /**
     * resetShoulderEncoder - stops and resets the shoulder encoder
     *
     * @author Jochen Fischer
     * @version 1.0 - 10/31/2016
     */
    public void resetShoulderEncoder() {
        motorShoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetElbowEncoder() {
        motorElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
