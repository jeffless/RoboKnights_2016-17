/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.content.pm.ActivityInfo;
import android.graphics.Color;
import android.media.MediaPlayer;
import android.view.View;
import android.view.ViewGroup;

//import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.util.*;

import com.kauailabs.navx.ftc.AHRS;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.Mat;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.concurrent.RunnableFuture;
//hello!

//TRY TO ADD METHODS FOR STRAFING DIAGONALLY (USING ONLY TWO WHEELS)

//Currently using FTC SDK 2.2

public abstract class OpMode_5220 extends LinearOpMode implements CameraBridgeViewBase.CvCameraViewListener2
{
    //CONSTANTS:

    protected static final int HAS_NOT_STARTED = 0;
    protected static final int SETUP = 1;
    protected static final int INIT = 2;
    protected static final int WAITING = 3;
    protected static final int RUNNING = 4;

    protected static final boolean BLUE = true;
    protected static final boolean RED = false;
    protected static final boolean RIGHT = true;
    protected static final boolean LEFT = false;
    protected static final boolean UP = true;
    protected static final boolean DOWN = false;
    protected static final boolean NEAR = true;
    protected static final boolean FAR = false;
    protected static final boolean FORWARDS = true;
    protected static final boolean BACKWARDS = false;

    protected static enum ProgramType {UNDECIDED, AUTONOMOUS, TELEOP};
    protected static ProgramType programType = ProgramType.UNDECIDED;

    protected static final int TELEOP_TIME_LIMIT = 1200000; //currently 20 minutes, more than enough for any single run.
    protected static final int AUTONOMOUS_TIME_LIMIT = 29000;

    protected static final double NORMAL = 2;
    protected static final double ENCODER = 3;
    protected static final double GYRO = 4;
    protected static final double SLOW = 5;

    protected static final double DETECT_NONE = 0;
    protected static final double DETECT_RED = 1;
    protected static final double DETECT_BLUE = 2;

    protected static final double WHEEL_DIAMETER = 4.0; //in inches
    protected static final double GEAR_RATIO = (1.5);
    protected static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    protected static final int ENCODER_COUNTS_PER_ROTATION = 1120; //WAS 1440

    protected static double temp = 0;
    protected static double right = 0;
    protected static double theta = 0;

    //CONFIGURABLE CONSTANTS:

    protected static final boolean TIMER_ON = false; //ALWAYS KEEP THIS OFF UNTIL/UNLESS WE DECIDE TO USE IT AND FIX THE CODE FOR USING THE IN-CODE TIME LIMIT (in runConditions)
    protected static final int TIMER_STOP_BUFFER = 500; //in millis

    protected static final double DEFAULT_DRIVE_POWER = 1.0;
    protected static final double DEFAULT_SYNC_POWER = 0.56;
    protected static final double DEFAULT_TURN_POWER = 0.30;
    protected static final double DEFAULT_TURN_POWER_HIGH = 1.0;
    protected static final double INIT_SERVO_POSITION = 0.5;

    protected static final double ENCODER_SYNC_PROPORTIONALITY_CONSTANT = 0.001; //0.001 means 50 encoder counts --> 5% power difference
    protected static final double GYRO_SYNC_PROPORTIONALITY_CONSTANT = 0.14; //this times 100 is the motor power difference per degree off.
    protected static final double GYRO_SYNC_DIFFERENTIAL_CONSTANT = 0.0;
    protected static final double GYRO_SYNC_INTEGRAL_CONSTANT = 0;
    protected static final double ENCODER_SYNC_UPDATE_TIME = 20; //in milliseconds for convenience
    protected static final double GYRO_SYNC_UPDATE_TIME = 32; //in milliseconds for convenience

    protected static final double ROTATE_IMU_UPDATE_TIME = 32; //in milliseconds for convenience
    protected static final double ROTATE_IMU_PROPORTIONALITY_CONSTANT = 32;
    protected static final double ROTATE_IMU_DIFFERENTIAL_CONSTANT = 0;


    protected static final double CLIMBER_FLING_TIME = 1.0;
    protected static final double HOOK_ADJUST_RELEASE_TIME = 2.0;
    protected static final double HOOK_ADJUST_RELEASE_DISTANCE = 0.7;

    protected static final double LINE_WHITE_THRESHOLD = 28;

    protected static final double DOOR_OPEN = 0.5;
    protected static final double DOOR_INIT = 0.8;
    protected static final double DOOR_CLOSED = 1.0;

    protected static final double RP_IN = 0.0;
    protected static final double RP_RELEASE = 0.1;
    protected static final double RP_OUT = 0.4;

    protected static final double HOOK_IN = 0.92;
    protected static final double HOOK_RELEASE = 0.4;

    protected static final double CLAMP_IN = 1.0;
    protected static final double CLAMP_DOWN = 0.93;

    protected static final double TARGET_VOLTAGE = 12.5;
    protected static final double TARGET_SPEED = 7.5e-7;

    protected static final double kP = 0.18;
    protected static final double kI = 0.0;
    protected static final double kD = 0.0;

    protected static final double ST_1 = 0.0;
    protected static final double ST_2 = 0.1;

    //MOTORS AND SERVOS:

    protected static final String[] motorNames = {}; //Fill this in later.

    protected DeviceInterfaceModule cdim;
    protected DcMotor leftFrontMotor;
    protected DcMotor rightFrontMotor;
    protected DcMotor leftBackMotor;
    protected DcMotor rightBackMotor;
    protected DcMotor sweeperMotor;
    protected DcMotor liftMotor;
    protected DcMotor flywheelLeft;
    protected DcMotor flywheelRight;

    protected DcMotor[] driveMotors = new DcMotor[4];
    protected int[] driveMotorInitValues = new int[4];

    protected Servo doorServo;
    protected Servo autoExtendServo;
    protected Servo clampServo;
    protected Servo hookServo;

    //SENSORS:

    public static final int NAVX_DIM_I2C_PORT = 5;

    protected AHRS navX;
    protected ColorSensor colorSensorFront;
    protected ColorSensor colorSensorDown;
    protected GyroSensor gyroSensor;
    protected TouchSensor touchSensor1;
    protected TouchSensor touchSensor2;
    protected TouchSensor touchSensorFront;

    //OTHER GLOBAL VARIABLES:

    //protected FtcRobotControllerActivity ftcRCA;
    protected boolean programFinished = false; //allows manual termination of the program in an orderly fashion, especially for autonomous
    protected boolean debugLoopOn = false;
    protected Stopwatch gameTimer;
    protected int phase = HAS_NOT_STARTED;

    protected VuforiaLocalizer vuforia;
    protected VuforiaTrackables beacons;

    protected MediaPlayer mediaPlayer;
    public static final boolean MUSIC_ON = true;

    protected static double voltage = 0;
    protected static double speed = 0;
    protected static double integral = 0;
    protected static double lastError;
    protected static double lastTime;
    protected static double lastEncoder;

    protected boolean newImage = false;
    private static JavaCameraView openCVView = null;
    private static int width;
    private static int height;
    private static boolean initialized = false;


    public void setup()//this and the declarations above are the equivalent of the pragmas in RobotC
    {
        phase = SETUP;

        //ftcRCA = FtcRobotControllerActivity.ftcRCA;

        hardwareMap.logDevices();

        cdim = hardwareMap.deviceInterfaceModule.get("Device Interface Module 3");
        waitFullCycle();
        navX = AHRS.getInstance(cdim, NAVX_DIM_I2C_PORT, AHRS.DeviceDataType.kProcessedData);

        leftFrontMotor = hardwareMap.dcMotor.get("lf");
        rightFrontMotor = hardwareMap.dcMotor.get("rf");
        leftBackMotor = hardwareMap.dcMotor.get("lb");
        rightBackMotor = hardwareMap.dcMotor.get("rb");

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        //leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        driveMotors[0] = leftFrontMotor;
        driveMotors[1] = rightFrontMotor;
        driveMotors[2] = leftBackMotor;
        driveMotors[3] = rightBackMotor;

        flywheelLeft = hardwareMap.dcMotor.get("flywheel1");
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelRight = hardwareMap.dcMotor.get("flywheel2");
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sweeperMotor = hardwareMap.dcMotor.get("sweeper1");
        sweeperMotor.setDirection(DcMotor.Direction.REVERSE);

        liftMotor = hardwareMap.dcMotor.get("lift");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);

        doorServo = hardwareMap.servo.get ("dServo");
        autoExtendServo = hardwareMap.servo.get("rpServo");
        hookServo = hardwareMap.servo.get("hServo");
        clampServo = hardwareMap.servo.get("cServo");

        colorSensorDown = hardwareMap.colorSensor.get("cSensorD");
        colorSensorFront = hardwareMap.colorSensor.get("cSensorF");
        colorSensorDown.setI2cAddress(I2cAddr.create8bit(0x3E));//IF 7 BIT DOESN'T WORK TRY 8 BIT ADDRESS (I2cAddr.create8bit(0x3E)), OR USING I2CADDR CONSTRUCTOR
        // in hex, 0x3e = 62. deault address is 60 (reserved for colorSensorDown)
        colorSensorFront.enableLed(false);
        colorSensorDown.enableLed(true);
        //gyroSensor = hardwareMap.gyroSensor.get("gSensor");
        touchSensorFront = hardwareMap.touchSensor.get("tSensor");
    }

    public void initialize()
    {
        moveRackAndPinion(RP_IN);
        moveHook(HOOK_IN);
        moveDoor(DOOR_INIT);
        moveBallClamp(CLAMP_IN);

        waitFullCycle();
/*
        gyroSensor.calibrate();
        while (runConditions() && gyroSensor.isCalibrating())
        {

        }
        waitFullCycle();
        gyroSensor.resetZAxisIntegrator();
        waitFullCycle();
*/
        navX.zeroYaw();

        phase = INIT;

        writeToLog ("Down: " + colorSensorDown.getI2cAddress());
        writeToLog("Front: " + colorSensorFront.getI2cAddress());
    }

    public void waitForStart ()
    {
        phase = WAITING;
        super.waitForStart();
    }

    public abstract void main(); //implement in all subclasses. This is the main body of the program. Maybe also make initializeRobot something to override if its different between OpModes.

    public final void runOpMode() throws InterruptedException
    {
        setup();
        initialize();

        telemetry.addData("1", "Ready to run.");
        telemetry.update();

        waitForStart();

        phase = RUNNING;
        gameTimer = new Stopwatch();

        main();
        end();
    }

    public void end()
    {
       stopDrivetrain();
    }

    //HELPER CLASSES AND METHODS:
    //______________________________________________________________________________________________________________

    public final boolean xor (boolean x, boolean y)
    {
        return ((x || y) && !(x && y));//make sure this works
    }

    public class Stopwatch
    {
        private final long start;

        public Stopwatch() {
            start = System.currentTimeMillis();
        }

        public int time()
        {
            long now = System.currentTimeMillis();
            return ((int) (now - start));
        }

        public double timeSeconds()
        {
            long now = System.currentTimeMillis();
            return (((double) (now - start)) / 1000);
        }
    }

    public class DebuggerDisplayLoop extends Thread
    {
        public void run()
        {
            DecimalFormat df = new DecimalFormat("#.##");
            String yaw;
            String pitch;
            String roll;
            String fh;
            String yprf;
            debugLoopOn = true;
            while (debugLoopOn && opModeIsActive())
            {

                yaw = df.format(navX.getYaw());
                pitch = df.format(navX.getPitch());
                roll = df.format(navX.getRoll());
                fh = df.format(navX.getFusedHeading());
                yprf = yaw + ", " + pitch + ", " + roll + ", " + fh;

                telemetry.addData("1", "Time Elapsed:" + gameTimer.time());

                telemetry.addData("2", "LFM: " + leftFrontMotor.getCurrentPosition() + ", RFM: " + rightFrontMotor.getCurrentPosition());
                telemetry.addData("3", "LBM: " + leftBackMotor.getCurrentPosition() + ", RBM: " + rightBackMotor.getCurrentPosition());

                telemetry.addData("5", "Down: R = " + colorSensorDown.red() + ", G = " + colorSensorDown.green() + ", B = " + colorSensorDown.blue() + ", A = " +  colorSensorDown.alpha());
                telemetry.addData("6", "Front: R = " + colorSensorFront.red() + ", G = " + colorSensorFront.green() + ", B = " + colorSensorFront.blue() + ", A = " +  colorSensorFront.alpha());
                telemetry.addData ("7", "Y,P,R,FH: " + yprf);

                telemetry.addData("8", "Voltage " + voltage);

                //waitOneFullHardwareCycle();
                telemetry.update();
            }
        }
    }

    public void writeToLog (String toWrite)
    {
        if (!runConditions()) return;
        String text = "USER MESSAGE: ";
        for (int i = 0; i < 54; i++) text = text + "*";
        for (int i = 0; i < 54; i++) text = text + " ";
        text = text + " " + toWrite + " ";
        for (int i = 0; i < 54; i++) text = text + " ";
        for (int i = 0; i < 54; i++) text = text + "_";
        //DbgLog.error(text);
        DbgLog.msg(text);
        DbgLog.msg("USER MESSAGE (short): " + toWrite);
        DbgLog.error("USER MESSAGE (short): " + toWrite);
    }

    public ProgramType getProgramType () //override in any meaningful subclass
    {
        return ProgramType.UNDECIDED;
    }

    public final boolean runConditions()
    {
        int maxTime;

        if (getProgramType() == ProgramType.AUTONOMOUS)
        {
            //maxTime = 120000 - TIMER_STOP_BUFFER;
            maxTime = AUTONOMOUS_TIME_LIMIT;
        }

        else if (getProgramType() == ProgramType.TELEOP)
        {
            return (opModeIsActive());
            //maxTime = 30000 - TIMER_STOP_BUFFER;
        }

        boolean timeValid = (!TIMER_ON || (gameTimer.time() < maxTime));
        return (opModeIsActive() && (!programFinished) && timeValid);
    }

    public int distanceToEncoderCount (double distance) //distance is in inches. MAKE SURE HAVING MECANUM WHEELS DOES NOT AFFECT THE ACCURACY OF DOING THIS CONVERSION BY PURE MATH
    {
        double wheelRotations = distance / WHEEL_CIRCUMFERENCE;
        double motorRotations = wheelRotations / GEAR_RATIO;
        long encoderCounts = Math.round(motorRotations * ENCODER_COUNTS_PER_ROTATION);
        return (int) encoderCounts;
    }

    public int distanceToStrafeEncoderCount (double distance) //THIS CANNOT BE DONE BY MATH. THIS CONVERSION FACTOR MUST BE DETERMINED EMPIRICALLY.
    {
        return ((int) (distance * ((double)ENCODER_COUNTS_PER_ROTATION / 13.7))); //TEMPORARY STAND-IN UNTIL WE ACTUALLY FIGURE OUT THE CONVERSION RATIO. This guesstimate assumes that one rotation of the wheels moves the robot 4.5 inches sideways.
    }

    public void sleep(int millis) //change back to old way if the new way doesn't work
    {
        int startTime = gameTimer.time();
        while (runConditions() && (gameTimer.time() < startTime + millis))
        {
            //waitFullCycle();
        }
        return;
    }

    public final void waitFullCycle ()
    {
        if (!runConditions()) return; //NOT SURE IF PUTTING THIS HERE IS A GOOD IDEA, TEST IT TO SEE IF IT IS OK.

        try
        {
            waitOneFullHardwareCycle();
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
    }

    public final void waitNextCycle ()
    {
        try
        {
            waitForNextHardwareCycle();
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
    }

    public int getGyroDirection () //placeholder
    {
        //return gyroSensor.getRotation();
        return gyroSensor.getHeading();
        //return 42.0; //testing
    }

    public double getIMUHeading ()
    {
        double yaw = navX.getYaw();
        double toReturn = yaw;
        if (yaw < 0.0) toReturn = 360 + yaw;
        return toReturn;
    }
/*
    public void restartRobot()
    {
        ftcRCA.requestRobotRestart();
    }
*/
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //MOVEMENT:

    //changes to make: unify moveTime and move with encoder into method with choice of encoder or time as a parameter.
    //do the same thing for rotation except with three options: gyro, encoder, and time.

    public final void setMotorPower(DcMotor motor, double power) //maybe not neccessary
    {
        motor.setPower(power);
    }

    public final void setLeftDrivePower (double power)
    {
        setMotorPower(leftFrontMotor, power);
        setMotorPower(leftBackMotor, power);
        //setMotorPower(leftMidMotor, power);
    }

    public final void setRightDrivePower (double power)
    {
        setMotorPower(rightFrontMotor, power);
        setMotorPower(rightBackMotor, power);
       // setMotorPower(rightMidMotor, power);
    }

    public final void setDrivePower (double power)
    {
        setLeftDrivePower(power);
        setRightDrivePower(power);
    }

    public void setStrafePower (double power)
    {
        setMotorPower (leftFrontMotor, power);
        setMotorPower (rightFrontMotor, -power);
        setMotorPower (leftBackMotor, -power);
        setMotorPower (rightBackMotor, power);
    }

    //add setLeftStrafePower and setRightStrafePower once I figure out turning which wheels makes it go where.


    public final void stopDrivetrain ()
    {
        setDrivePower(0);
        waitFullCycle();
        setDrivePower(0);
        waitFullCycle(); //not sure about thsi one.
    }

    public final void resetDriveEncoders ()
    {
        for (DcMotor dcm: driveMotors)
        {
            resetEncoder(dcm);
        }

    }

    public int getEncoderValue (DcMotor dcm)
    {
        if (motorToNumber(dcm) < 0) return dcm.getCurrentPosition();
        else return (dcm.getCurrentPosition() - driveMotorInitValues[motorToNumber(dcm)]);
    }

    public int motorToNumber (DcMotor dcm)
    {
        if (dcm == leftFrontMotor)
        {
            return 0;
        }

        if (dcm == rightFrontMotor)
        {
            return 1;
        }

        if (dcm == leftBackMotor)
        {
            return 2;
        }

        if (dcm == rightBackMotor)
        {
            return 3;
        }

        else
        {
            return -1;
        }
    }

    public void resetEncoder (DcMotor dcm)
    {
        driveMotorInitValues[motorToNumber(dcm)] = dcm.getCurrentPosition();
    }

    public boolean hasEncoderReached (DcMotor dcm, int encoderCount) //assumes that encoders start at 0 and are not moving to zero.
    {
        if (motorToNumber(dcm) == -1) return false;

        if (encoderCount > 0)
        {
            if (getEncoderValue(dcm) < encoderCount) return false;
            else return true;
        }

        else if (encoderCount < 0)
        {
            if (getEncoderValue(dcm) > encoderCount) return false;
            else return true;
        }

        else //encoderCount is 0
        {
            return (getEncoderValue(dcm) == 0);
        }
    }

    public final int getSideEncoderAverage (boolean side)
    {
        //if (side == RIGHT) return getEncoderValue(rightBackMotor);
        int addon = (side == RIGHT ? 1 : 0);
        int sum = getEncoderValue(driveMotors[0 + addon]) + getEncoderValue(driveMotors[2 + addon]);
        int average = (int) (1.0 * sum / 2.0);
        return average;
    }

    public final int getDriveEncoderAverage ()
    {
        double doubleAverage = (1.0 * (getSideEncoderAverage(LEFT) + getSideEncoderAverage(RIGHT))) / 2.0;
        return (int) doubleAverage;
    }

    public final int getStrafeEncoderAverage ()
    {
        double sum = getEncoderValue(leftFrontMotor) + getEncoderValue(rightBackMotor) - getEncoderValue(rightFrontMotor) - getEncoderValue(leftBackMotor);
        double average = sum / 4;
        return (int) average;
    }

    public final int getTurnEncoderAverage ()
    {
        double doubleAverage = (1.0 * (getSideEncoderAverage(LEFT) - getSideEncoderAverage(RIGHT))) / 2.0;
        return (int) doubleAverage;
    }


    public final boolean driveEncodersHaveReached(int encoderCount)
    {
        //return (hasEncoderReached(leftFrontMotor, encoderCount) && hasEncoderReached(rightFrontMotor, encoderCount)); //OLD METHOD
        if (encoderCount > 0)
        {
            if (getDriveEncoderAverage() < encoderCount) return false;
            else return true;
        }

        else if (encoderCount < 0)
        {
            if (getDriveEncoderAverage() > encoderCount) return false;
            else return true;
        }

        else //encoderCount is 0
        {
            return (getDriveEncoderAverage() == 0);
        }
    }

    public final boolean strafeEncodersHaveReached(int encoderCount)
    {
        //return (hasEncoderReached(leftFrontMotor, encoderCount) && hasEncoderReached(rightFrontMotor, encoderCount)); //OLD METHOD
        if (encoderCount > 0)
        {
            if (getStrafeEncoderAverage() < encoderCount) return false;
            else return true;
        }

        else if (encoderCount < 0)
        {
            if (getStrafeEncoderAverage() > encoderCount) return false;
            else return true;
        }

        else //encoderCount is 0
        {
            return (getStrafeEncoderAverage() == 0);
        }
    }

    public final boolean turnEncodersHaveReached(int encoderCount)
    {
        //return (hasEncoderReached(leftFrontMotor, encoderCount) && hasEncoderReached(rightFrontMotor, -encoderCount)); //make sure the minus sign on rightFrontMotor works.
        if (encoderCount > 0)
        {
            if (getTurnEncoderAverage() < encoderCount) return false;
            else return true;
        }

        else if (encoderCount < 0)
        {
            if (getTurnEncoderAverage() > encoderCount) return false;
            else return true;
        }

        else //encoderCount is 0
        {
            return (getTurnEncoderAverage() == 0);
        }
    }

    public String getModeText (double mode)
    {
        if (mode == NORMAL) return "Normal";
        else if (mode == ENCODER) return "Encoder";
        else if (mode == GYRO) return "Gyro";
        else return "";
    }

    public final void move (double distance, double... params)
    {
        if (!runConditions()) return;

        double power = DEFAULT_DRIVE_POWER;
        double mode = NORMAL;

        if (params.length > 2)
        {
            return;
        }

        else if (params.length == 1)
        {
            if (Math.abs(params[0]) < 1.1) //second parameter is power, 1.1 used instead of 1 to completely deal with floating point inaccuracy
            {
                power = params[0];
            }

            else
            {
                mode = params[0];
                if (mode != NORMAL)
                {
                    power = DEFAULT_SYNC_POWER;
                }
            }
        }

        else if (params.length == 2)
        {
            power = params[0];
            mode = params[1];
        }

        //Main method body:

        if (power * distance < 0)
        {
            power = -power;
        }

        int encoderCount = distanceToEncoderCount(distance);
        writeToLog("MOVING: Distance = " + distance + ", Encoder Count = " + encoderCount + ", Mode = " + getModeText(mode) + ", Power = " + power);
        writeToLog("MOVING: UnReset encoder values are LFM: " + getEncoderValue(leftFrontMotor) + ", " + getEncoderValue(rightFrontMotor));
        //navX.zeroYaw();

        double powerChange = 0;
        double updateTime = ((mode == ENCODER) ? ENCODER_SYNC_UPDATE_TIME : GYRO_SYNC_UPDATE_TIME);

        resetDriveEncoders();
        writeToLog("MOVING: Initialized encoder values (should be 0) are LFM: " + getEncoderValue(leftFrontMotor) + ", RFM = " + getEncoderValue(rightFrontMotor));


        int i = 0;
        int prevYawsSize = 7;
        ArrayList<Double> prevYaws = new ArrayList<Double>(prevYawsSize);
        for (int j = 0; j < prevYawsSize; j++) prevYaws.add(0.0);

        setDrivePower(power);
        //double prevYaw;

        while (runConditions() && !driveEncodersHaveReached(encoderCount)) //change back to runConditions if it works, change back to driveEncodersHaveReached if it works
        {
            if (i >= 2) i = 0;

            if (mode != NORMAL)
            {
                if (mode == ENCODER)
                {
                    double frontDifference = getEncoderValue(leftFrontMotor) - getEncoderValue(rightFrontMotor);
                    double backDifference = getEncoderValue(leftBackMotor) - getEncoderValue(rightBackMotor);
                    double averageDifference = (frontDifference + backDifference) / 2;
                    powerChange = backDifference * ENCODER_SYNC_PROPORTIONALITY_CONSTANT;
                }

                else if (mode == GYRO)
                {
                    double yaw = navX.getYaw();
                    if (prevYaws.size() >= prevYawsSize) prevYaws.remove(0);
                    prevYaws.add(yaw);
                    powerChange = yaw * GYRO_SYNC_PROPORTIONALITY_CONSTANT;
                    if (prevYaws.size() >= prevYawsSize)
                    {
                        double roc = (yaw - prevYaws.get(prevYaws.size() - 2)) / updateTime;
                        powerChange = powerChange - (GYRO_SYNC_DIFFERENTIAL_CONSTANT * roc);

                        double sum = 0;
                        for (Double d: prevYaws)
                        {
                            sum += d;
                        }

                        powerChange = powerChange + (sum * GYRO_SYNC_INTEGRAL_CONSTANT);
                    }

                }

                setLeftDrivePower(Range.clip(power - powerChange, -1.0, 1.0));
                setRightDrivePower(Range.clip(power + powerChange, -1.0, 1.0));

                double initTime = gameTimer.time();
                while ((gameTimer.time() - initTime) < updateTime)
                {
                    if (driveEncodersHaveReached(encoderCount))
                    {
                        break;
                    }
                }
                //if (i == 0) writeToLog("IMU YAW: " + navX.getYaw());
                i++;
            }

            //waitFullCycle();

            //do nothing if mode is NORMAL.
        }
        stopDrivetrain();
        if (!runConditions()) return;
        //writeToLog("MOVING: Final encoder values are LFM: " + getEncoderValue(leftFrontMotor) + ", " + getEncoderValue(rightFrontMotor));
        waitFullCycle();
        //waitFullCycle();
        sleep(99); //maybe reduce this if it wastes too much time to have this safety interval.
    }

    public final void moveSmooth (double distance) //untested
    {
        if (!runConditions()) return;

        //Main method body:

        int encoderCount = distanceToEncoderCount(distance);
        writeToLog("MOVING: Distance = " + distance + ", Encoder Count = " + encoderCount);
        writeToLog("MOVING: UnReset encoder values are LFM: " + getEncoderValue(leftFrontMotor) + ", " + getEncoderValue(rightFrontMotor));
        //double initialDirection = getGyroDirection();

        resetDriveEncoders();
        writeToLog("MOVING: Initialized encoder values (should be 0) are LFM: " + getEncoderValue(leftFrontMotor) + ", RFM = " + getEncoderValue(rightFrontMotor));
        //setDrivePower(power);

        double sign = (distance >= 0 ? 1 : -1);

        double minPower = 0.068;
        double maxPower = 0.75;
        double originalMaxPower = maxPower;
        double smoothDistance = Math.abs(distance / 2.3);
        int smoothEncoderCounts = distanceToEncoderCount(smoothDistance);

        while (runConditions() && !driveEncodersHaveReached(encoderCount)) //change back to runConditions if it works, change back to driveEncodersHaveReached if it works
        {
            double power = minPower;
            int dea = Math.abs(getDriveEncoderAverage());

            if (dea <= smoothEncoderCounts)
            {
                power = minPower + (((double) dea / smoothEncoderCounts) * (maxPower - minPower));
                telemetry.addData("3", "In ramp up");

            }

            else if (dea >= Math.abs(encoderCount) - smoothEncoderCounts)
            {
                //power = minPower + ((Math.abs(Math.abs(encoderCount) - dea) / (double) smoothEncoderCounts) * (maxPower - minPower));
                if (maxPower == originalMaxPower) maxPower = maxPower / 2;
                double distanceFromRampDown = dea - (Math.abs(encoderCount) - smoothEncoderCounts);
                double proportionOfDistance = distanceFromRampDown / (double) smoothEncoderCounts;
                power = maxPower - (proportionOfDistance * (maxPower - minPower));
                telemetry.addData("3", "In ramp down");
                telemetry.addData("5", "proportion: " + proportionOfDistance);
            }

            else
            {
                power = maxPower;
                telemetry.addData("3", "In middle");
            }

            telemetry.addData("2", "Power = " + power);
            telemetry.addData("4", "DEA: " + dea + " Target: " + encoderCount);
            setDrivePower(power * sign);

        }
        stopDrivetrain();
        if (!runConditions()) return;
        //writeToLog("MOVING: Final encoder values are LFM: " + getEncoderValue(leftFrontMotor) + ", " + getEncoderValue(rightFrontMotor));
        waitFullCycle();
        //waitFullCycle();
        sleep(99); //maybe reduce this if it wastes too much time to have this safety interval.
    }

    public void moveSimple (int count)
    {
        setDrivePower(DEFAULT_DRIVE_POWER);
        while (runConditions() && !driveEncodersHaveReached(count))
        {
            //waitFullCycle();
        }
        stopDrivetrain();
        //waitFullCycle();
    }

    public final void moveTime(int time, double power)
    {
        writeToLog("Moving at " + power + " power for " + time + " ms");
        setDrivePower(power);
        sleep(time);
        stopDrivetrain();
        if (!runConditions()) return;
        waitFullCycle();
        //stopDrivetrain();
    }

    public final void strafe (double distance, double... params)
    {
        if (!runConditions()) return;

        double power = DEFAULT_DRIVE_POWER;
        double mode = NORMAL;

        if (params.length > 2)
        {
            return;
        }

        else if (params.length == 1)
        {
            if (Math.abs(params[0]) < 1.1) //second parameter is power, 1.1 used instead of 1 to completely deal with floating point inaccuracy
            {
                power = params[0];
            }

            else
            {
                mode = params[0];
                if (mode != NORMAL)
                {
                    power = DEFAULT_SYNC_POWER;
                }
            }
        }

        else if (params.length == 2)
        {
            power = params[0];
            mode = params[1];
        }

        //Main method body:

        if (power * distance < 0)
        {
            power = -power;
        }

        int encoderCount = distanceToStrafeEncoderCount(distance);
        writeToLog("STRAFING: Distance = " + distance + ", Encoder Count = " + encoderCount + ", Mode = " + getModeText(mode) + ", Power = " + power);
        writeToLog("STRAFING: UnReset encoder values are LFM: " + getEncoderValue(leftFrontMotor) + ", " + getEncoderValue(rightFrontMotor));
        //navX.zeroYaw();

        double powerChange = 0;
        double updateTime = ((mode == ENCODER) ? ENCODER_SYNC_UPDATE_TIME : GYRO_SYNC_UPDATE_TIME);

        resetDriveEncoders();
        writeToLog("STRAFING: Initialized encoder values (should be 0) are LFM: " + getEncoderValue(leftFrontMotor) + ", RFM = " + getEncoderValue(rightFrontMotor));
/*
        int i = 0;
        int prevYawsSize = 7;
        ArrayList<Double> prevYaws = new ArrayList<Double>(prevYawsSize);
        for (int j = 0; j < prevYawsSize; j++) prevYaws.add(0.0);
*/
        setStrafePower(power);


        //double prevYaw;

        while (runConditions() && !strafeEncodersHaveReached(encoderCount)) //change back to runConditions if it works, change back to driveEncodersHaveReached if it works
        {
            //if (i >= 2) i = 0;

            if (mode != NORMAL)
            {
                /*
                if (mode == ENCODER)
                {
                    double frontDifference = getEncoderValue(leftFrontMotor) - getEncoderValue(rightFrontMotor);
                    double backDifference = getEncoderValue(leftBackMotor) - getEncoderValue(rightBackMotor);
                    double averageDifference = (frontDifference + backDifference) / 2;
                    powerChange = backDifference * ENCODER_SYNC_PROPORTIONALITY_CONSTANT;
                }

                else if (mode == GYRO)
                {
                    double yaw = navX.getYaw();
                    if (prevYaws.size() >= prevYawsSize) prevYaws.remove(0);
                    prevYaws.add(yaw);
                    powerChange = yaw * GYRO_SYNC_PROPORTIONALITY_CONSTANT;
                    if (prevYaws.size() >= prevYawsSize)
                    {
                        double roc = (yaw - prevYaws.get(prevYaws.size() - 2)) / updateTime;
                        powerChange = powerChange - (GYRO_SYNC_DIFFERENTIAL_CONSTANT * roc);

                        double sum = 0;
                        for (Double d: prevYaws)
                        {
                            sum += d;
                        }

                        powerChange = powerChange + (sum * GYRO_SYNC_INTEGRAL_CONSTANT);
                    }

                }

                setLeftDrivePower(Range.clip(power - powerChange, -1.0, 1.0));
                setRightDrivePower(Range.clip(power + powerChange, -1.0, 1.0));

                double initTime = gameTimer.time();
                while ((gameTimer.time() - initTime) < updateTime)
                {
                    if (driveEncodersHaveReached(encoderCount))
                    {
                        break;
                    }
                }
                if (i == 0) writeToLog("IMU YAW: " + navX.getYaw());
                i++;
                */
            }

            //waitFullCycle();

            //do nothing if mode is NORMAL.
        }
        stopDrivetrain();
        if (!runConditions()) return;
        //writeToLog("MOVING: Final encoder values are LFM: " + getEncoderValue(leftFrontMotor) + ", " + getEncoderValue(rightFrontMotor));
        waitFullCycle();
        //waitFullCycle();
        sleep(99); //maybe reduce this if it wastes too much time to have this safety interval.
    }

    public final void strafeTime(int time, double power)
    {
        writeToLog("Strafing at " + power + " power for " + time + " ms");
        setStrafePower(power);
        sleep(time);
        stopDrivetrain();
        if (!runConditions()) return;
        waitFullCycle();
        //stopDrivetrain();
    }

    //ROTATION:

    public final void setTurnPower (double power) //problm with this?
    {
        setLeftDrivePower(power);
        setRightDrivePower(-power);
    }

    public final void waitForGyroRotation (double degrees) //degrees must be less than 355
    {
        //convert degrees to proper value for this method
        gyroSensor.calibrate();
        while (gyroSensor.isCalibrating()) {};
        waitFullCycle();
        gyroSensor.resetZAxisIntegrator();
        waitFullCycle();
        sleep(500);
        if (degrees < 0)
        {
            while (runConditions() && (getGyroDirection() > (360 - degrees) || getGyroDirection() < 5))
            {

            }
        }

        else
        {
            while (runConditions() && (getGyroDirection() < degrees || getGyroDirection() > 355))
            {

            }
        }
    }

    public final void waitForIMURotation (double degrees) //degrees must be less than 355
    {
        //convert degrees to proper value for this method
        /*
        sleep(500);
        if (navX.getFusedHeading() > degrees)
        {
            while (runConditions() && navX.getFusedHeading() > (degrees))
            {

            }
        }

        else
        {
            while (runConditions() && navX.getFusedHeading() < degrees)
            {

            }
        }*/

        waitFullCycle();
        /*
        while (runConditions() && navX.isCalibrating());
        sleep(50);
        waitFullCycle();
        */
        if (degrees < 0)
        {
            while (runConditions() && (getIMUHeading() > (360 + degrees) || getIMUHeading() < 4))
            {
                waitNextCycle();
            }
        }

        else
        {
            while (runConditions() && (getIMUHeading() < degrees || getIMUHeading() > 356))
            {
                waitNextCycle();
            }
        }
    }

    public final void waitForAbsoluteGyroRotation (double degrees)
    {
        //finish later
    }

    public final void swingTurn (double distance, boolean side, double power)
    {
        if (power * distance < 0)
        {
            power = -power;
        }

        resetDriveEncoders();
        if (side == RIGHT) setRightDrivePower(power);
        else setLeftDrivePower(power);

        int encoderCount = distanceToEncoderCount(distance);

        while (runConditions() && (side == RIGHT ? !hasEncoderReached(rightFrontMotor, encoderCount) : !hasEncoderReached(leftFrontMotor, encoderCount))) //change back to runConditions if neecessary
        {

        }
        stopDrivetrain();
    }

    public final void swingTurn (double distance, boolean side)
    {

    }

    public final void rotate (double degrees, double power) //gyro rotation, add thing to make negative degrees = negative power.
    {
        while (navX.isMoving());
        if (power * degrees < 0) power = -power;
        navX.zeroYaw();
        setTurnPower(power);
        waitForGyroRotation(degrees);
        stopDrivetrain();
    }

    public final void rotateIMU (double degrees, double power) //Do NOT give degrees more than ~350
    {
        if (power * degrees < 0) power = -power;
        waitFullCycle();
        navX.zeroYaw();
        waitFullCycle();
        sleep(100);
        writeToLog("Starting high power rotation");
        setTurnPower(power);
        //waitForIMURotation(degrees);
        waitFullCycle();
        /*
        while (runConditions() && navX.isCalibrating());
        sleep(50);
        waitFullCycle();
        */
        if (degrees < 0)
        {
            while (runConditions() && (getIMUHeading() > (360 + degrees) || getIMUHeading() < 4))
            {
                waitNextCycle();
            }
        }

        else
        {
            while (runConditions() && (getIMUHeading() < degrees || getIMUHeading() > 356))
            {
                waitNextCycle();
            }
        }
        writeToLog("Done with high power rotation");
        stopDrivetrain();
        waitFullCycle();
        /*
        if (degrees > 0)
        {
            double overshoot = 0.7;
            setTurnPower(-0.16);
            while (runConditions() && getIMUHeading() > (degrees - overshoot))
            {
                waitNextCycle();
            }
        }

        else if (degrees < 0)
        {
            double overshoot = 0.7;
            setTurnPower(0.16);
            while (runConditions() && getIMUHeading() <  (degrees + overshoot))
            {
                waitNextCycle();
            }
        }
*/

        final double correctionBasePower = 0.108;
        final double powerMultiplier = 0.06;
        final double margin = 0.4;
        final int correctionInterval = 100;
        double correctionPower = correctionBasePower;

        if (degrees > 0)
        {

            while (runConditions() && Math.abs(getIMUHeading() - degrees) > margin)
            {
                while (runConditions() && Math.abs(getIMUHeading() - degrees) > margin)
                {
                    double difference = Math.abs(getIMUHeading() - degrees);
                    //if (difference > 4) correctionPower = DEFAULT_TURN_POWER_HIGH;
                    //else correctionPower = correctionBasePower;

                    if (getIMUHeading() < degrees) setTurnPower(correctionPower);
                    else if (getIMUHeading() > degrees) setTurnPower(-correctionPower);
                    waitNextCycle();
                }

                waitFullCycle();
                stopDrivetrain();
                sleep(correctionInterval);
                waitFullCycle();
            }
        }

        else if (degrees < 0)
        {
            while (runConditions() && Math.abs(getIMUHeading() - (360 + degrees)) > margin)
            {
                while (runConditions() && Math.abs(getIMUHeading() - (360 + degrees)) > margin)
                {
                    double difference = Math.abs(getIMUHeading() - (360 + degrees));
                    //if (difference > 4) correctionPower = DEFAULT_TURN_POWER_HIGH;
                    //else correctionPower = correctionBasePower;

                    if (getIMUHeading() < 360 + degrees) setTurnPower(correctionPower);
                    else if (getIMUHeading() > 360 + degrees) setTurnPower(-correctionPower);
                    waitNextCycle();
                }

                waitFullCycle();
                stopDrivetrain();
                sleep(correctionInterval);
                waitFullCycle();
            }
        }


        stopDrivetrain();
        waitFullCycle();
        navX.zeroYaw();
        waitFullCycle();


        /*
        navX.zeroYaw();
        setTurnPower((power < 0 ? 1 : -1) * 0.12);
        waitForIMURotation(difference);
        stopDrivetrain();
        waitFullCycle();\*/
    }

    public final void rotateIMU (double degrees) //gyro rotation, add thing to make negative degrees = negative power.
    {
        rotateIMU(degrees, DEFAULT_TURN_POWER_HIGH);
    }

    public final void rotate (double degrees)
    {
        rotate (degrees, DEFAULT_TURN_POWER);
    }

    public final void rotateEncoder (double distance, double power) //add thing to make negative distance = negative power.
    {
        if (power * distance < 0)
        {
            power = -power;
        }

        resetDriveEncoders();
        setTurnPower(power);
        while (runConditions() && !turnEncodersHaveReached(distanceToEncoderCount(distance))) //change back to runConditions if neecessary
        {
            //waitFullCycle();
        }
        stopDrivetrain();
        //waitFullCycle();
    }

    public final void rotateEncoder (double distance)
    {
        rotateEncoder(distance, DEFAULT_TURN_POWER_HIGH);
    }

    public final void rotateTime (int time, double power) //time in millis
    {
        setTurnPower(power);
        sleep (time);
        stopDrivetrain();
    }

    //ATTACHMENTS:

    public final void moveDoor(double position)
    {
        doorServo.setPosition(position);
    }

    public final void moveHook(double position)
    {
        hookServo.setPosition(position);
    }

    public final void setSweeperPower (double power)
    {
        setMotorPower(sweeperMotor, power);
    }

    public final void moveRackAndPinion (double position)
    {
        autoExtendServo.setPosition(position);
    }

    public final void moveBallClamp(double position)
    {
        clampServo.setPosition(position);
    }

    public final boolean isBallLoaded () //use sensor soon
    {
        return true;
    }

    public final void shoot()
    {
        double error = TARGET_VOLTAGE - voltage;
        double motorOut = (error * kP) + .82;
        motorOut = Range.clip(motorOut, 0, 1);
        setMotorPower(flywheelLeft, motorOut);
        setMotorPower(flywheelRight, motorOut);
        sleep(200);

        /*
        double error = TARGET_SPEED - flywheelSpeed();
        double derivative = error - lastError;
        integral += error;
        lastError = error;

        double motorOut = (kP * error) + (kI * integral) + (kD * derivative) + .86;
        motorOut = Range.clip(motorOut, 0.0, 1.0);
        setMotorPower(flywheelLeft, motorOut);
        setMotorPower(flywheelRight, motorOut);
        sleep(200);
        */
    }

    public final void stopShooting()
    {
        setMotorPower(flywheelLeft, 0);
        setMotorPower(flywheelRight, 0);
    }

    public class ShootThread implements Runnable
    {
        Thread thrd;

        private boolean suspended;
        private boolean stopped;

        private boolean opModeHasBeenActive = false;

        ShootThread()
        {
            thrd = new Thread(this);
            suspended = true;
            stopped = false;
            voltage = batteryVoltage();
            thrd.start();
        }

        public void run()
        {
            try
            {
                while(!opModeHasBeenActive || runConditions()) //should keep running
                {
                    if (opModeIsActive()) opModeHasBeenActive = true;
                    shoot();

                    synchronized (this)
                    {
                        while(suspended)
                        {
                            if (opModeIsActive()) opModeHasBeenActive = true;
                            stopShooting();
                            wait();
                        }
                        if(stopped) break;
                    }
                }
            } catch(InterruptedException exc){}
        }

        synchronized void mStop()
        {
            stopped = true;
            suspended = false;
            notify();
        }

        synchronized void mSuspend()
        {
            suspended = true;
        }

        synchronized void mResume ()
        {
            suspended = false;
            voltage = batteryVoltage();
            integral = 0;
            notify();
        }

        public boolean isSuspended()
        {
            return suspended;
        }
    }

    public double batteryVoltage()
    {
        return this.hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public double flywheelSpeed()
    {
        long time = System.nanoTime();
        long encoder = flywheelRight.getCurrentPosition();
        double speed = (encoder - lastEncoder) / (time - lastTime);

        lastEncoder = encoder;
        lastTime = time;

        return speed;
    }

    public double getFloorBrightness ()
    {
        return (colorSensorDown.red() + colorSensorDown.green() + colorSensorDown.blue());
    }

    //JUST FOR FUN:
/*
    public void playMusic (int resid)
    {
        if (runConditions() && !MUSIC_ON) return;
        mediaPlayer = MediaPlayer.create(ftcRCA, resid);
        mediaPlayer.start();
    }

    public void stopMusic ()
    {
        if (mediaPlayer == null) return;
        mediaPlayer.stop();
        mediaPlayer = null;
    }
    */

    public void initializeVuforia(){
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey = "AcjcUvP/////AAAAGZk8Oo2BiUH4lYtmypMLxPoh5M3gwDE8WJsu13qi2h2KT3hWI+28EgFYToXpq7lUI/2xGSArueKvAzg4" +
                "+kgBe7jXAtv7l8U1v1wxVvVbrFXRuEBwUPYPNkqUPZeD+xiVlRVqPObIoBHTYfS6i+PtGBKu+lpOGCi2eIuTvhEawydEF17lD24K8ip9cWuVVIw6LAzBjckFU" +
                "soVgCsmnOdsgQjxJ8xRr3nmO+O88LYAMvG9x+rLcjFIF9u7K6Xx54JvAa9aa1b+55CtclWL1eK76YcIT8uabochFK8iZytmVKAwqfxIHasdokE12cTCwvY9m" +
                "D5KBHdLjwKPj6sjqjAWjEzcAW9GcSj6YBGcJNZRB5pU";
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");

        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");
    }

    public void trackBeacons(){
        beacons.activate();
        for(VuforiaTrackable beac : beacons) {
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();

            if(pose != null) {
                VectorF translation = pose.getTranslation();

                telemetry.addData(beac.getName() + "-Translation", translation);

                double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));

                telemetry.addData(beac.getName() + "-Degrees", degreesToTurn);
            }
        }
        telemetry.update();
    }

    private void initializeCameraView()
    {
        final Activity mActivity = (Activity) hardwareMap.appContext;
        mActivity.setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);

        class CameraThread implements Runnable
        {
            private JavaCameraView mCameraView = null;
            public JavaCameraView getJCV() {return mCameraView;}

            public void run()
            {
                mCameraView = new JavaCameraView(mActivity, CameraBridgeViewBase.CAMERA_ID_BACK);
                mCameraView.setMaxFrameSize(480, 320);

                ViewGroup.LayoutParams params = new ViewGroup.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT);
                mCameraView.setLayoutParams(params);

                ViewGroup img = (ViewGroup) mActivity.findViewById(R.id.cameraMonitorViewId);
                img.addView(mCameraView, 0);
            }
        }

        CameraThread cameraThread = new CameraThread();
        mActivity.runOnUiThread(cameraThread);

        JavaCameraView mCameraView = null;
        while(mCameraView == null)
        {
            mCameraView = cameraThread.getJCV();
        }

        openCVView = mCameraView;
        openCVView.setVisibility(CameraBridgeViewBase.VISIBLE);
        openCVView.setCvCameraViewListener(this);

        width = openCVView.getMeasuredWidth();
        height = openCVView.getMeasuredHeight();
    }

    protected void initializeOpenCV()
    {
        BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(hardwareMap.appContext)
        {
            @Override
            public void onManagerConnected(int status)
            {
                switch (status)
                {
                    case LoaderCallbackInterface.SUCCESS:
                    {
                        initializeCameraView();
                        openCVView.enableView();
                    } break;
                    default:
                    {
                        super.onManagerConnected(status);
                    } break;
                }
            }
        };

        if(!OpenCVLoader.initDebug())
        {
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_2_0, hardwareMap.appContext, mLoaderCallback);
        }

        else
        {
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    protected void endCamera()
    {
        openCVView.disableView();
    }
    @Override
    public void onCameraViewStarted(int width, int height)
    {

    }

    @Override
    public void onCameraViewStopped()
    {
        openCVView.disableView();
    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame)
    {
        Mat rgb = inputFrame.rgba();
        newImage = true;

        Mat rtrnImage = rgb;

        return rtrnImage;
    }
}
