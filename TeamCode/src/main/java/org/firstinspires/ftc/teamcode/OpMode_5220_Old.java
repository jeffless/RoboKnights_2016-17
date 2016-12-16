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

import android.media.MediaPlayer;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import java.text.DecimalFormat;
import java.util.ArrayList;

//import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
//hello!

//TRY TO ADD METHODS FOR STRAFING DIAGONALLY (USING ONLY TWO WHEELS)

//Currently using FTC SDK 2.2

public abstract class OpMode_5220_Old extends LinearOpMode
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

    protected static enum ProgramType {UNDECIDED, AUTONOMOUS, TELEOP};
    protected static ProgramType programType = ProgramType.UNDECIDED;

    protected static final int TELEOP_TIME_LIMIT = 1200000; //currently 20 minutes, more than enough for any single run.
    protected static final int AUTONOMOUS_TIME_LIMIT = 29000;

    protected static final double NORMAL = 2;
    protected static final double ENCODER = 3;
    protected static final double GYRO = 4;

    protected static final double WHEEL_DIAMETER = 6.0; //in inches
    protected static final double GEAR_RATIO = 3.0 / 4.0;
    protected static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    protected static final int ENCODER_COUNTS_PER_ROTATION = 1120; //WAS 1440

    protected static final double SWIVEL_INIT = 0.7529; //// may be reset in TeleOp
    protected static final double SWIVEL_90 = 0.4706;
    protected static final double SWIVEL_180 = SWIVEL_90 * 2;
    protected static final double SWIVEL_360 = SWIVEL_180 * 2;

    protected static final double LEFT_CLIMBER_INIT = 0.0;
    protected static final double RIGHT_CLIMBER_INIT = 0.9;
    protected static final double CLIMBER_OFFSET = 0.7;

    protected static final double LEFT_HOOK_ADJUST_INIT = 1.0;
    protected static final double RIGHT_HOOK_ADJUST_INIT = 0.0;
    protected static final double HOOK_ADJUST_OFFSET = 1.0;


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

    protected static final double LINE_WHITE_THRESHOLD = 50;

    //MOTORS AND SERVOS:

    protected static final String[] motorNames = {}; //Fill this in later.

    protected DeviceInterfaceModule cdim;
    protected DcMotor leftFrontMotor;
    protected DcMotor rightFrontMotor;
    protected DcMotor leftBackMotor;
    protected DcMotor rightBackMotor;
    protected DcMotor sweeperMotor1;
    protected DcMotor sweeperMotor2;
    protected DcMotor slideMotor;
    protected DcMotor liftMotor1;
    protected DcMotor liftMotor2;

    protected DcMotor[] driveMotors = new DcMotor[4];
    protected int[] driveMotorInitValues = new int[4];
    protected int slideInit;

    protected Servo swivelServo;
    protected Servo releaseServo;
    protected Servo buttonServo;
    protected Servo leftWallServo;
    protected Servo rightWallServo;
    protected Servo leftDumpServo;
    protected Servo rightDumpServo;
    protected Servo leftClimberServo;
    protected Servo rightClimberServo;
    protected Servo hookServo;
    protected Servo leftHookAdjustServo;
    protected Servo rightHookAdjustServo;

    protected double swivelServoInit;

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
    protected boolean pinOn = true;
    protected boolean programFinished = false; //allows manual termination of the program in an orderly fashion, especially for autonomous
    protected boolean debugLoopOn = false;
    protected Stopwatch gameTimer;
    protected boolean isArmMoving = false;
    protected int phase = HAS_NOT_STARTED;
    protected double swivelPosition;

    protected MediaPlayer mediaPlayer;
    public static final boolean MUSIC_ON = true;

    public void setup()//this and the declarations above are the equivalent of the pragmas in RobotC
    {
        phase = SETUP;

        //ftcRCA = FtcRobotControllerActivity.ftcRCA;

        hardwareMap.logDevices();

        cdim = hardwareMap.deviceInterfaceModule.get("Device Interface Module 4");

        leftFrontMotor = hardwareMap.dcMotor.get("lf");
        rightFrontMotor = hardwareMap.dcMotor.get("rf");
        leftBackMotor = hardwareMap.dcMotor.get("lb");
        rightBackMotor = hardwareMap.dcMotor.get("rb");

        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        driveMotors[0] = leftFrontMotor;
        driveMotors[1] = rightFrontMotor;
        driveMotors[2] = leftBackMotor;
        driveMotors[3] = rightBackMotor;

        sweeperMotor1 = hardwareMap.dcMotor.get("sweeper1");
        sweeperMotor2 = hardwareMap.dcMotor.get("sweeper1");
        sweeperMotor1.setDirection(DcMotor.Direction.REVERSE);
        sweeperMotor2.setDirection(DcMotor.Direction.REVERSE);
        slideMotor = hardwareMap.dcMotor.get("slides");
        slideInit = slideMotor.getCurrentPosition();

        //configure lift motors
        liftMotor1 = hardwareMap.dcMotor.get("lm1");
        liftMotor2 = hardwareMap.dcMotor.get("lm2");

        swivelServo = hardwareMap.servo.get("sServo");
        releaseServo = hardwareMap.servo.get("rServo");
        buttonServo = hardwareMap.servo.get("bServo");
        leftWallServo = hardwareMap.servo.get ("lwServo");
        rightWallServo = hardwareMap.servo.get ("rwServo");
        leftDumpServo = hardwareMap.servo.get("ldServo");
        rightDumpServo = hardwareMap.servo.get("rdServo");
        leftClimberServo = hardwareMap.servo.get("lcServo");
        rightClimberServo = hardwareMap.servo.get("rcServo");
        hookServo = hardwareMap.servo.get("hServo");
        leftHookAdjustServo = hardwareMap.servo.get("laServo");
        rightHookAdjustServo = hardwareMap.servo.get("raServo");

        colorSensorDown = hardwareMap.colorSensor.get("cSensor1");
        colorSensorFront = hardwareMap.colorSensor.get("cSensor2");
        colorSensorFront.setI2cAddress(I2cAddr.create7bit(0x3E));//IF 7 BIT DOESN'T WORK TRY 8 BIT ADDRESS (I2cAddr.create8bit(0x3E)), OR USING I2CADDR CONSTRUCTOR
        // in hex, 0x3e = 62. deault address is 60 (reserved for colorSensorDown)
        colorSensorFront.enableLed(false);
        colorSensorDown.enableLed(false);
        gyroSensor = hardwareMap.gyroSensor.get("gSensor");
        touchSensorFront = hardwareMap.touchSensor.get("tSensor1");

        navX = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("Device Interface Module 4"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData);
    }

    public void initialize()
    {
        moveDumper(DOWN);
        leftClimberServo.setPosition(LEFT_CLIMBER_INIT);
        rightClimberServo.setPosition(RIGHT_CLIMBER_INIT);
        buttonServo.setPosition(0.1);
        hookServo.setPosition(1.0);
        swivelServo.setPosition(SWIVEL_INIT);
        leftHookAdjustServo.setPosition(LEFT_HOOK_ADJUST_INIT);
        rightHookAdjustServo.setPosition(RIGHT_HOOK_ADJUST_INIT);

        waitFullCycle();

        gyroSensor.calibrate();
        while (runConditions() && gyroSensor.isCalibrating())
        {

        }
        waitFullCycle();
        gyroSensor.resetZAxisIntegrator();
        waitFullCycle();

        navX.zeroYaw();

        moveWall(DOWN);
        phase = INIT;

        writeToLog ("Down: " + colorSensorDown.getI2cAddress());
        writeToLog("Front: " + colorSensorFront.getI2cAddress());

    }

    public void waitForStart () throws InterruptedException
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

        waitForStart();

        phase = RUNNING;
        gameTimer = new Stopwatch();
        if (pinOn) releasePin();

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
                telemetry.addData("4", "Swivel: " + df.format(swivelServo.getPosition()) + ", Dumper: " + df.format(leftDumpServo.getPosition()));
                telemetry.addData("5", "Slides:" + getSlidePosition());

                telemetry.addData("6", "Down: R = " + colorSensorDown.red() + ", G = " + colorSensorDown.green() + ", B = " + colorSensorDown.blue() + ", A = " +  colorSensorDown.alpha());
                telemetry.addData("7", "Front: R = " + colorSensorFront.red() + ", G = " + colorSensorFront.green() + ", B = " + colorSensorFront.blue() + ", A = " +  colorSensorFront.alpha());
                telemetry.addData ("8", "Y,P,R,FH: " + yprf);

                //waitOneFullHardwareCycle();
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
/*
    public void setCustomSkin() //maybe transfer this sort of app modification stuff to ftcrobotcontrolleractivity. It might be more appropriate there.
    {
        View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.scanButton);
        relativeLayout.setBackgroundColor(Color.CYAN);
        View button1 = ((Activity) hardwareMap.appContext).findViewById(R.id.file_activate_button); //try different button if this doesn't work.
        View.OnLongClickListener listener = new View.OnLongClickListener()
        {
            public int numPresses = 0;
            public boolean onLongClick(View v)
            {
                telemetry.addData("4", "Button clicks: " + numPresses++);
                return false;
            }
        };
        button1.setOnLongClickListener(listener);
        View entireScreen = ((Activity) hardwareMap.appContext).findViewById(R.id.entire_screen); //try different button if this doesn't work.
        View button2 = ((Activity) hardwareMap.appContext).findViewById(R.id.textWifiDirectStatus); //try different button if this doesn't work.
        //button2.setOnLongClickListener(); //maybe put new teleOp thing here.
        //try adding a new listener to the buttons to make them do different things.
    }
*/
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
        return ((int) (distance * ((double)ENCODER_COUNTS_PER_ROTATION / 4.5))); //TEMPORARY STAND-IN UNTIL WE ACTUALLY FIGURE OUT THE CONVERSION RATIO. This guesstimate assumes that one rotation of the wheels moves the robot 4.5 inches sideways.
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

    void setStrafePower (double power)
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
        return (dcm.getCurrentPosition() - driveMotorInitValues[motorToNumber(dcm)]);
    }

    public int getSlidePosition ()
    {
        return slideMotor.getCurrentPosition() - slideInit;
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
        if (side == RIGHT) return getEncoderValue(rightBackMotor);
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
        navX.zeroYaw();

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
                if (i == 0) writeToLog("IMU YAW: " + navX.getYaw());
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
        navX.zeroYaw();

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

    public final void shoot ()
    {
        //shoot a ball
    }

    public static final double DUMPER_UP = 1.0;
    public static final double DUMPER_DOWN = 0.1;
    public static final double dumperHeight = 0.47;

    public final void moveDumper (boolean b)
    {
        moveDumper(b == UP ? DUMPER_UP : DUMPER_DOWN);
    }

    public final void moveDumper (double d)
    {
        leftDumpServo.setPosition(d);
        rightDumpServo.setPosition(1.0 - d);
    }

    public static final double COLLECT = SWIVEL_INIT;
    public static final double BLUE_HIGH = 0.863;
    public static final double BLUE_MEDIUM = 0.882;
    public static final double RED_HIGH = 0.66666;
    public static final double RED_MEDIUM = 0.635;

    public final void moveSwivel (double position)
    {
        swivelServo.setPosition(position);
    }

    public static final double leftWallInit = 0.48;
    public static final double rightWallInit = 0.5;
    public static final double wallOffset = 0.335;

    public final void moveWall (boolean position)
    {
        if (position == UP)
        {
            leftWallServo.setPosition(leftWallInit + wallOffset);
            rightWallServo.setPosition(rightWallInit - wallOffset);
        }

        else if (position == DOWN)
        {
            leftWallServo.setPosition(leftWallInit);
            rightWallServo.setPosition(rightWallInit);
        }
    }
    
    public final void setSweeperPower (double power)
    {
        sweeperMotor1.setPower(power);
        sweeperMotor2.setPower(power);
    }

    public final void setLiftPower (double power)
    {
        liftMotor1.setPower(power);
        liftMotor2.setPower(-power);
    }

    public final void setHookPosition(double position)
    {
        hookServo.setPosition(position);
    }

    public final void setHookPosition(boolean position)
    {
        setHookPosition(position == UP ? 0.0 : 1.0);
    }

    public final void setHookAdjustPosition (double position)
    {
        leftHookAdjustServo.setPosition(LEFT_HOOK_ADJUST_INIT - position);
        rightHookAdjustServo.setPosition(RIGHT_HOOK_ADJUST_INIT + position);
    }

    public final void moveSlides (int position)
    {
        if (getSlidePosition() < position)
        {
            slideMotor.setPower(1.0);
            while (runConditions() && getSlidePosition() < position)
            {

            }
        }

        if (getSlidePosition() > position)
        {
            slideMotor.setPower(-1.0);
            while (runConditions() && getSlidePosition() > position)
            {

            }
        }

        slideMotor.setPower(0);
        waitFullCycle();
        slideMotor.setPower(0);
        waitFullCycle();
    }

    public final void releasePin()
    {
        releaseServo.setPosition(1.0);
    }

    public void flingClimbers ()
    {
        if (!runConditions()) return;
        sleep (200);
       // writeToLog ("Flinging Climbers.");
        Stopwatch climberTimer = new Stopwatch();
        double timeInMillis = CLIMBER_FLING_TIME * 1000.0;
        while (runConditions() && climberTimer.time() < timeInMillis)
        {
            double newPosition = (climberTimer.time() / timeInMillis);
            if (newPosition > 1) newPosition = 1;
            moveDumper (newPosition);
            //waitFullCycle(); //not sure if needed here
        }
        sleep (1000);
        moveDumper(DOWN);
       // writeToLog("Climber flinging is done.");

    }

    public void hookAdjustRelease ()
    {
        if (!runConditions()) return;
        Stopwatch timer = new Stopwatch();
        double timeInMillis = HOOK_ADJUST_RELEASE_TIME * 1000.0;
        waitFullCycle();
        while (runConditions() && timer.time() < timeInMillis)
        {
            double newPosition = (timer.time() / timeInMillis) * HOOK_ADJUST_RELEASE_DISTANCE;
            if (newPosition > HOOK_ADJUST_RELEASE_DISTANCE) newPosition = HOOK_ADJUST_RELEASE_DISTANCE;
            setHookAdjustPosition (newPosition);
            waitNextCycle();
        }
        sleep (1000);
        waitNextCycle();
        setHookAdjustPosition(0);
        waitFullCycle();
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
}
