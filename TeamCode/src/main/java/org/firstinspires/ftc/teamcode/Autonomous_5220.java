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

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.util.Range;


//TODO:

//IDEA: Use ultrasonic sensor to detect if we are being PATHed from proceding to the second beacon, and if that is the case, immediately drive back to the first beacon, re-align, then go shoot as if second beacon was never attempted.

//Hello world.

//NOTE: Do NOT put waitFullCycle in loops. Only put in between other stuff

@TeleOp(name = "Autonomous 5220", group = "Main")
//@Disabled
public class Autonomous_5220 extends OpMode_5220
{
    public static final int START_NORMAL = 0;
    public static final int START_FAR = 1;
    public static final int START_AIM = 2;
    public static final int NUM_STARTS = 3;

    public static final int END_NORMAL_CAP_BALL = 0;
    public static final int END_NORMAL_BLOCK = 1;
    public static final int END_NORMAL_SHOOT = 2;
    public static final int END_FAR_RAMP = 3;
    public static final int END_FAR_CAP_BALL = 4;
    public static final int END_FAR_BLOCK_BALL = 5;
    public static final int END_FAR_BLOCK_BEACON = 6;
    public static final int END_AIM_CAP_BALL = 7;
    public static final int END_AIM_RAMP = 8;
    public static final int NUM_ENDS = 9;

    private double overLineTime = 950;

    private Autonomous_5220 opMode = this;

    private boolean color = RED; //arbitrary default
    private int startPosition = START_NORMAL;
    private boolean thirdBallOn = false;
    private int startWaitTime = 0; //in seconds, no need for non-integer numbers.
    private int endPath = END_NORMAL_CAP_BALL;
    private boolean runCollector = false;

    private ShootThread shoot;
    private VoltageThread voltage;

    public ProgramType getProgramType ()
    {
        return ProgramType.AUTONOMOUS;
    }

    private class ProgramKiller extends Thread
    {
        public void run()
        {
            while (opModeIsActive() && gameTimer.time() < 29.5)
            {

            }

            stopDrivetrain();
            writeToLog("Program Killer has terminated the program.");
        }
    }

    private class ConfigLoop extends Thread //uses gamepad 1 left and right bumpers to configure. Will probably change to using analog stick or top hat with up and down for changing options.
    {
        private static final int UP = 1;
        private static final int DOWN = -1;

        private static final int COLOR = 0; //these are also their telemetry lines when added to 1.
        private static final int START = 1;
        private static final int THREE_BALLS = 2;
        private static final int WAIT = 3;
        private static final int PATH = 4;
        private static final int COLLECTOR = 5;


        private static final int NUM_SETTINGS = 6; //always make sure this is correct.

        private int currentSetting = 0;

        private String[] telemetryLines = new String[NUM_SETTINGS];


        public void run ()
        {
            for (int i = 0; i < telemetryLines.length; i++) telemetryLines[i] = "";
            telemetryLines[COLOR] = ("*Color: " + (color == RED ? "RED" : "BLUE"));
            telemetryLines[START] = ("Start Position: " + startPositionToString(startPosition));
            telemetryLines[THREE_BALLS] = ("Third Ball: " + (thirdBallOn ? "ON" : "OFF"));
            telemetryLines[WAIT] = ("Wait Time: " + startWaitTime + " seconds");
            telemetryLines[PATH] = ("End Path: " + endPathToString(endPath));
            telemetryLines[COLLECTOR] = ("Run Collector: " + (runCollector ? "ON" : "OFF"));

            writeLinesToTelemetry();

            boolean prevL = false;
            boolean prevR = false;
            boolean bothPressed = false;

            while (phase < RUNNING) //ends when program is actually started. note to userL try to leave at least half a second in between config and running :D
            {
                //make sure this algorithm works properly.
                boolean l;
                boolean r;

                l = gamepad1.left_bumper;
                r = gamepad1.right_bumper;


                if (bothPressed)
                {
                    if (!l && !r)
                    {
                        nextSetting();
                        bothPressed = false;
                    }

                    continue;
                }

                if (l && r) //and of course, !bothPressed implicitly, since the program would not make it here if bothPressed were true.
                {
                    bothPressed = true;
                    prevL = false;
                    prevR = false;

                    continue;
                }

                if (l != prevL)
                {
                    if (!l) //released
                    {
                        adjustSetting(currentSetting, DOWN);
                    }

                    prevL = l;
                }

                if (r != prevR)
                {
                    if (!r) //released
                    {
                        adjustSetting(currentSetting, UP);
                    }

                    prevR = r;
                }

                if (gamepad1.y)
                {
                    colorSensorDown.enableLed(true);
                }

                if (gamepad1.x)

                {
                    colorSensorDown.enableLed(false);
                }

                telemetry.update();
                waitNextCycle();

                //sleep(10); //not sure if neccessary
            }
        }

        private void nextSetting ()
        {
            if (telemetryLines[currentSetting].charAt(0) == '*') //change to string equals comparison if this doesn't work
            {
                telemetryLines[currentSetting] = telemetryLines[currentSetting].substring(1); //remove starter asterisk for old setting
            }

            currentSetting++;
            currentSetting = currentSetting % NUM_SETTINGS;

            telemetryLines[currentSetting] = "*" + telemetryLines[currentSetting]; //add starter asterisk to new setting

            writeLinesToTelemetry();
        }

        private void adjustSetting (int setting, int direction)
        {
            if (setting == COLOR)
            {
                color = !color;

                telemetryLines[COLOR] = ("Color: " + (color == RED ? "RED" : "BLUE"));
            }

            else if (setting == START)
            {
                startPosition = (startPosition + direction) % NUM_STARTS;
                telemetryLines[START] = ("Start Position: " + startPositionToString(startPosition));
            }

            else if (setting == THREE_BALLS)
            {
                thirdBallOn = !thirdBallOn;
                telemetryLines[THREE_BALLS] = ("Third Ball: " + (thirdBallOn ? "ON" : "OFF"));
            }

            else if (setting == WAIT)
            {
                startWaitTime += direction;
                if (startWaitTime < 0)
                {
                    startWaitTime = 0;
                }

                telemetryLines[WAIT] = ("Wait Time: " + startWaitTime + " seconds");
            }

            else if (setting == PATH)
            {
                endPath = (endPath + direction) % NUM_ENDS;
                telemetryLines[PATH] = ("End Path: " + endPathToString(endPath));
            }

            else if (setting == COLLECTOR)
            {
                runCollector = !runCollector;
                telemetryLines[COLLECTOR] = ("Run Collector: " + (runCollector ? "ON" : "OFF"));
            }

            if (telemetryLines[currentSetting].charAt(0) != '*') //change to string equals comparison if this doesn't work
            {
                telemetryLines[currentSetting] = "*" + telemetryLines[currentSetting];
            }

            writeLinesToTelemetry();
        }

        private void writeLinesToTelemetry ()
        {
            for (int i = 0; i < telemetryLines.length; i++)
            {
                telemetry.addData("" + (i + 2), telemetryLines[i]);
            }
        }

        private void updateConfigDisplay() //identify current setting with asterisk before name of setting, or somewhere else.
        {
            String[] telemetryLines = new String [NUM_SETTINGS + 1]; //row zero is "Configuration", the rest are for settings. each setting's number is it's telemetry line.

        }

        private String startPositionToString (int s)
        {
            switch (s)
            {
                case START_NORMAL: return "NORMAL";
                case START_FAR: return "FAR (away from corner vortex)";
                case START_AIM: return "AIM (angled at the far beacon line)";
                default: return "Error: Start Position Number.";
            }
        }

        private String endPathToString (int s)
        {
            switch (s)
            {
                case END_NORMAL_CAP_BALL: return "NORMAL: CAP BALL";
                case END_NORMAL_BLOCK: return "NORMAL: BEACON DEFENSE";
                case END_NORMAL_SHOOT: return "NORMAL: SHOOTING";
                case END_FAR_RAMP: return "FAR: RAMP";
                case END_FAR_CAP_BALL: return "FAR: CAP BALL";
                case END_FAR_BLOCK_BALL: return "FAR: BLOCK CAP BALL";
                case END_FAR_BLOCK_BEACON: return "FAR: BLOCK BEACON";
                case END_AIM_CAP_BALL: return "AIM: CAP BALL";
                case END_AIM_RAMP: return "AIM: RAMP";
                default: return "Error: End Path Number.";
            }
        }
    }

    public void initialize () //override
    {
        super.initialize(); //do everything in the original, common initialization.
        new ConfigLoop().start(); //
        waitFullCycle();
        //colorSensorDown.enableLed(true);
    }

    public void test() //for debug, whenever we want to test something independent of the rest of the autonomous program
    {
        while (runConditions());
        stopDrivetrain();
    }

    //AUTONOMOUS ONLY UTILITIES

    private void shootAutonomousBalls()
    {
        shoot.mResume();
        sleep(1200);
        moveDoor(DOOR_OPEN);
        sleep(1200);
        moveDoor(DOOR_CLOSED);
        sleep(100);
        shoot.mSuspend();
    }


    private void diagonalStrafeAgainstWall(boolean direction, double... params)
    {
        double mode = NORMAL;

        if (params.length > 1)
        {
            return;
        }

        else if(params.length == 1)
        {
            mode = params[0];
        }

        if(mode == NORMAL)
        {
            if (direction == FORWARDS)
            {
                setMotorPower(leftFrontMotor, 0.7);
                setMotorPower(rightBackMotor, 0.7);

                setMotorPower(leftBackMotor, 0.1);
                setMotorPower(rightFrontMotor, 0.1);
            }

            else if (direction == BACKWARDS)
            {
                setMotorPower(leftFrontMotor, -0.1);
                setMotorPower(rightBackMotor, -0.1);

                setMotorPower(leftBackMotor, -0.7);
                setMotorPower(rightFrontMotor, -0.7);
            }
        }

        else if(mode == SLOW) //VALUES NEED TO BE TESTED
        {
            if (direction == FORWARDS)
            {
                setMotorPower(leftFrontMotor, 0.5);
                setMotorPower(rightBackMotor, 0.5);

                setMotorPower(leftBackMotor, 0.1);
                setMotorPower(rightFrontMotor, 0.1);
            }

            else if (direction == BACKWARDS)
            {
                setMotorPower(leftFrontMotor, -0.1);
                setMotorPower(rightBackMotor, -0.1);

                setMotorPower(leftBackMotor, -0.5);
                setMotorPower(rightFrontMotor, -0.5);
            }
        }
    }

    private void waitForLine ()
    {
        while (runConditions() && getFloorBrightness() < LINE_WHITE_THRESHOLD)
        {

        }
    }

    private void safeWaitForLine() //NEED TO TEST
    {
        double firstColor = DETECT_NONE;
        boolean detectBoth = false;

        while(runConditions() && getFloorBrightness() < LINE_WHITE_THRESHOLD)
        {
            if(colorSensorFront.blue() >= 3)
            {
                firstColor = DETECT_BLUE;
            }

            else if(colorSensorFront.red() >= 2)
            {
                firstColor = DETECT_RED;
            }

            sleep(800);

            if(firstColor == DETECT_RED && colorSensorFront.red() >= 3)
            {
                detectBoth = true;
            }

            else if(firstColor == DETECT_BLUE && colorSensorFront.blue() >= 2)
            {
                detectBoth = true;
            }

            if(detectBoth)
            {
                int beginning = gameTimer.time();

                if(gameTimer.time() > beginning + overLineTime)
                {
                    moveBackToLine();
                }
            }
        }
    }

    private void moveBackToLine()
    {
        if (color == BLUE)
        {
            diagonalStrafeAgainstWall(FORWARDS, SLOW);
        }

        else if (color == RED)
        {
            diagonalStrafeAgainstWall(BACKWARDS, SLOW);
        }
    }

    //MAIN AUTONOMOUS CODE:

    private void getThirdBall ()
    {
        if (startPosition == START_NORMAL)
        {
            if (color == BLUE)
            {
                setSweeperPower(1);
                sleep(800);
                strafe(-4);
                rotateEncoder(-13.8);
                setSweeperPower(0);
            }

            else if (color == RED)
            {
                setSweeperPower(1);
                sleep(800);
                strafe(4);
                rotateEncoder(-23.8);
                setSweeperPower(0);
            }
        }

        else if (startPosition == START_FAR)
        {
            if (color == BLUE)
            {
                setSweeperPower(1);
                sleep(800);
                strafe(4);
                rotateEncoder(-23.8);
                setSweeperPower(0);
            }

            else if (color == RED)
            {
                setSweeperPower(1);
                sleep(800);
                strafe(4);
                rotate(-13.8);
                setSweeperPower(0);
            }
        }

        else if (startPosition == START_AIM)
        {
            if (color == BLUE)
            {
               //SHOULD NEVER HAPPEN
            }

            else if (color == RED)
            {
                //SHOULD NEVER HAPPEN
            }
        }
    }

    private void startToShootingPosition()
    {
        setSweeperPower(0);

        if(!thirdBallOn)
        {
            if(color == BLUE)
            {
                move(-4, 0.4);
                rotateEncoder(-10.1);
                move(-23.5, 0.6);
            }

            else if(color == RED)
            {
                move(4, 0.4);
                rotateEncoder(6.8);
                move(18, 0.6);
            }
        }

        if(thirdBallOn)
        {
            if(color == BLUE)
            {
                move(-23.5, 0.6);
            }

            else if(color == RED)
            {
                move(18, 0.6);
            }
        }
    }

    private void shootingPositionToWall ()
    {
        if (color == BLUE)
        {
            move (-17.3, 0.5);
            rotateEncoder(-30.0, 0.7);
            move (5);
            strafeTime(800, 0.7);
            stopDrivetrain();
        }

        else if (color == RED)
        {
            move(15, 0.5);
            rotateEncoder(22.0, 0.7);
            move(-3);
            strafeTime(1000, 0.7);
            stopDrivetrain();
        }
    }

    private void startToWall ()
    {
        if (color == BLUE)
        {
            move(-7, 0.4);
            rotateEncoder(-9.2);
            move(-11, 0.4);
        }

        else if (color == RED)
        {
            rotateEncoder(-12.5);
            move(-44.7);
            rotateEncoder(9.1);
            strafeTime(1000, 0.5);
        }
    }

    private void startToFarBeacon()
    {
        if (color == BLUE)
        {
            setDrivePower(0.9);
            waitForLine();
            move(7);
            rotateEncoder(9);
            strafe(-17);
        }

        else if (color == RED)
        {
            setDrivePower(0.9);
            waitForLine();
            rotateEncoder(-9);
            strafe(17);
        }
    }

    private void findButton()
    {
        if(startPosition == START_NORMAL || startPosition == START_FAR)
        {
            if (color == BLUE)
            {
                diagonalStrafeAgainstWall(FORWARDS);
                while (runConditions() && colorSensorFront.blue() < 3) ;
                stopDrivetrain();
            }

            else if (color == RED)
            {
                diagonalStrafeAgainstWall(BACKWARDS);
                while (runConditions() && colorSensorFront.red() < 2) ;
                stopDrivetrain();
            }
        }

        else if(startPosition == START_AIM)
        {
            if (color == BLUE)
            {
                diagonalStrafeAgainstWall(BACKWARDS);
                while (runConditions() && colorSensorFront.blue() < 3) ;
                stopDrivetrain();
            }

            else if (color == RED)
            {
                diagonalStrafeAgainstWall(FORWARDS);
                while (runConditions() && colorSensorFront.red() < 2) ;
                stopDrivetrain();
            }
        }
    }

    private void pushButton()
    {
        moveRackAndPinion(RP_OUT);
        sleep(1500);
        moveRackAndPinion(RP_RELEASE);
        sleep(600);
    }

    private void pushButtonsAlongWall ()
    {
        if (runCollector && (color == RED)) setSweeperPower(0.95);

        findButton();

        if(startPosition == START_NORMAL || startPosition == START_FAR)
        {
            if(color == RED)
            {

            }

            if(color == BLUE)
            {
                move(0.9);
            }
        }

        else if(startPosition == START_AIM)
        {
            if(color == RED)
            {
                move(0.9);
            }

            if(color == BLUE)
            {

            }
        }

        pushButton();
        sleep(200);

        if(startPosition == START_NORMAL || startPosition == START_FAR) move (color == BLUE ? 18: -20);
        else if(startPosition == START_AIM) move(color == BLUE ? -18: 20);

        findButton();

        if(startPosition == START_NORMAL || startPosition == START_FAR)
        {
            if(color == RED)
            {

            }

            if(color == BLUE)
            {
                move(0.9);
            }
        }

        else if(startPosition == START_AIM)
        {
            if(color == RED)
            {
                //UNFINISHED
            }

            if(color == BLUE)
            {
                //UNFINISHED
            }
        }


        pushButton();
        setSweeperPower(0);
        sleep(200);
    }

    private void alignWithFarLine()
    {
        if (color == BLUE)
        {
            move(10);
            diagonalStrafeAgainstWall(BACKWARDS, SLOW);
            waitForLine();
            stopDrivetrain();
            sleep(150);
        }

        else if (color == RED)
        {
            if(runCollector)
            {
                setSweeperPower(0.95);
            }

            move (-10);
            diagonalStrafeAgainstWall(FORWARDS, SLOW);
            waitForLine();
            stopDrivetrain();
            setSweeperPower(0);
            sleep(150);
        }
    }

    private void farBeaconToBall()
    {
        if(runCollector)
        {
            setSweeperPower(1.0);
        }

        if (color == BLUE)
        {
            strafe(-17.3);
            rotateEncoder(-5.3);
            move(-50);
        }

        else if (color == RED)
        {
            strafe (-16.5);
            rotateEncoder(4.6);
            move(50);
        }

        setSweeperPower(0.0);
    }

    private void ballToShooting()
    {
        if(color == BLUE)
        {
            strafe(14.3);
        }

        if(color == RED)
        {
            strafe (-32.2);
            move(-3);
        }
    }

    private void farBeaconToOpponent()
    {
        if(color == BLUE)
        {
            strafe(-62);
            move(15);
        }

        else if(color == RED)
        {
            strafe(-62);
            move(-15);
        }
    }

    private void farBeaconToBallWithShooting()
    {
        if(color == BLUE)
        {
            //THIS SHOULD NEVER HAPPEN
        }

        else if(color == RED) //untested
        {
            strafe (-5);
            rotateEncoder(-14);
            strafe (20);
            shootAutonomousBalls();
            strafe (30); //hopefully knocking the cap ball off with strafing will work
        }
    }

    private void farStartToShootingPosition () //UNFINISHED
    {
        if(color == BLUE)
        {

        }

        else if (color == RED)
        {
            move(9.6);
            rotateEncoder(25);
            strafe(-5);
        }
    }

    private void closeBeaconToShootingPosition() //UNFINISHED
    {
        if(color == BLUE)
        {

        }

        else if(color ==  RED)
        {

        }
    }

    private void shootingPositionToRamp() //UNTESTED
    {
        if(color == BLUE)
        {

        }

        else if(color ==  RED)
        {

        }
    }

    private void shootingPositiongToCapBall() //UNTESTEd
    {
        if(color == BLUE)
        {

        }

        else if(color ==  RED)
        {

        }
    }

    public void autonomous ()
    {
        if(startPosition == START_FAR) //EVERYTHING HERE IS UNTESTED
        {
            farStartToShootingPosition();
            shootAutonomousBalls();

            if (endPath == END_FAR_RAMP)
            {
                if (color == RED)
                {
                    while (gameTimer.time() < 19000);
                    rotateEncoder(-8.7);
                    moveTime (4000, 0.9);
                }

                else if (color == BLUE)
                {

                }

            }

            else if (endPath == END_FAR_CAP_BALL)
            {
                if (color == RED)
                {
                    rotateEncoder(-15);
                    move (32);
                }

                else if (color == BLUE)
                {

                }

            }

            else if (endPath == END_FAR_BLOCK_BALL)
            {
                if (color == RED)
                {
                    while (gameTimer.time() < 10750);
                    move (-20);
                    strafe (12);
                }

                else if (color == BLUE)
                {

                }

            }

            else if (endPath == END_FAR_BLOCK_BEACON) //if this works, it'll be OP when both our partner and at least one of our opponents has a full autonomous like ours, which takes more than ~12 seconds to reach the far beacon
            {
                if (color == RED)
                {
                    rotateEncoder(-15);
                    move (32);
                    while (gameTimer.time() < 10750);
                    rotateEncoder(10);
                    strafeTime(5000, 1.0);
                }

                else if (color == BLUE)
                {

                }
            }

        }

        else if (startPosition == START_NORMAL)
        {
            if (thirdBallOn) getThirdBall();
            sleep(200);

            startToShootingPosition();
            sleep(800);
            shootAutonomousBalls();
            sleep(100);
            shootingPositionToWall();
            pushButtonsAlongWall();

            if (endPath == END_NORMAL_BLOCK)
            {
                farBeaconToOpponent();
            }

            else if (endPath == END_NORMAL_SHOOT)
            {
                alignWithFarLine();
                farBeaconToBallWithShooting();
            }
            else if (endPath == END_NORMAL_CAP_BALL)
            {
                alignWithFarLine();
                farBeaconToBall();
            }
        }

        else if(startPosition == START_AIM)
        {
            startToFarBeacon();
            pushButtonsAlongWall();

            closeBeaconToShootingPosition();
            sleep(800);
            shootAutonomousBalls();
            sleep(100);

            if(endPath == END_AIM_CAP_BALL)
            {
                shootingPositiongToCapBall();
            }

            else if(endPath == END_AIM_RAMP)
            {
                shootingPositionToRamp();
            }
        }

        stopDrivetrain();
        shoot.mStop();
    }

    public void main ()
    {
        new DebuggerDisplayLoop().start();
        waitFullCycle();
        //navX.zeroYaw();
        waitFullCycle();

        colorSensorDown.enableLed(true);
        waitFullCycle();
        colorSensorDown.enableLed(true);
        waitFullCycle();

        moveRackAndPinion(RP_RELEASE);
        sleep(200);
        moveDoor(DOOR_CLOSED);
        waitFullCycle();

        shoot = new ShootThread();
        voltage = new VoltageThread();

        while (gameTimer.time() < (startWaitTime * 1000))
        {

        }

        autonomous();
    }
}