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
    public static final int START_RAMP = 0;
    public static final int START_CORNER = 1;
    public static final int START_STRAIGHT = 2;
    public static final int NUM_STARTS = 3;

    public static final int END_BALL = 0;
    public static final int END_BLOCK = 1;
    public static final int NUM_ENDS = 2;

    public double linePATHedTime = 19500;
    private boolean linePATHed = false;

    private double overLineTime = 950;

    private Autonomous_5220 opMode = this;

    private boolean color = RED; //arbitrary default
    private int startPosition = START_RAMP;
    private int startWaitTime = 0; //in seconds, no need for non-integer numbers.
    private boolean firstBeacon = NEAR;
    private boolean secondBeaconOn = true;
    private int endPath = END_BALL;

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
        private static final int WAIT = 2;
        private static final int FIRST_BEACON = 3;
        private static final int SECOND_BEACON_ON = 4;
        private static final int PATH = 5;


        private static final int NUM_SETTINGS = 6; //always make sure this is correct.

        private int currentSetting = 0;

        private String[] telemetryLines = new String[NUM_SETTINGS];


        public void run ()
        {
            for (int i = 0; i < telemetryLines.length; i++) telemetryLines[i] = "";
            telemetryLines[COLOR] = ("Color: " + (color == RED ? "RED" : "BLUE")); //maybe add starter asterisk here. Not sure if it is neccessary.
            telemetryLines[START] = ("Start Position " + startPositionToString(startPosition));
            telemetryLines[WAIT] = ("Wait Time (in seconds): " + startWaitTime /*+ " seconds"*/);
            telemetryLines[FIRST_BEACON] = ("First Beacon Choice: " + ((firstBeacon == NEAR) ? "NEAR" : "FAR"));
            telemetryLines[SECOND_BEACON_ON] = ("Second Beacon Scoring: " + (secondBeaconOn ? "ON" : "OFF"));
            telemetryLines[PATH] = (endPathToString(endPath));
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
                telemetryLines[START] = ("Start Position " + startPositionToString(startPosition));
            }

            else if (setting == WAIT)
            {
                startWaitTime += direction;
                if (startWaitTime < 0)
                {
                    startWaitTime = 0;
                }

                telemetryLines[WAIT] = ("Wait Time(in seconds): " + startWaitTime /*+ " seconds"*/);
            }

            else if (setting == FIRST_BEACON)
            {
                firstBeacon = !firstBeacon;

                telemetryLines[FIRST_BEACON] = ("First Beacon Choice: " + ((firstBeacon == NEAR) ? "NEAR" : "FAR"));
            }

            else if (setting == SECOND_BEACON_ON)
            {
                secondBeaconOn = !secondBeaconOn;

                telemetryLines[SECOND_BEACON_ON] = ("Second Beacon Scoring: " + (secondBeaconOn ? "ON" : "OFF"));
            }

            else if (setting == PATH)
            {
                endPath = (endPath + direction) % NUM_STARTS;
                telemetryLines[PATH] = ("End Path: " + endPathToString(endPath));
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
                case START_RAMP: return "RAMP START";
                case START_CORNER: return "CORNER START";
                case START_STRAIGHT: return "STRAIGHT START";
                default: return "Error: Start Position Number.";
            }
        }

        private String endPathToString (int s)
        {
            switch (s)
            {
                case END_BALL: return "BALL";
                case END_BLOCK: return "BEACON DEFENSE";
                default: return "Error: Start Position Number.";
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
        shoot();
        moveDoor (DOOR_OPEN);
        setSweeperPower(1.0);
        sleep(600);
        setSweeperPower(0);
        moveDoor(DOOR_CLOSED);
        shoot();
        sleep(100);

        /*
        setSweeperPower(1.0);
        sleep(500);

        setSweeperPower(0.0);
        sleep(500);

        shoot();
        sleep(100);
        */
    }

    private void doubleShootAutonomousBalls()
    {
        shoot();
        sleep(250);
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

            if(firstColor == DETECT_RED && colorSensorFront.blue() >= 3)
            {
                detectBoth = true;
            }

            else if(firstColor == DETECT_BLUE && colorSensorFront.red() >= 2)
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

    private void startToShootingPosition()
    {
        //move (-7, 0.4);
        //move(-33, 0.4);

        if(color == BLUE)
        {
            move(-7, 0.4);
            rotateEncoder(-1.2);
        }

        else if(color == RED)
        {
            move(-7, 0.5);
            rotateEncoder(0.3);
        }
    }
    
    private void shootingPositionToWall ()
    {
        if (color == BLUE)
        {
            //rotateEncoder(1.2);
            //move(26);

            rotateEncoder(11);
            move (-51);
            rotateEncoder(29.3);
            strafeTime(1000, 0.7);
        }

        else if (color == RED)
        {
            //rotateEncoder(-0.3);
            //move (26);

            rotateEncoder(-12.5);
            move(-44.7);
            rotateEncoder(9.1);
            strafeTime(1000, 0.5);
        }
    }


    private void findButton()
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

    private void pushButton()
    {
        moveRackAndPinion(RP_OUT);
        sleep(1500);
        moveRackAndPinion(RP_IN);
        sleep(600);
    }

    private void pushButtonsAlongWall ()
    {
        findButton();

        if(color == RED)
        {
            move(1.8);
        }

        pushButton();
        move (color == BLUE ? 18: -14);
        findButton();

        if(color == RED)
        {
            move(1.8);
        }

        pushButton();
    }

    private void alignWithFarLine()
    {
        if (color == BLUE)
        {
            //move(12, 0.7);
            move(14, 0.7);
            diagonalStrafeAgainstWall(BACKWARDS, SLOW);
            waitForLine();
            stopDrivetrain();
            sleep(150);
        }

        else if (color == RED)
        {
            move (-8, 0.6);
            diagonalStrafeAgainstWall(FORWARDS, SLOW);
            waitForLine();
            stopDrivetrain();
            sleep(150);
        }
    }

    private void farBeaconToBall()
    {
        if (color == BLUE)
        {
            strafe(-22);
            rotateEncoder(5.6);
            move(-55);

            strafe(-19);
            //rotateEncoder(5.6);
            rotateEncoder(5.6);
            move(-14);
            sleep(1200);

            shoot();
            sleep(250);
            move(-47);
        }
        //programFinished = true;

        else if (color == RED)
        {
            strafe (-17);
            rotateEncoder(32);
            move(-44);
        }
    }

    private void farBeaconToFarShooting()
    {
        if(color == BLUE)
        {
            strafe(-19);
            rotateEncoder(5.6);
            move(-14);
            sleep(1200); //should cut down sleep time
        }

        if(color == RED) //UNTESTED
        {
            strafe (-17);
            rotateEncoder(32);
            move(-14);
            sleep(1200); //should cut down sleep time
        }
    }

    private void farShootingToBall()
    {
        move(-47);
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
    //OLD STUFF:

    private void wallToBall ()
    {
        if(color == BLUE)
        {
            strafe(-21);
            rotateEncoder(6.4);
            move(-49);
        }

        else if (color == RED)
        {
            strafe (-23);
            rotateEncoder(28);
            move (-36);
        }
    }

    public void autonomous ()
    {
        /*
        startToShootingPosition();
        shootAutonomousBalls();
        shootingPosToWall();
        wallToBeacon(firstBeacon);
        beaconToWall(firstBeacon);
        wallToBeacon(!firstBeacon);
        //beaconToWall(!firstBeacon);
        //wallToBall();
        beaconToBall();
        */

        startToShootingPosition();
        //shootAutonomousBalls();
        sleep(100);
        shootingPositionToWall();
        pushButtonsAlongWall();

        if(endPath == END_BLOCK)
        {
            farBeaconToOpponent();
        }

        else if(endPath == END_BALL)
        {
            alignWithFarLine();
            farBeaconToBall();
        }

        stopDrivetrain();
    }

    public void main ()
    {
        //ftcRCA.color = color;
        new DebuggerDisplayLoop().start();
        waitFullCycle();

        //navX.zeroYaw();
        waitFullCycle();

        colorSensorDown.enableLed(true);
        waitFullCycle();
        colorSensorDown.enableLed(true);
        waitFullCycle();

        moveRackAndPinion(RP_RELEASE);
        waitFullCycle();

        while (gameTimer.time() < (startWaitTime * 1000))
        {

        }

        linePATHedTime = 2750000; //really big number just for debug
        //test();
        autonomous();
    }
}