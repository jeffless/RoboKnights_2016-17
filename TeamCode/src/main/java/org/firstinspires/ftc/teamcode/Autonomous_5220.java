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

/*
For autonomous:

Move: Forwards (Positive power) is Cap Ball Lift side
Strafe: Positive Power is towards autonomous apparatus side.
Rotate: Positive Power is Counter-Clockwise Rotation.
*/

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
    public static final int END_NORMAL_RAMP = 3;
    public static final int END_FAR_RAMP = 4;
    public static final int END_FAR_CAP_BALL = 5;
    public static final int END_FAR_RAM_BALL = 6;
    public static final int END_FAR_BLOCK_BALL = 7;
    public static final int END_FAR_BLOCK_BEACON = 8;
    public static final int END_FAR_STOP = 9;
    public static final int END_AIM_CAP_BALL = 10;
    public static final int END_AIM_RAMP = 11;
    public static final int NUM_ENDS = 12;

    private double overLineTime = 950;

    private Autonomous_5220 opMode = this;

    private boolean color = BLUE; //arbitrary default
    private int startPosition = START_AIM;
    private boolean thirdBallOn = false;
    private int startWaitTime = 0; //in seconds, no need for non-integer numbers.
    private int endPath = END_AIM_CAP_BALL;

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

        private static final int NUM_SETTINGS = 5; //always make sure this is correct.

        private int currentSetting = 0;

        private String[] telemetryLines = new String[NUM_SETTINGS];


        public void run ()
        {
            for (int i = 0; i < telemetryLines.length; i++) telemetryLines[i] = "";
            telemetryLines[COLOR] = ("*Color: " + (color == RED ? "RED " : "BLUE"));
            telemetryLines[START] = ("Start Position: " + startPositionToString(startPosition));
            telemetryLines[THREE_BALLS] = ("Third Ball: " + (thirdBallOn ? "ON" : "OFF"));
            telemetryLines[WAIT] = ("Wait Time: " + startWaitTime + " seconds");
            telemetryLines[PATH] = ("End Path: " + endPathToString(endPath));

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

                telemetryLines[COLOR] = ("Color: " + (color == RED ? "RED " : "BLUE"));
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
                case START_FAR: return "FAR (Away From Corner Vortex)";
                case START_AIM: return "AIM (Far Beacon First)";
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
                case END_NORMAL_RAMP: return "NORMAL: RAMP";
                case END_FAR_RAMP: return "FAR: RAMP";
                case END_FAR_CAP_BALL: return "FAR: CAP BALL";
                case END_FAR_RAM_BALL: return "FAR: RAM BALL";
                case END_FAR_BLOCK_BALL: return "FAR: BLOCK CAP BALL";
                case END_FAR_BLOCK_BEACON: return "FAR: BLOCK BEACON";
                case END_FAR_STOP: return "FAR: STOP";
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
        //while (runConditions());
        strafeIMU(2000, 0.3);
        stopDrivetrain();
    }

    //AUTONOMOUS ONLY UTILITIES

    private void shootAutonomousBalls()
    {
        shoot.mResume();
        moveRamp(RAMP_IN);
        sleep(2800);
        moveDoor(DOOR_OPEN);
        sleep(400);
        moveRamp(RAMP_OPEN);
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
                setMotorPower(leftFrontMotor, 0.8);
                setMotorPower(rightBackMotor, 0.8);

                setMotorPower(leftBackMotor, 0.1);
                setMotorPower(rightFrontMotor, 0.1);
            }

            else if (direction == BACKWARDS)
            {
                setMotorPower(leftFrontMotor, -0.1);
                setMotorPower(rightBackMotor, -0.1);

                setMotorPower(leftBackMotor, -0.8);
                setMotorPower(rightFrontMotor, -0.8);
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
            idle();
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
                sleep(1800);
                strafe(-4);
                rotateEncoder(37.8);
                setSweeperPower(0);
            }

            else if (color == RED)
            {
                setSweeperPower(1);
                sleep(1800);
                strafe(4);
                rotateEncoder(-7.1);
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
                move(-5, 0.64, ENCODER);
                sleep(200);
                rotateEncoder(-8.4);
                sleep(200);
                move(-18.9, 0.64, ENCODER);
            }

            else if(color == RED)
            {
                move(1.4, 0.65, ENCODER);
                sleep(200);
                rotateEncoder(5.4);
                sleep(200);
                move(18.7, 0.65, ENCODER);
            }
        }

        if(thirdBallOn)
        {
            if(color == BLUE)
            {
                move(-23.1, 0.65);
            }

            else if(color == RED)
            {
                move(16, 0.65);
            }
        }
    }

    private void shootingPositionToWall ()
    {
        if(thirdBallOn)
        {
            if (color == BLUE)
            {
                //rotateEncoder(2, 0.7);
                move (-12.1, 0.86);
                rotateEncoder(-31, 0.7);
                move(1);
                strafeTime(1000, 0.7);
                stopDrivetrain();
            }

            else if (color == RED)
            {
                move(12.7, 0.86);
                rotateEncoder(20, 0.7);
                strafeTime(1000, 0.7);
                stopDrivetrain();
            }
        }

        else if(!thirdBallOn)
        {
            if (color == BLUE)
            {
                move (-11.3, 0.8);
                rotateEncoder(-28.4, 0.8);
                move(2.5);
                strafeTime(1000, 0.7);
                diagonalStrafeAgainstWall(BACKWARDS);
                sleep (450);
                stopDrivetrain();
            }

            else if (color == RED)
            {
                //rotateEncoder(-1, 0.6);
                move(13.7, 0.8);
                rotateEncoder(21, 0.7);
                strafeTime(1000, 0.7);
                stopDrivetrain();
            }
        }
    }

    private void aimStartToFarBeacon()
    {
        if (color == BLUE)
        {
            move (4, 0.6, ENCODER);
            rotateEncoder(-3.7);
            move(64);
            rotateEncoder(3.7);
            strafeTime(1100, 0.7);
            stopDrivetrain();

            /*
            move(70);
            setDrivePower(0.3);
            waitForLine();
            stopDrivetrain();
            rotateEncoder(2);
            sleep(350);
            move(3);
            strafeTime(1000, 0.7);
            stopDrivetrain();
             */

        }

        else if (color == RED)
        {
            /*
            setDrivePower(0.9);
            waitForLine();
            rotateEncoder(-9);
            strafe(17);
            */

            move (-4, 0.6, ENCODER);
            rotateEncoder(3);
            move(-73);
            rotateEncoder(-4);
            strafeTime(1200, 0.7);
            stopDrivetrain();
        }
    }

    private void findButton(boolean direction)
    {
        /* OLD:
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

        */

        if (color == RED)
        {
            diagonalStrafeAgainstWall(direction);
            while (runConditions() && colorSensorFront.red() < 2) ;
            stopDrivetrain();
        }

        else if (color == BLUE) //ADD PROTECTION BY CHECKING OPPOSITE COLOR ISN'T DETECTED?
        {
            diagonalStrafeAgainstWall(direction);
            while (runConditions() && colorSensorFront.blue() < 3) ;
            stopDrivetrain();
        }
    }

    private void pushButton()
    {
        moveRackAndPinion(RP_OUT);
        sleep(1500);
        moveRackAndPinion(RP_RETRACTED);
        sleep(600);
    }

    private void pushButtonsAlongWall ()
    {
        //findButton(FORWARDS);

        if(startPosition == START_NORMAL)
        {
            if(color == BLUE)
            {
                findButton(FORWARDS);
                move(1.0, 0.8);
            }

            if(color == RED)
            {
                findButton(BACKWARDS);
                //move(0.5, 0.8);
            }
        }

        else if(startPosition == START_AIM)
        {
            if(color == BLUE)
            {
                findButton(FORWARDS);
                move(0.8, 0.8);
            }

            if(color == RED)
            {
                findButton(BACKWARDS);
               //move(0.2, 0.8);
            }
        }

        /*if (color == BLUE)
        {
            if (colorSensorAlt.blue() > colorSensorAlt.red())
            {
                pushButton();
            }
        }

        else if (color == RED)
        {
            if (colorSensorAlt.red() > colorSensorAlt.blue())
            {
                pushButton();
            }
        } */

        pushButton();
        sleep(720);
        if(startPosition == START_NORMAL || startPosition == START_FAR) move ((color == BLUE ? 20: -23), 0.9, ENCODER);
        else if(startPosition == START_AIM) move((color == BLUE ? -26: 23), 0.9, ENCODER);

        //findButton();

        if(startPosition == START_NORMAL || startPosition == START_FAR)
        {
            if(color == BLUE)
            {
                findButton(FORWARDS);
                move(1.0, 0.8);
            }

            if(color == RED)
            {
                findButton(BACKWARDS);
                move(0.2, 0.8);
            }
        }

        else if(startPosition == START_AIM)
        {
            if(color == BLUE)
            {
                findButton(BACKWARDS);
                move(0.5, 0.8);
            }

            if(color == RED)
            {
                findButton(FORWARDS);
                move(0.2, 0.8);
            }
        }


        /*if (color == BLUE)
        {
            if (colorSensorAlt.blue() > colorSensorAlt.red())
            {
                pushButton();
            }
        }

        else if (color == RED)
        {
            if (colorSensorAlt.red() > colorSensorAlt.blue())
            {
                pushButton();
            }
        }
        */

        pushButton();

        setSweeperPower(0);
        sleep(200);
    }

    private void alignWithFarLine()
    {
        if (color == BLUE)
        {
            move(7);
            diagonalStrafeAgainstWall(BACKWARDS, SLOW);
            waitForLine();
            stopDrivetrain();
            sleep(150);
        }

        else if (color == RED)
        {
            move (-2);
            diagonalStrafeAgainstWall(FORWARDS, SLOW);
            waitForLine();
            stopDrivetrain();
            setSweeperPower(0);
            sleep(150);
        }
    }

    private void alignWIthCloseLine()
    {
        if(color == BLUE)
        {
            move(-7.2);
            diagonalStrafeAgainstWall(FORWARDS, SLOW);
            waitForLine();
            stopDrivetrain();
            sleep(150);
        }

        if(color == RED)
        {
            move(6.3);
            diagonalStrafeAgainstWall(BACKWARDS, SLOW);
            waitForLine();
            stopDrivetrain();
            sleep(150);
        }
    }

    private void farBeaconToBall()
    {
        if (color == BLUE)
        {
            strafe(-5);
            rotateEncoder(-6.3);
            move(-50);
        }

        else if (color == RED)
        {
            strafe (-7);
            rotateEncoder(5.3);
            move(50);
        }

        setSweeperPower(0.0);
    }

    public void farBeaconToRamp()
    {
        if (color == BLUE)
        {
            move(-37);
            strafe(-5);
            moveTime (1500, -0.75);
        }

        else if (color == RED)
        {
            move(37);
            strafe(-5);
            moveTime (1500, 0.75);
        }
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
            strafe(-36);
            move(15);

        }

        else if(color == RED)
        {
            strafe(-36);
            move(-15);
            sleep(3500);
            move(15);

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

    private void farStartToShootingPosition ()
    {
        if(color == BLUE)
        {
            move(-18.1, 0.82, ENCODER);
            sleep(350);
            rotateEncoder(-25.8, 0.7);
        }

        else if (color == RED)
        {
            /*move(11, 0.6);
            rotateEncoder(20.5, 0.6); */
            move(13.2, 0.82, ENCODER);
            sleep(350);
            rotateEncoder(22, 0.7);
            //move (1, 0.6);
        }
    }

    private void farShootingToRamp()
    {
        if (color == RED)
        {
            rotateEncoder(-7);
            move (40);
            rotateEncoder(9.7);
            moveTime (1000, 0.9);
        }

        else if (color == BLUE)
        {
            rotateEncoder(3.2);
            moveTime(2100, -0.7);
        }
    }

    private void farShootingToCapBall()
    {
        if (color == RED)
        {
            rotateEncoder(-18.5);
            move(30);
        }

        else if (color == BLUE)
        {
            rotateEncoder(-17.2);
            move(34);
        }
    }

    private void farShootingToRamBall()
    {
        if (color == RED)
        {
            rotateEncoder(-7);
            move (19, 0.9);
            rotateEncoder(11.8);
            //moveTime (1500, -1.0);
            setRightDrivePower(-0.45);
            setLeftDrivePower(-0.9);
            sleep(1600);
            stopDrivetrain();
            while (gameTimer.time() < 10000 && runConditions());
            move (2, 0.5);
            moveTime (800, -0.7);
            strafeTime(1000, -0.6);
        }

        else if (color == BLUE)
        {
            rotateEncoder(4);
            move(-22, 0.9);
            rotateEncoder(15);
            setRightDrivePower(-0.9);
            setLeftDrivePower(-0.45);
            sleep(1600);
            stopDrivetrain();
            while(gameTimer.time() < 10000 && runConditions());
            move(2, 0.5);
            moveTime(800, -0.7);
            strafeTime(1000, 0.6);
        }
    }

    private void farShootingToBlockBall()
    {
        if (color == RED)
        {
            /*
            rotateEncoder(-13);
            move(34);
            rotateEncoder(-16);
            while(gameTimer.time() < 10750 && runConditions());
            move(28);
            */

            strafe (3.7);
            while(gameTimer.time() < 10700 && runConditions());
            move (-25.46);
            rotateEncoder(-17.2);
            moveTime (1000, -0.6);


        }

        else if (color == BLUE)
        {
            rotateEncoder(27.5);
            strafe (-5);
            while(gameTimer.time() < 10700 && runConditions());
            move (-25.46);
            rotateEncoder(15.5);
            moveTime (1000, -0.6);
        }
    }

    private void farShootingToBlockBeacon()
    {
        if (color == RED)
        {
            /*
            rotateEncoder(-13);
            move(39);
            rotateEncoder(-10.8);

            while(gameTimer.time() < 10750 && runConditions());

            move(35.2);
            rotateEncoder(17.5);
            move(-9.8);
            strafe(11.7);
            sleep(3000);
            strafe(-17);
            */

            //EXPERIMENTAL, using cap ball:
            rotateEncoder(5);
            move (55, 0.8, ENCODER);
            rotateEncoder(14);
            //put cap ball prongs down
            move(32, 0.4, ENCODER);
            //lift prongs up a teensy bit
            move (-5, 0.4, ENCODER);
            rotateEncoder(-10);
            while (gameTimer.time() < 10000 && runConditions());
            move (40);



        }

        else if (color == BLUE)
        {
            rotateEncoder(13);
            move(-34);
            rotateEncoder(-31);

            while(gameTimer.time() < 10750 && runConditions());

            strafe(6);
            move(35);
            move(20);
            sleep(3500);
            move(-20);
        }
    }

    private void closeBeaconToShootingPosition()
    {
        if(color == BLUE)
        {
            strafe(-8);
            rotateEncoder(29, 0.5);
        }

        else if(color ==  RED)
        {
            strafe(-10);
            rotateEncoder(-35, 0.5);
        }
    }

    private void aimShootingPositionToRamp()
    {
        if(color == BLUE)
        {
            moveTime(1400, 0.7);
        }

        else if(color ==  RED)
        {
            moveTime(1400, -0.7);
        }
    }

    private void aimShootingPositionToCapBall()
    {
        if(color == BLUE)
        {
            rotateEncoder(18);
            strafe(-10);
            move(-26);
        }

        else if(color ==  RED)
        {
            rotateEncoder(5);
            strafe(12);
            move(-26);
        }
    }

    public void autonomous ()
    {
        if(startPosition == START_FAR)
        {
            farStartToShootingPosition();
            sleep(800);
            shootAutonomousBalls();

            if (endPath == END_FAR_RAMP)
            {
                farShootingToRamp();
            }

            else if (endPath == END_FAR_CAP_BALL)
            {
                farShootingToCapBall();
            }

            else if (endPath == END_FAR_RAM_BALL)
            {
                farShootingToRamBall();
            }

            else if (endPath == END_FAR_BLOCK_BALL)
            {
                farShootingToBlockBall();
            }

            else if (endPath == END_FAR_BLOCK_BEACON) //if this works, it'll be OP when both our partner and at least one of our opponents has a full autonomous like ours, which takes more than ~12 seconds to reach the far beacon
            {
                farShootingToBlockBeacon();
            }

            else if (endPath == END_FAR_STOP)
            {
                stopDrivetrain();
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
                alignWithFarLine();
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

            else if (endPath == END_NORMAL_RAMP)
            {
                //alignWithFarLine();
                farBeaconToRamp();
            }
        }

        else if(startPosition == START_AIM)
        {
            aimStartToFarBeacon();
            pushButtonsAlongWall();

            alignWIthCloseLine();

            closeBeaconToShootingPosition();
            sleep(800);
            shootAutonomousBalls();
            sleep(100);

            if(endPath == END_AIM_CAP_BALL)
            {
                aimShootingPositionToCapBall();
            }

            else if(endPath == END_AIM_RAMP)
            {
                aimShootingPositionToRamp();
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
        //voltage = new VoltageThread();

        while (gameTimer.time() < (startWaitTime * 1000) && runConditions())
        {

        }

        autonomous();
        //test();
    }
}