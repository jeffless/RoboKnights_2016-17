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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


//TODO:

//IDEA: Use ultrasonic sensor to detect if we are being blocked from proceding to the second beacon, and if that is the case, immediately drive back to the first beacon, re-align, then go shoot as if second beacon was never attempted.

//Hello world.

//NOTE: Do NOT put waitFullCycle in loops. Only put in between other stuff

@Autonomous(name = "Autonomous 5220 Old", group = "Main")
@Disabled
public class Autonomous_5220_Old extends OpMode_5220_Old
{
    public static final int START_RAMP = 0;
    public static final int START_CORNER = 1;
    public static final int START_STRAIGHT = 2;
    public static final int NUM_STARTS = 3;

    public double lineBlockedTime = 19500;
    private boolean lineBlocked = false;

    private Autonomous_5220_Old opMode = this;

    private boolean color = RED; //arbitrary default
    private int startPosition = START_RAMP;
    private int startWaitTime = 0; //in seconds, no need for non-integer numbers.
    private boolean firstBeacon = NEAR;
    private boolean secondBeaconOn = true;

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


        private static final int NUM_SETTINGS = 5; //always make sure this is correct.

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

                waitFullCycle();

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
    }

    public void initialize () //override
    {
        super.initialize(); //do everything in the original, common initialization.
        new ConfigLoop().start(); //
        waitFullCycle();
        colorSensorDown.enableLed(true);
    }

    public void test() //for debug, whenever we want to test something independent of the rest of the autonomous program
    {
        telemetry.addData("1", "Started moving.");
    }

    public void autonomous ()
    {
        startToLine();

        followLineUntilTouch();
        scoreRescueBeacon();

        if (secondBeaconOn)
        {
            moveToOtherBeacon();
            followLineUntilTouch();
            scoreRescueBeacon();
        }

        beaconToShootingPosition();
        shootAutonomousBalls();
        sleep(100);
    }

    private void startToLine ()
    {
        boolean c = color;

        if (c == BLUE)
        {
            if (startPosition == START_RAMP)
            {
                if(firstBeacon == NEAR)
                {

                }

                else if (firstBeacon == FAR)
                {

                }

            }

            else if (startPosition == START_CORNER)
            {
                if(firstBeacon == NEAR)
                {

                }

                else if (firstBeacon == FAR)
                {

                }
            }

            else if (startPosition == START_STRAIGHT)
            {
                //Don't bother with this option for now
            }
        }

        else if (c == RED)
        {

            if (startPosition == START_RAMP)
            {
                if(firstBeacon == NEAR)
                {

                }

                else if (firstBeacon == FAR)
                {

                }
            }

            else if (startPosition == START_CORNER)
            {
                if(firstBeacon == NEAR)
                {

                }

                else if (firstBeacon == FAR)
                {

                }
            }

            else if (startPosition == START_STRAIGHT) //untested
            {
                //Don't bother with this option for now
            }
        }

        sleep(100);
    }

    private void moveToOtherBeacon ()
    {
        //go right if RED (false) and NEAR (true) OR BLUE (true) and FAR (false)
        //go left if RED (false) and FAR (false) OR BLUE (true) and NEAR (true)
        //Need XOR between color and firstBeacon. If XOR is true then go right, otherwise go left

        if (xor(color, firstBeacon)) //GO LEFT
        {

        }

        else //GO RIGHT
        {

        }
    }

    private void beaconToShootingPosition ()
    {
        boolean currentBeacon = (secondBeaconOn ? !firstBeacon : firstBeacon);

        if (color == RED)
        {
            if (currentBeacon == NEAR)
            {

            }

            else if (currentBeacon == FAR)
            {

            }
        }

        else if (color == BLUE)
        {
            if (currentBeacon == NEAR)
            {

            }

            else if (currentBeacon == FAR)
            {

            }
        }
    }

    private void shootAutonomousBalls()
    {
        shoot();
        shoot();
        shoot();
    }

    private void waitForLine ()
    {
        while (runConditions() && getFloorBrightness() < LINE_WHITE_THRESHOLD)
        {

        }
    }

    private void driveToLine (double power)
    {
        if (!runConditions()) return;
        setDrivePower(power);
        waitForLine();
        stopDrivetrain();
    }

    private void turnToLine (double power)
    {
        if (!runConditions()) return;
        setTurnPower(power);
        waitForLine();
        stopDrivetrain();
        sleep(50);
    }

    private void strafeToLine (double power)
    {
        if (!runConditions()) return;
        setStrafePower(power);
        waitForLine();
        stopDrivetrain();
        sleep(50);
    }

    private void turnAcrossLine (double power)
    {
        if (!runConditions()) return;
        setTurnPower(power);
        waitForLine();
        while (runConditions() && getFloorBrightness() >= LINE_WHITE_THRESHOLD)
        {

        }
        stopDrivetrain();
        sleep(50);
    }

    private void followLineUntilTouch ()
    {
        if (!runConditions()) return;

        while (runConditions() && touchSensorFront.getValue() < 0.04)
        {
            //INSERT NEW BETTER LINE FOLLOWING ALGORITHM USING MECANUM WHEELS HERE
        }

        sleep(50);
        stopDrivetrain();
        moveTime(200, 0.32);
        stopDrivetrain();
    }

    private void scoreRescueBeacon ()
    {
        if (!runConditions()) return;
        Boolean rbcWrapper = getRescueBeaconColor();
        if (rbcWrapper == null) return;
        boolean rescueBeaconColor = Boolean.valueOf(rbcWrapper); //make sure this works properly
        if (rescueBeaconColor == color)
        {

        }

        else
        {

        }
    }

    public Boolean getRescueBeaconColor () //experimental for the time being
    {
        int red = colorSensorFront.red();
        int blue = colorSensorFront.blue();
        int green = colorSensorFront.green();

        if (red > blue) return RED;
        else if (blue > red) return BLUE;
        else return null;
    }

    public void main ()
    {
        //new ProgramKiller().start(); //PROGRAM KILLER MESSES UP AUTONOMOUS.
        //ftcRCA.color = color;
        new DebuggerDisplayLoop().start();
        waitFullCycle();

        navX.zeroYaw();
        waitFullCycle();
/*
        lineBlockedTime = lineBlockedTime + startWaitTime; //intentionally disabling this stall detection for now
        if (startPosition == START_CORNER) lineBlockedTime = lineBlockedTime + 12; //tiny value is intentional, blue is about as fast as red.
*/
        colorSensorDown.enableLed(true);
        waitFullCycle();
        colorSensorDown.enableLed(true);
        waitFullCycle();

        while (gameTimer.time() < (startWaitTime * 1000))
        {

        }

        lineBlockedTime = 2750000; //really big number just for debug
       // test();
        autonomous();
    }
}
