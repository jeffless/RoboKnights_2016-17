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

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

import org.firstinspires.ftc.teamcode.OpMode_5220;

@TeleOp(name = "TeleOp 5220", group = "Main") //change to autonomous if making this called a teleop changes something for the worse
//@Disabled
public class TeleOp_5220 extends OpMode_5220 //this is a comment. It is a long comment.
{
    private static final double JOYSTICK_THRESHOLD = 0.08; //below this joysticks won't cause movement.
    private static final double SLOW_POWER = 0.10;


    private static final double SWIVEL_INCREMENT = 0.00658; //changed from 0.005

    private static final double SWIVEL_INCREMENT_TIME = 60; //in millis, every incrmeent time, it goes 0.01 counts. about 24 increments to go 180 then.
    private static final double SWIVEL_INERTIA_CORRECTION_MULTIPLIER = 0.5;

    private static final double DUMPER_INCREMENT = 0.025; //changed from 0.005
    private static final double DUMPER_INCREMENT_TIME = 60; //in millis, every incrmeent time, it goes 0.01 counts. about 24 increments to go 180 then.


    private static final double ARM_INCREMENT = 0.04;
    private static final double ARM_INCREMENT_TIME = 30; //in millis, every incrmeent time, it goes 0.01 counts. about 24 increments to go 180 then.


    private static final double HOOK_TILT_INCREMENT = 0.072;

    private static final double POLAR_CONTROL_R_THRESHOLD = 0.8; //max R value should be about 1

    private double g1Stick1Xinit;
    private double g1Stick1Yinit;

    private boolean color;

    private boolean reverseDriveOn = false;
    private boolean slowDriveOn = false;
    private boolean polarOn = false;
    private boolean resetAutomationOn = false;
    private boolean scoringAutomationOn = false;
    private boolean fieldOrient = false;

    private double throttle, direction, strafe;
    private double temp, theta;
    private double leftFront, leftBack, rightBack, rightFront;

    private static final boolean DPAD_LIFT = false;
    private static final boolean DPAD_DRIVE = true;
    private boolean dPadMode = DPAD_LIFT;

    public ProgramType getProgramType ()
    {
        return ProgramType.TELEOP;
    }

    //INITIALZATION:

    public void initialize ()
    {
        super.initialize();
        g1Stick1Xinit = gamepad1.left_stick_x;
        g1Stick1Yinit = gamepad1.left_stick_y;
        //color = ftcRCA.color;
    }

    //MAIN PROGRAM:

    public void loop5220()
    {
        //STATE VARIABLES FOR LOOP:
        Stopwatch topHatXTime = null;
        Stopwatch topHatYTime = null;
        Stopwatch dumperTime = null;
        Stopwatch hookTiltTime = null;

        double swivelMovementStart = 0.0;

        boolean strafing = false;
        boolean reverse = false;

        Gamepad prevGamepad1 = new Gamepad(); //IF USING THESE GAMEPAD OBJECTS WORKS, REPLACE ALL THE INDIVIDUAL BOOLEANS BELOW WITH THEIR PREVGAMEPAD OBJECT COUNTERPARTS.
        Gamepad prevGamepad2 = new Gamepad();

        try
        {
            prevGamepad1.copy(gamepad1);
            prevGamepad2.copy(gamepad2);
        }

        catch (RobotCoreException rce)
        {
            writeToLog("prevGamepad Copying Error.");
        }


        boolean prevTopHatUp1 = false; //maybe change these initialization if they mess something up
        boolean prevTopHatDown1 = false;
        boolean prevTopHatLeft1 = false;
        boolean prevTopHatRight1 = false;
        boolean prevTopHatUp2 = false; //maybe change these initialization if they mess something up
        boolean prevTopHatDown2 = false;
        boolean prevTopHatLeft2 = false;
        boolean prevTopHatRight2 = false;
        boolean prevX1 = false;
        boolean prevRSB1 = false;
        boolean prevB1 = false;
        boolean prevLB = false;
        boolean prevLT = false;
        boolean prevBack = false;
        boolean prevY2 = false;
        boolean prevX2 = false;
        boolean prevB2 = false;
        boolean prevA2 = false;
        boolean prevLB2 = false;
        boolean prevLT2 = false;
        boolean prevRB2 = false;
        boolean prevRT2 = false;

        while (runConditions())
        {

            if(!fieldOrient) {
                double leftPower;
                double rightPower;

                double throttle = (-(gamepad1.left_stick_y - g1Stick1Yinit));
                double direction = (gamepad1.left_stick_x - g1Stick1Xinit);

                if (reverse) {
                    throttle = -throttle;
                }

                rightPower = throttle - direction;
                leftPower = throttle + direction;

                rightPower = Range.clip(rightPower, -2, 2);
                leftPower = Range.clip(leftPower, -2, 2);

/* NOT USING DPAD FOR NOW
                if ((leftPower < 15 && leftPower> -15) && (rightPower < 15 && rightPower > -15))
                {
                    int powerVal = 15;
                    int turnPowerVal = 15;
                    int topHatVal = joystick.joy1_TopHat;
                    if (topHatVal == 0)
                    {
                        // go forward
                        leftPower = powerVal;
                        rightPower = powerVal;
                    }
                    else if (topHatVal == 4)
                    {
                        // go backward
                        leftPower = -powerVal;
                        rightPower = -powerVal;
                    }
                }
                */
                boolean powersZero = true;
                if (Math.abs(leftPower) < 0.05) {
                    leftPower = 0;
                } else powersZero = false;

                if (Math.abs(rightPower) < 0.05) {
                    rightPower = 0;
                } else powersZero = false;

                if (powersZero) {
                    double frontPower;
                    double backPower;

                    throttle = (-(gamepad1.right_stick_y - g1Stick1Yinit));
                    direction = (gamepad1.right_stick_x - g1Stick1Xinit);

                    if (reverse) {
                        throttle = -throttle;
                    }

                    backPower = throttle - direction; //SWITCH THESE AROUND IF THIS ENDS UP BEING THE WRONG WAY
                    frontPower = throttle + direction;

                    backPower = Range.clip(backPower, -2, 2);
                    frontPower = Range.clip(frontPower, -2, 2);

                    if (Math.abs(frontPower) < 0.05) {
                        frontPower = 0;
                    }

                    if (Math.abs(backPower) < 0.05) {
                        backPower = 0;
                    }

                    if (frontPower == 0 && backPower == 0) {
                        if (dPadMode == DPAD_DRIVE) {
                            if (gamepad1.dpad_up) setDrivePower(-0.21);
                            else if (gamepad1.dpad_down) setDrivePower(0.21);
                            else if (gamepad1.dpad_right) setStrafePower(-0.34);
                            else if (gamepad1.dpad_left) setStrafePower(0.34);
                            else setDrivePower(0);
                        } else {
                            setDrivePower(0);
                        }
                    } else {

                        setMotorPower(leftFrontMotor, frontPower);
                        setMotorPower(rightFrontMotor, backPower);
                        setMotorPower(leftBackMotor, backPower);
                        setMotorPower(rightBackMotor, frontPower);
                    }

                    strafing = true;
                } else {
                    strafing = false;
                }

                if (!strafing) {
                    if (reverse) {
                        leftPower = -leftPower;
                        rightPower = -rightPower;
                    }

                    setLeftDrivePower(leftPower);
                    setRightDrivePower(rightPower);
                }

                if (gamepad1.start && !prevGamepad1.start) {
                    reverse = !reverse;
                    //sleep (20);
                }
            }

            else
            {
                double throttle = (-(gamepad1.left_stick_y - g1Stick1Yinit));
                double direction = (gamepad1.left_stick_x - g1Stick1Xinit);
                double strafe = (gamepad1.right_stick_x - g1Stick1Xinit);

                theta = Math.toRadians(navX.getYaw());

                temp = throttle * Math.cos(theta) - strafe * Math.sin(theta);
                strafe = throttle * Math.sin(theta) + strafe * Math.cos(theta);
                throttle = temp;

                leftFront = throttle + direction + strafe;
                leftBack = throttle + direction - strafe;
                rightBack = throttle - direction + strafe;
                rightFront = throttle - direction - strafe;

                leftFront = Range.clip(leftFront, -2, 2);
                leftBack = Range.clip(leftBack, -2, 2);
                rightBack = Range.clip(rightBack, -2, 2);
                rightFront = Range.clip(rightFront, -2, 2);

                boolean powersZero = true;
                if (Math.abs(leftFront) < 0.05) {
                    leftFront = 0;
                }

                if (Math.abs(leftBack) < 0.05) {
                    leftBack = 0;
                }

                if (Math.abs(rightBack) < 0.05) {
                    rightBack = 0;
                }

                if (Math.abs(rightFront) < 0.05) {
                    rightFront = 0;
                }

                setMotorPower(leftFrontMotor, leftFront);
                setMotorPower(leftBackMotor, leftBack);
                setMotorPower(rightBackMotor, rightBack);
                setMotorPower(rightFrontMotor, rightFront);
            }

            if (gamepad1.back && !prevGamepad1.back)
            {
                dPadMode = !dPadMode;
            }

            if (shooterState == SHOOTER_READY)
            {
                if ((gamepad1.a && !prevGamepad1.a) || (gamepad2.a && !prevGamepad2.a))
                {
                    shootMulti();
                }

                else if (gamepad2.x)
                {
                    setMotorPower(shooterMotor, 1.0);
                    shooterChanged = true;
                }

                else if (gamepad2.y)
                {
                    setMotorPower(shooterMotor, -1.0);
                    shooterChanged = true;
                }
                else setMotorPower(shooterMotor, 0.0);
            }




/*
            if (gamepad1.x && (!prevB1)) shootMulti();
            if (gamepad1.y && (!prevGamepad1.y)) shootMulti(); //if this prevGamepad thing works then all the individual previous value variables can be eliminated.
*/
            if (gamepad1.right_bumper || gamepad2.right_bumper) setSweeperPower(1.0);
            else if (gamepad1.right_trigger > 0.7 || gamepad2.right_trigger > 0.7) setSweeperPower(-1.0);
            else setSweeperPower(0);

            if ((gamepad1.b && !prevGamepad1.b) || (gamepad2.b && !prevGamepad2.b))
                moveDoor(doorServo.getPosition() != DOOR_OPEN ? DOOR_OPEN : DOOR_CLOSED);

            if ((gamepad1.y && !prevGamepad1.y) || (gamepad2.start && !prevGamepad2.start))
                moveRackAndPinion(autoExtendServo.getPosition() != RP_IN ? RP_IN : RP_OUT);

            double liftPower = 0;

            if (dPadMode == DPAD_LIFT)
            {
                if (gamepad1.dpad_up) liftPower = 1.0;
                else if (gamepad1.dpad_down) liftPower = -1.0;
                else liftPower = 0;
            }

            if (liftPower == 0)
            {
                if (gamepad2.dpad_up) liftPower = 1.0;
                else if (gamepad2.dpad_down) liftPower = -1.0;
                else liftPower = 0;
            }

            setMotorPower(liftMotor, liftPower);

            if ((gamepad1.dpad_left && dPadMode == DPAD_LIFT) || gamepad2.dpad_left) moveLiftTiltServo(LIFT_TILT_FORWARDS);
            else if ((gamepad1.dpad_right  && dPadMode == DPAD_LIFT) || gamepad2.dpad_right) moveLiftTiltServo(LIFT_TILT_BACKWARDS );

            if ((gamepad1.left_bumper && !prevGamepad1.left_bumper) || (gamepad2.left_bumper && !prevGamepad2.left_bumper)) setShooterPreset(currentShooterPreset + 1);
            if ((gamepad1.left_trigger > 0.7 && !(prevGamepad1.left_trigger > 0.7)) || (gamepad2.left_trigger > 0.7 && !(prevGamepad2.left_trigger > 0.7))) setShooterPreset(currentShooterPreset - 1);

            //for debug
            /*
            if (gamepad2.a) colorSensorDown.enableLed(true);
            if (gamepad2.b) colorSensorDown.enableLed(false);
            if (gamepad2.y) colorSensorFront.enableLed(true);
            if (gamepad2.x) colorSensorFront.enableLed(false);
*/
            //PREVIOUS VALUE SETTINGS

            prevTopHatUp1 = gamepad1.dpad_up;
            prevTopHatDown1 = gamepad1.dpad_down;
            prevTopHatRight1 = gamepad1.dpad_right;
            prevTopHatLeft1 = gamepad1.dpad_left;
            prevTopHatUp2 = gamepad2.dpad_up;
            prevTopHatDown2 = gamepad2.dpad_down;
            prevTopHatRight2 = gamepad2.dpad_right;
            prevTopHatLeft2 = gamepad2.dpad_left;

            prevRSB1 = gamepad1.right_stick_button;
            prevX1 = gamepad1.x;
            prevB1 = gamepad1.b;
            prevLB = gamepad1.left_bumper;
            prevLT = gamepad1.left_trigger > 0.7;
            prevBack = gamepad1.back;

            prevY2 = gamepad2.y;
            prevX2 = gamepad2.x;
            prevB2 = gamepad2.b;
            prevA2 = gamepad2.a;
            prevLB2 = gamepad2.left_bumper;
            prevLT2 = gamepad2.left_trigger > 0.7;
            prevRB2 = gamepad2.right_bumper;
            prevRT2 = gamepad2.right_trigger > 0.7;

            try
            {
                prevGamepad1.copy(gamepad1);
                prevGamepad2.copy(gamepad2);
            }

            catch (RobotCoreException rce)
            {
                writeToLog("prevGamepad Copying Error.");
            }

           // telemetry.addData("9", "RSA: " + resetAutomationOn);
            waitNextCycle();


        }
    }

    public void main ()
    {

        new DebuggerDisplayLoop().start();
        //for (DcMotor dcm: driveMotors) dcm.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        /*
        waitFullCycle();
        colorSensorFront.enableLed(true);
        waitFullCycle();
        colorSensorDown.enableLed(true);
        waitFullCycle();
*.*/
        while (runConditions())
        {
            try
            {
                loop5220();
            }
            catch (Exception e)
            {
                DbgLog.error(e.getMessage());
            }
        }

        //loop5220();

    }
}