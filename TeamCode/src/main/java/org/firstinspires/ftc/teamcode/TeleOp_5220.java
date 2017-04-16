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

@TeleOp(name = "TeleOp 5220", group = "Main") //change to autonomous if making this called a teleop changes something for the worse
//@Disabled
public class TeleOp_5220 extends OpMode_5220 //this is a comment. It is a long comment.
{
    private double g1Stick1Xinit;
    private double g1Stick1Yinit;

    private boolean color;

    private boolean reverseDriveOn = false;
    private boolean slowDriveOn = false;
    private boolean fieldOrient = false;

    private int teleOpShootingState = SHOOTER_OFF;
    private static final int SHOOTER_OFF = 0;
    private static final int SHOOTER_ON = 1;
    private static final int SHOOTER_FIRING = 2;
    private boolean fireWhenReady = false;

    private static final boolean DPAD_LIFT = false;
    private static final boolean DPAD_DRIVE = true;
    private boolean dPadMode = DPAD_LIFT;

    private ShootThread shoot;
    private VoltageThread voltage;
    private DPadThread dPad;

    public ProgramType getProgramType ()
    {
        return ProgramType.TELEOP;
    }

    //INITIALIZATION:

    public void initialize ()
    {
        super.initialize();
        g1Stick1Xinit = gamepad1.left_stick_x;
        g1Stick1Yinit = gamepad1.left_stick_y;
        moveDoor(DOOR_CLOSED);
    }

    //MAIN PROGRAM:

    public void loop5220()
    {
        //STATE VARIABLES FOR LOOP:
        Stopwatch shooterTimer = null;
        Stopwatch delayTimer = null;

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

        while (runConditions())
        {
            if(!fieldOrient) {
                double leftPower;
                double rightPower;

                double throttle = ((gamepad1.left_stick_y - g1Stick1Yinit));
                double direction = (-(gamepad1.left_stick_x - g1Stick1Xinit));

                if (reverse) {
                    direction = -direction;
                }

                if(slowDriveOn)
                {
                    throttle = SLOW_P * throttle;
                    direction = SLOW_P * direction;
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

                    throttle = ((gamepad1.right_stick_y - g1Stick1Yinit));
                    direction = (-(gamepad1.right_stick_x - g1Stick1Xinit));

                    /*if (reverse) {
                        direction = -direction;
                    }*/

                    if(slowDriveOn)
                    {
                        throttle = SLOW_P * throttle;
                        direction = SLOW_P * direction;
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
                            /*if (gamepad1.dpad_up) setDrivePower(-0.34);
                            else if (gamepad1.dpad_down) setDrivePower(0.34);
                            else if (gamepad1.dpad_right) setStrafePower(0.40);
                            else if (gamepad1.dpad_left) setStrafePower(-0.40);
                            */
                            /*
                            if (gamepad1.dpad_up) setDrivePower(-0.21);
                            else if (gamepad1.dpad_down) setDrivePower(0.21);
                            else if (gamepad1.dpad_right) setStrafePower(0.34);
                            else if (gamepad1.dpad_left) setStrafePower(-0.34);
                            else setDrivePower(0);
                            */

                            /*
                            if(dPad == null)
                            {
                                if(gamepad1.dpad_up) dPad = new DPadThread(DPAD_FORWARDS);
                                else if(gamepad1.dpad_down) dPad = new DPadThread(DPAD_BACKWARDS);
                                else if(gamepad1.dpad_right) dPad = new DPadThread(DPAD_STRAFE_RIGHT);
                                else if(gamepad1.dpad_left) dPad = new DPadThread(DPAD_STRAFE_LEFT);
                                else stopDrivetrain();
                            }

                            else
                            {
                                if(!gamepad1.dpad_up && prevGamepad1.dpad_up)
                                {
                                    dPad.stopMoving();
                                    dPad = null;
                                }
                                else if(!gamepad1.dpad_down && prevGamepad1.dpad_down)
                                {
                                    dPad.stopMoving();
                                    dPad = null;
                                }
                                else if(!gamepad1.dpad_right && prevGamepad1.dpad_right)
                                {
                                    dPad.stopMoving();
                                    dPad = null;
                                }
                                else if(!gamepad1.dpad_left && prevGamepad1.dpad_left)
                                {
                                    dPad.stopMoving();
                                    dPad = null;
                                }

                                else stopDrivetrain();
                            }
                            */
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
                double throttle = ((gamepad1.left_stick_y - g1Stick1Yinit));
                double strafe = (-(gamepad1.left_stick_x - g1Stick1Xinit));
                double direction  = (-(gamepad1.right_stick_x - g1Stick1Xinit));

                double heading = normalizeIMUHeading();

                double cosHeading = Math.cos(Math.toRadians(heading));
                double sinHeading = Math.sin(Math.toRadians(heading));

                double xOut = strafe * cosHeading - throttle * sinHeading;
                double yOut = strafe * sinHeading + throttle * cosHeading;

                double leftFront = yOut + xOut + direction;
                double leftBack = yOut - xOut + direction;
                double rightBack = yOut + xOut - direction;
                double rightFront = yOut - xOut - direction;

                leftFront = Range.clip(leftFront, -1, 1);
                leftBack = Range.clip(leftBack, -1, 1);
                rightBack = Range.clip(rightBack, -1, 1);
                rightFront = Range.clip(rightFront, -1, 1);

                setMotorPower(leftFrontMotor, leftFront);
                setMotorPower(leftBackMotor, leftBack);
                setMotorPower(rightBackMotor, rightBack);
                setMotorPower(rightFrontMotor, rightFront);
            }

            if (gamepad1.left_bumper && !prevGamepad1.left_bumper)
            {
                slowDriveOn = !slowDriveOn;
            }

            if(gamepad1.guide && !prevGamepad1.guide)
            {
                fieldOrient = !fieldOrient;
            }

            if ((gamepad1.a && !prevGamepad1.a) || (gamepad2.a && !prevGamepad2.a) || fireWhenReady)
            {
                if (teleOpShootingState == SHOOTER_OFF)
                {
                    moveRamp(RAMP_IN);
                    shoot.mResume();
                    shooterTimer = new Stopwatch();
                    teleOpShootingState = SHOOTER_ON;
                }

                else if (teleOpShootingState == SHOOTER_ON) //THE ELSE HERE IS IMPORTANT
                {
                    if (shooterTimer.time() < 2800)
                    {
                        fireWhenReady = true;
                    }

                    else
                    {
                        moveDoor(DOOR_OPEN);
                        fireWhenReady = false;
                        shooterTimer = null;
                        delayTimer = new Stopwatch();
                        teleOpShootingState = SHOOTER_FIRING;
                    }
                }

                else if (teleOpShootingState == SHOOTER_FIRING)
                {
                    shoot.mSuspend();
                    delayTimer = null;
                    moveDoor(DOOR_CLOSED);
                    moveRamp(RAMP_OPEN);
                    teleOpShootingState = SHOOTER_OFF;
                }
            }

            if(teleOpShootingState == SHOOTER_FIRING)
            {
                if(delayTimer.time() > 400)
                {
                    moveRamp(RAMP_OPEN);
                }
            }

            if(gamepad2.left_bumper)
            {
                shoot.mSuspend();
                delayTimer = null;
                moveDoor(DOOR_CLOSED);
                moveRamp(RAMP_OPEN);
                teleOpShootingState = SHOOTER_OFF;
            }

            if (gamepad1.right_bumper || gamepad2.right_bumper)
            {
                setSweeperPower(1.0);
                moveDoor(DOOR_CLOSED);
            }
            else if (gamepad1.right_trigger > 0.7 || gamepad2.right_trigger > 0.7)
            {
                setSweeperPower(-1.0);
                moveDoor(DOOR_CLOSED);
            }
            else
            {
                setSweeperPower(0);
            }

            if ((gamepad1.b && !prevGamepad1.b) || (gamepad2.b && !prevGamepad2.b))
                moveDoor(doorServo.getPosition() != DOOR_OPEN ? DOOR_OPEN : DOOR_CLOSED);

            if ((gamepad1.x && !prevGamepad1.x) || (gamepad2.x && !prevGamepad2.x))
                moveRamp(rampServo.getPosition() != RAMP_OPEN ? RAMP_OPEN : RAMP_IN);

            if ((gamepad1.y && !prevGamepad1.y) || (gamepad2.y && !prevGamepad2.y))
                moveRackAndPinion(autoExtendServo.getPosition() != RP_IN ? RP_IN : RP_OUT);

            double liftPower = 0;

            if (dPadMode == DPAD_LIFT)
            {
                if (gamepad1.dpad_up) liftPower = 1.0;
                else if (gamepad1.dpad_down) liftPower = -1.0;
                else liftPower = 0;

                if ((gamepad1.dpad_right && !prevGamepad1.dpad_right))
                {
                    if(hookServo.getPosition() == HOOK_IN)
                    {
                        moveLiftClamp(LIFT_CLAMP_RELEASE);
                        moveHook(HOOK_RELEASE);
                        moveBallClamp(BALL_CLAMP_RELEASE);
                    }

                    else if(hookServo.getPosition() == HOOK_RELEASE)
                    {
                        moveLiftClamp(LIFT_CLAMP_IN);
                        moveHook(HOOK_IN);
                        moveBallClamp(BALL_CLAMP_IN);
                    }
                }

                if ((gamepad1.dpad_left && !prevGamepad1.dpad_left) || ((gamepad1.left_trigger > 0.7) && !(prevGamepad1.left_trigger > 0.7)))
                {
                    if(ballClampServo.getPosition() == BALL_CLAMP_IN || ballClampServo.getPosition() == BALL_CLAMP_CLAMP)
                    {
                        moveBallClamp(BALL_CLAMP_RELEASE);
                    }

                    else if(ballClampServo.getPosition() == BALL_CLAMP_RELEASE)
                    {
                        moveBallClamp(BALL_CLAMP_CLAMP);
                    }
                }
            }

            if (liftPower == 0)
            {
                if (gamepad2.dpad_up) liftPower = 1.0;
                else if (gamepad2.dpad_down) liftPower = -1.0;
                else liftPower = 0;

                if ((gamepad2.dpad_right && !prevGamepad2.dpad_right))
                {
                    if(hookServo.getPosition() == HOOK_IN)
                    {
                        moveLiftClamp(LIFT_CLAMP_RELEASE);
                        moveHook(HOOK_RELEASE);
                        moveBallClamp(BALL_CLAMP_RELEASE);
                    }

                    else if(hookServo.getPosition() == HOOK_RELEASE)
                    {
                        moveLiftClamp(LIFT_CLAMP_IN);
                        moveHook(HOOK_IN);
                        moveBallClamp(BALL_CLAMP_IN);
                    }
                }

                if ((gamepad2.dpad_left && !prevGamepad2.dpad_left))
                {
                    if(ballClampServo.getPosition() == BALL_CLAMP_IN || ballClampServo.getPosition() == BALL_CLAMP_CLAMP)
                    {
                        moveBallClamp(BALL_CLAMP_RELEASE);
                    }

                    else if(ballClampServo.getPosition() == BALL_CLAMP_RELEASE)
                    {
                        moveBallClamp(BALL_CLAMP_CLAMP);
                    }
                }
            }

            setMotorPower(liftMotor, liftPower);
            //PREVIOUS VALUE SETTINGS

            try
            {
                prevGamepad1.copy(gamepad1);
                prevGamepad2.copy(gamepad2);
            }

            catch (RobotCoreException rce)
            {
                writeToLog("prevGamepad Copying Error.");
            }

            waitNextCycle();
        }
    }

    public void main ()
    {

        new DebuggerDisplayLoop().start();
        /*
        waitFullCycle();
        colorSensorFront.enableLed(true);
        waitFullCycle();
        colorSensorDown.enableLed(true);
        waitFullCycle();
        */

        shoot = new ShootThread();
        voltage = new VoltageThread();

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