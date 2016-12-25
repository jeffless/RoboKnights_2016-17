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

/*
TODO:
Add option to (try to) dump blocks in low goal before climbing ramp on our side
Add option to try to drive to other side of field (make sure its allowed), climb up the ramp of our color there, and position for scoring in the medium goal. probably terminate if gyro reads too much directional change.
Add ultrasonic sensor when we add rescue beacon detection?
*/

//Hello world.

//NOTE: Do NOT put waitFullCycle in loops. Only put in between other stuff


import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "FlywheelTest", group = "Tools")
//@Disabled
public class FlywheelTest extends LinearOpMode
{
    private DcMotor flywheelLeft;
    private DcMotor flywheelRight;
    protected DcMotor sweeperMotor;

    //double kI = 0.025;
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;

    double integral;
    double derivative;

    double motorOut = 0.0;
    double fTarget = 800;
    double fVelocity = 0.0;
    double fError = 0.0;
    double fLastError = 0.0;
    double tbh = 0;

    long fEncoder = 0;
    long fVelocityTime = 0;
    long fLastEncoder = 0;
    long fLastVelocityTime = 0;

    public void runOpMode ()
    {
        flywheelLeft = hardwareMap.dcMotor.get("fl");
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flywheelRight = hardwareMap.dcMotor.get("fr");
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sweeperMotor = hardwareMap.dcMotor.get("sweeper1");
        sweeperMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        try
        {
            waitForStart();
        }
        catch (Exception e)
        {

        }
        while (opModeIsActive())
        {
            flywheelLeft.setPower(.73);
            flywheelRight.setPower(.73);

            printVelocity();

            //setFPower(calculatePID());
            telemetry.addData("4", "" + kP);
            telemetry.addData("5", "" + fVelocity);
        }
    }

    private void printVelocity()
    {
        fVelocityTime = System.nanoTime();
        fEncoder = flywheelLeft.getCurrentPosition();
        fVelocity = (fEncoder - fLastEncoder) / (fVelocityTime - fLastVelocityTime);

        telemetry.addData("5", "" + fVelocity);
        telemetry.update();
    }

    private void setFPower(double power)
    {
        flywheelLeft.setPower(power);
        flywheelRight.setPower(power);
    }

    //TBH CODE
    private double calculateTBH()
    {
        fVelocityTime = System.nanoTime();
        fEncoder = flywheelLeft.getCurrentPosition();
        fVelocity = (fEncoder - fLastEncoder) / (fVelocityTime - fLastVelocityTime);
        fError = fTarget - fVelocity;

        integral += fError;
        motorOut = integral * kI;
        Range.clip(motorOut, 0, 1);

        if(Math.signum(fError) != Math.signum(fLastError))
        {
            tbh = (motorOut + tbh) / 2;
            motorOut = tbh;
            fLastError = fError;
        }
        return motorOut;
    }

    private double calculatePID()
    {
        fVelocityTime = System.nanoTime();
        fEncoder = flywheelLeft.getCurrentPosition();
        fVelocity = (fEncoder - fLastEncoder) / (fVelocityTime - fLastVelocityTime);
        fError = fTarget - fVelocity;

        integral += fError;
        if(fError == 0)
        {
            integral = 0;
        }

        if(Math.abs(fError) > 50)
        {
            integral = 0;
        }

        derivative = fError - fLastError;

        fLastError = fError;
        fLastEncoder = fEncoder;
        fLastVelocityTime = fVelocityTime;

        motorOut = kP * fError + kI * integral + kD * derivative;

        Range.clip(motorOut, 0, 1);
        return motorOut;
    }

    public void adjustPID()
    {
        if(gamepad1.right_bumper)
        {
            kP += 0.05;
        }
    }
}