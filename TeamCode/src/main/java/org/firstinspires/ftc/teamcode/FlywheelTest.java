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
    protected DcMotor flywheelLeft;
    protected DcMotor flywheelRight;
    protected DcMotor sweeperMotor;

    protected Servo doorServo;

    //double kI = 0.025;
    private double kP = 0.12;
    private double kI = 0.0;
    private double kD = 0.0;

    private double integral = 0.0;
    private double derivative = 0.0;

    private double motorOut = 0.0;
    private double fTarget = 7.0e-7;
    private double fVelocity = 0.0;
    private double fError = 0.0;
    private double fLastError = 0.0;
    private double tbh = 0.0;

    private int fEncoder = 0;
    private int fLastEncoder = 0;

    private long fVelocityTime = 0;
    private long fLastVelocityTime = 0;

    private double place = 0.1;

    public void runOpMode ()
    {
        flywheelLeft = hardwareMap.dcMotor.get("fl");
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //flywheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flywheelRight = hardwareMap.dcMotor.get("fr");
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //flywheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sweeperMotor = hardwareMap.dcMotor.get("sweeper1");
        sweeperMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        doorServo = hardwareMap.servo.get("dServo");

        try
        {
            waitForStart();
        }
        catch (Exception e)
        {

        }
        while (opModeIsActive())
        {
            doorServo.setPosition(0.9);
            bangBang();

            doorServo.setPosition(0.60); //down position
            sleep(2000);
        }
    }

    private void printVelocity()
    {
        fVelocityTime = System.nanoTime();
        fEncoder = flywheelRight.getCurrentPosition();
        fVelocity = (double)(fEncoder - fLastEncoder) / (fVelocityTime - fLastVelocityTime);

        telemetry.addData("3", "Time " + fVelocityTime);
        telemetry.addData("4", "Encoder " + fEncoder);
        telemetry.addData("5", "Last Encoder " + fLastEncoder);
        telemetry.addData("6", "Encoder Change " + (fEncoder - fLastEncoder));
        telemetry.addData("7", "Time Change " + (fVelocityTime - fLastVelocityTime));
        telemetry.addData("8", "Velocity " + fVelocity);
        telemetry.update();

        fLastEncoder = fEncoder;
        fLastVelocityTime = fVelocityTime;
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
        fVelocity = ((double)(fEncoder - fLastEncoder) / (fVelocityTime - fLastVelocityTime));
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
        fVelocity = (double)(fEncoder - fLastEncoder) / (fVelocityTime - fLastVelocityTime);
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

        motorOut = (kP * fError) + (kI * integral) + (kD * derivative);

        Range.clip(motorOut, 0.0, 1.0);
        return motorOut;
    }


    public void bangBang()
    {
        fVelocityTime = System.nanoTime();
        fEncoder = flywheelLeft.getCurrentPosition();
        fVelocity = (double)(fEncoder - fLastEncoder) / (fVelocityTime - fLastVelocityTime);

        if(fVelocity >= fTarget)
        {
            setFPower(.36);
        }

        else if(fVelocity < fTarget)
        {
            setFPower(0.81);
        }

        fLastEncoder = fEncoder;
        fLastVelocityTime = fVelocityTime;
    }

    public void adjustPID()
    {
        if(gamepad1.right_bumper)
        {
            kP += (1.0 * place);
        }

        if(gamepad1.left_bumper)
        {
            kP -= (1.0 * place);
        }

        if(gamepad1.right_trigger > 0.7)
        {
            place *= 10.0;
        }

        if(gamepad1.left_trigger > 0.7)
        {
            place *= 0.1;
        }
    }
}