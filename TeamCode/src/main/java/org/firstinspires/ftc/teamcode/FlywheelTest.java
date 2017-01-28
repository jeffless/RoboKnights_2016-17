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


import android.util.Log;

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

    /*double tbhI = 300.0;
    private double kP = 0.4;
    private double kI = 0.0;
    //private double kD = 380000;
    private double kD = 0.0;

    private double integral = 0.0;
    private double derivative = 0.0;

    private double motorOut = 0.0;
    private double fTarget = 1.1e-6;
    private double fVelocity = 0.0;
    private double fError = 0.0;
    private double fLastError = 0.0;
    private double tbh = 0.0;

    private int fEncoder = 0;
    private int fLastEncoder = 0;

    private long fVelocityTime = 0;
    private long fLastVelocityTime = 0;

    private double place = 0.1;

    private boolean firstCross;

    private double tolerance = 0.5e-7;

    private double targetVoltage = 12.5;
    private double voltage;
    */

    protected static final double TARGET_VOLTAGE = 12.5;
    protected static final double TARGET_SPEED = 1.1e-6;

    protected static final double vP = 0.18;
    protected static final double sP = 0.37;
    protected static final double sI = 0.0;
    protected static final double sD = 0.0;

    protected static double voltage = 0;
    protected static double speed = 0;
    protected static double integral = 0;
    protected static double lastError;
    protected static double lastTime;
    protected static double lastEncoder;

    private static final String TAG = "MyActivity";

    public void runOpMode ()
    {
        flywheelLeft = hardwareMap.dcMotor.get("flywheel1");
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //flywheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelRight = hardwareMap.dcMotor.get("flywheel2");
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //flywheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sweeperMotor = hardwareMap.dcMotor.get("sweeper1");
        sweeperMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        doorServo = hardwareMap.servo.get("dServo");

        //firstCross = true;

        try
        {
            waitForStart();
        }
        catch (Exception e)
        {

        }

        ShootThread shoot = new ShootThread();
        while (opModeIsActive())
        {
            voltage = batteryVoltage();
            sleep(100);
            sweeperMotor.setPower(1);
            sleep(3000);
            sweeperMotor.setPower(0);
            shoot.mResume();
            sleep(1200);
            doorServo.setPosition(1.0);
            sleep(1200);
            shoot.mSuspend();
            sleep(100);
        }
    }

    /*
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
        telemetry.addData("9", "Voltage " + batteryVoltage());
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
    private void calculateTBH()
    {
        fVelocityTime = System.nanoTime();
        fEncoder = flywheelLeft.getCurrentPosition();
        fVelocity = ((double)(fEncoder - fLastEncoder) / (fVelocityTime - fLastVelocityTime));
        fError = fTarget - fVelocity;

        motorOut = motorOut  + (fError * tbhI);

        motorOut = Range.clip(motorOut, 0 ,1);

        if(Math.signum(fError) != Math.signum(fLastError))
        {
            if(firstCross)
            {
                motorOut = 0.3;
                firstCross = false;
            }

            else
            {
                motorOut = 0.5 * (motorOut + tbh);
            }

            tbh = motorOut;
        }

        fLastError = fError;

        setFPower(motorOut );
    }

    private void calculatePID()
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

        derivative = fError - fLastError;

        fLastError = fError;
        fLastEncoder = fEncoder;
        fLastVelocityTime = fVelocityTime;

        motorOut = (kP * fError) + (kI * integral) + (kD * derivative) + .8;

        motorOut = Range.clip(motorOut, 0.0, 1.0);

        telemetry.addData("1", "kP " + (kP * fError));
        telemetry.addData("2", "Error " + fError);
        telemetry.addData("3", "Time " + fVelocityTime);
        telemetry.addData("4", "Encoder " + fEncoder);
        telemetry.addData("5", "Last Encoder " + fLastEncoder);
        telemetry.addData("6", "Encoder Change " + (fEncoder - fLastEncoder));
        telemetry.addData("7", "Time Change " + (fVelocityTime - fLastVelocityTime));
        telemetry.addData("8", "Velocity " + fVelocity);
        telemetry.addData("9", "Result " + motorOut);
        telemetry.update();

        Log.wtf(TAG, String.valueOf(fError));

        setFPower(motorOut);
    }


    public void bangBang()
    {
        fVelocityTime = System.nanoTime();
        fEncoder = flywheelLeft.getCurrentPosition();
        fVelocity = (double)(fEncoder - fLastEncoder) / (fVelocityTime - fLastVelocityTime);

        if(fVelocity >= (fTarget + tolerance))
        {
            setFPower(.84);
        }

        else if(fVelocity < (fTarget - tolerance))
        {
            setFPower(.9);
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

    public void printVoltage()
    {
        double voltage = this.hardwareMap.voltageSensor.iterator().next().getVoltage();
        telemetry.addData("1", "Voltage " + voltage);
        telemetry.update();
        sleep(200);
    }


    public double batteryVoltage()
    {
        return this.hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void voltageProportional()
    {
        double kP = .15;
        double error = targetVoltage - voltage;
        motorOut = (error * kP) + .88;
        motorOut = Range.clip(motorOut, 0, 1);
        setFPower(motorOut);
    } */

    private void setFPower(double power)
    {
        flywheelLeft.setPower(power);
        flywheelRight.setPower(power);
    }

    public final void shoot()
    {
        double vError = TARGET_VOLTAGE - voltage;
        double vMotorOut = (vError * vP) + .82;
        vMotorOut = Range.clip(vMotorOut, 0, 1);

        double sError = TARGET_SPEED - flywheelSpeed();
        double derivative = sError - lastError;
        integral += sError;
        lastError = sError;

        double sMotorOut = (sP * sError) + (sI * integral) + (sD * derivative) + vMotorOut;
        sMotorOut = Range.clip(sMotorOut, 0.0, 1.0);
        setFPower(sMotorOut);
        sleep(50);
    }

    public final void stopShooting()
    {
        setFPower(0);
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

    public double batteryVoltage()
    {
        return this.hardwareMap.voltageSensor.iterator().next().getVoltage();
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
                while(!opModeHasBeenActive || opModeIsActive()) //should keep running
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
}