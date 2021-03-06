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

import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name = "FlywheelTest", group = "Tools")
//@Disabled
public class FlywheelTest extends LinearOpMode
{
    private DcMotor flywheelLeft;
    protected DcMotor flywheelRight;
    protected AnalogInput currentSensor;
    private Servo doorServo;

    private static final double TOLERANCE = 0.5e-7;
    private static final double TARGET_VELOCITY = 2.567e-6;

    protected static final double VELOCITY_P = 7.2;
    protected static final double VELOCITY_I = 0.1;
    protected static final double VELOCITY_D = 0.0;

    private static double voltage = 0.0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        flywheelLeft = hardwareMap.dcMotor.get("flywheel1");
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheelRight = hardwareMap.dcMotor.get("flywheel2");
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        currentSensor = hardwareMap.analogInput.get("current");

        doorServo = hardwareMap.servo.get("dServo");
        doorServo.setPosition(0.5);

        PIDCalculator velocityPID = new PIDCalculator();
        PIDCalculator voltagePID = new PIDCalculator();
        VelocityCalculator flywheelVelocity = new VelocityCalculator();
        BangBangCalculator velocityBangBang = new BangBangCalculator();
        VelocityCalculator leftVelocity = new VelocityCalculator();
        VelocityCalculator rightVelocity = new VelocityCalculator();

        waitForStart();
        voltage = this.hardwareMap.voltageSensor.iterator().next().getVoltage();

        while(opModeIsActive())
        {
            double input = currentSensor.getVoltage();
            double current = ((input + 30) / 60) * 1023;

            leftVelocity.setParameters(System.nanoTime(), flywheelLeft.getCurrentPosition());
            rightVelocity.setParameters(System.nanoTime(), flywheelRight.getCurrentPosition());

            double leftVelocityResult = leftVelocity.getVelocity();
            double rightVelocityResult = rightVelocity.getVelocity();

            double voltageError = 12.5 - voltage;
            voltagePID.setParameters(0.18, 0.0, 0.0, voltageError, 0.65);
            double voltageOut = voltagePID.getPID();

            /*double velocityError = (TARGET_VELOCITY - leftVelocity.getVelocity());
            velocityPID.setParameters(VELOCITY_P, VELOCITY_I, VELOCITY_D, velocityError , voltageOut);
            double motorOut = velocityPID.getPID();
            motorOut = Range.clip(motorOut, 0, 1); */

            double velocityError = (TARGET_VELOCITY - rightVelocityResult);
            Log.wtf("mai_error", String.valueOf(velocityError));

            flywheelLeft.setPower(voltageOut);
            flywheelRight.setPower(voltageOut);

            telemetry.addData("Left Velocity: ", leftVelocityResult);
            telemetry.addData("Right Velocity: ", rightVelocityResult);
            telemetry.addData("Current: ", current);
            telemetry.update();
            sleep(80);
        }

        /*while(opModeIsActive())
        {
            double currentVoltage = this.hardwareMap.voltageSensor.iterator().next().getVoltage();
            flywheelRight.setPower(0.82);
            flywheelLeft.setPower(0.82);
            flywheelVelocity.setParameters(System.nanoTime(), flywheelRight.getCurrentPosition());
            do/
            uble currentVelocity = flywheelVelocity.getVelocity();

            telemetry.addData("Velocity: ", currentVelocity);
            Log.wtf("FLYWHEELVELOCITY", String.valueOf(currentVelocity) + "," + String.valueOf(currentVoltage));
            sleep(10000);

            flywheelLeft.setPower(0.0);
            flywheelRight.setPower(0.0);
            sleep(5000);
        } */

        /*
        PID Example
         */
        /*
        while(opModeIsActive())
        {
            flywheelVelocity.setParameters(System.nanoTime(), flywheelRight.getCurrentPosition());
            double currentVelocity = flywheelVelocity.getVelocity();
            double currentError = TARGET_VELOCITY - currentVelocity;

            velocityPID.setParameters(0.37, 0.1, 0.0, currentError, 0.85);
            double motorOut = velocityPID.getPID();

            motorOut = Range.clip(motorOut, 0, 1);
            flywheelLeft.setPower(motorOut);
            flywheelRight.setPower(motorOut);
        }
        */

        /*
        Bang Bang Example
         */
        /*
        while(opModeIsActive())
        {
            flywheelVelocity.setParameters(System.nanoTime(), flywheelRight.getCurrentPosition());
            double currentVelocity = flywheelVelocity.getVelocity();

            velocityBangBang.setParameters(currentVelocity, TARGET_VELOCITY, 0.84, 0.9, TOLERANCE);
            double motorOut = velocityBangBang.getBangBang();

            motorOut = Range.clip(motorOut, 0, 1);
            flywheelLeft.setPower(motorOut);
            flywheelRight.setPower(motorOut);
        }
        */
    }

    public class PIDCalculator
    {
        private double kP, kI, kD, error, constant = 0;
        private double lastError = 0;
        private double integral, derivative = 0;

        public void setParameters(double kP, double kI, double kD, double error, double constant)
        {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.error = error;
            this.constant = constant;
        }

        public double getPID()
        {
            derivative = error - lastError;
            integral += error;
            lastError = error;
            return (kP * error) + (kI * integral) + (kD * derivative) + constant;
        }
    }

    public class BangBangCalculator
    {
        private double target = 0;
        private double velocity = 0;
        private double lowerPower, higherPower = 0;
        private double tolerance = 0;

        public void setParameters(double target, double velocity, double lowerPower, double higherPower, double tolerance)
        {
            this.target = target;
            this.velocity = velocity;
            this.lowerPower = lowerPower;
            this.higherPower = higherPower;
            this.tolerance = tolerance;
        }

        public double getBangBang()
        {
            if(velocity >= (target + tolerance))
            {
                return lowerPower;
            }

            else
            {
                return higherPower;
            }
        }
    }

    public class VelocityCalculator
    {
        private long time, encoder = 0;

        private long lastEncoder, lastTime = 0;

        public void setParameters(long time, long encoder)
        {
            this.time = time;
            this.encoder = encoder;
        }

        public double getVelocity()
        {
            double velocity = (double) (encoder - lastEncoder) / (time - lastTime);

            lastEncoder = encoder;
            lastTime = time;

            return velocity;
        }
    }
}