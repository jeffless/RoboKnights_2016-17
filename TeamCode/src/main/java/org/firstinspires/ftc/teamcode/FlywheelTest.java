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

    private static final double TOLERANCE = 0.5e-7;
    private static final double TARGET_VELOCITY = 1.1e-6;

    @Override
    public void runOpMode() throws InterruptedException
    {
        flywheelLeft = hardwareMap.dcMotor.get("flywheel1");
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheelRight = hardwareMap.dcMotor.get("flywheel2");
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PIDCalculator velocityPID = new PIDCalculator();
        VelocityCalculator flywheelVelocity = new VelocityCalculator();
        BangBangCalculator velocityBangBang = new BangBangCalculator();

        waitForStart();

        /*
        PID Example
         */
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

        /*
        Bang Bang Example
         */
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
    }

    public class PIDCalculator
    {
        private double kP, kI, kD, error, constant;
        private double lastError;
        private double integral, derivative;

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
        private double target;
        private double velocity;
        private double lowerPower, higherPower;
        private double tolerance;

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
        private long time, encoder;

        private long lastEncoder, lastTime;

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