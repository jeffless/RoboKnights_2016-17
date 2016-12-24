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
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "FlywheelTest", group = "Tools")
//@Disabled
public class FlywheelTest extends LinearOpMode
{
    protected DcMotor flywheelLeft;
    protected DcMotor flywheelRight;
    protected DcMotor sweeperMotor;

    protected Servo doorServo;

    protected static final double DOOR_OPEN = 0.0;
    protected static final double DOOR_CLOSED = 1.0;

    public void runOpMode ()
    {
        flywheelLeft = hardwareMap.dcMotor.get("fl");
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flywheelRight = hardwareMap.dcMotor.get("fr");
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sweeperMotor = hardwareMap.dcMotor.get("sweeper1");
        sweeperMotor.setDirection(DcMotor.Direction.FORWARD);

        doorServo = hardwareMap.servo.get ("dServo");

        doorServo.setPosition(1.0);

        try
        {
            waitForStart();
        }
        catch (Exception e)
        {

        }
        while (opModeIsActive())
        {
            sweeperMotor.setPower(1.0);

            flywheelLeft.setPower(.8);
            flywheelRight.setPower(.8);

            telemetry.addData("Left Encoder", flywheelLeft.getCurrentPosition());
            telemetry.addData("Right Encoder", flywheelRight.getCurrentPosition());
            telemetry.update();
        }
    }

    public final void shoot()
    {
        flywheelLeft.setPower(0.8);
        flywheelRight.setPower(0.8);
    }

    private final class ShootThread implements Runnable
    {
        private Thread shooting;

        public void startShooting()
        {
            if(shooting == null)
            {
                shooting = new Thread(this);
                shooting.start();
            }
        }

        public void stopShooting()
        {
            if(shooting != null)
            {
                shooting.interrupt();
                shooting = null;
            }
        }

        public void run ()
        {
            while(!Thread.currentThread().isInterrupted())
            {
                shoot();
            }
        }
    }

    public void shootMulti()
    {
        ShootThread shoot = new ShootThread();
        shoot.startShooting();
    }

    public final void moveDoor(double position)
    {
        doorServo.setPosition(position);
    }

}
