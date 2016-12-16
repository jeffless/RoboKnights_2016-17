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
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Demo 5220", group = "Main")
//@Disabled
public class Demo_Drive_5220 extends OpMode
{
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private boolean slow = false;

    private float x, y, z, w, pwr;
    private static double deadzone = 0.2;

    @Override
    public void init()
    {
        leftBack = hardwareMap.dcMotor.get("lb");
        leftFront = hardwareMap.dcMotor.get("lf");
        rightBack = hardwareMap.dcMotor.get("rb");
        rightFront = hardwareMap.dcMotor.get("rf");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        /*
        leftMotor = hardwareMap.dcMotor.get("left");
        rightMotor = hardwareMap.dcMotor.get("right");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
         */
    }

    @Override
    public void loop()
    {
        getJoyVals();
        mecanumDrive();

        //twoWheelDrive();
    }

    private void getJoyVals()
    {
        y = gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        z = gamepad1.right_stick_x;
        w = gamepad1.right_stick_y;

        if(Math.abs(x)<deadzone) x = 0;
        if(Math.abs(y)<deadzone) y = 0;
        if(Math.abs(z)<deadzone) z = 0;
        if(Math.abs(w)<deadzone) w = 0;
    }

    private void mecanumDrive()
    {
        pwr = y;
        rightFront.setPower(Range.clip(pwr - x+z, -1, 1));
        leftBack.setPower(Range.clip(pwr - x-z, -1, 1));
        leftFront.setPower(Range.clip(pwr + x-z, -1, 1));
        rightBack.setPower(Range.clip(pwr + x+z, -1, 1));
    }

    private void twoWheelDrive()
    {
        float leftY = -gamepad1.left_stick_y;
        float rightY = -gamepad1.right_stick_y;

        if(gamepad1.right_bumper)
        {
            slow = true;
        }

        if(gamepad1.left_bumper)
        {
            slow = false;
        }

        if(!slow)
        {
            leftMotor.setPower(leftY);
            rightMotor.setPower(rightY);
            telemetry.addData("Mode:" , "Fast");
        }

        else
        {
            leftMotor.setPower(leftY * 0.5);
            rightMotor.setPower(rightY * 0.5);
            telemetry.addData("Mode:" , "Slow");
        }
    }

}