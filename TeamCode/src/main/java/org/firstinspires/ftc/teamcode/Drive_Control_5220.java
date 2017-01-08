/* Copyright (c) 2014 Qualcomm Technologies Inc

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

@Autonomous(name = "Drive Control 5220", group = "Tools")
//@Disabled
public class Drive_Control_5220 extends OpMode_5220
{
    public static final int BACKWARDS = 0;
    public static final int FORWARDS = 1;
    public static final int LEFT = 2;
    public static final int RIGHT = 3;

    public static final int OPTION_CHANGE_TIME = 1000; //in millis
    private Stopwatch s;
    private int direction;

    private String directionToString (int d)
    {
        switch (d)
        {
            case BACKWARDS:
                return "BACKWARDS";
            case FORWARDS:
                return "FORWARDS";
            case LEFT:
                return "LEFT";
            case RIGHT:
                return "RIGHT";
            default:
                return null;
        }
    }

    private class DirectionChooser extends Thread
    {
        public void run ()
        {
            while (phase != RUNNING)
            {
                direction = (s.time() / OPTION_CHANGE_TIME) % 4;
                telemetry.addData("1", "Direction: " + directionToString(direction));
            }
        }
    }

    public void initialize ()
    {
        super.initialize();
        s = new Stopwatch();
        new DirectionChooser().start();
    }

	public void main ()
    {
        if (direction == BACKWARDS)
        {
            setDrivePower(-0.7);
        }

        else if (direction == FORWARDS)
        {
            setDrivePower(0.7);
        }

        else if (direction == LEFT)
        {
            setTurnPower(-1.0);
        }

        else if (direction == RIGHT)
        {
            setTurnPower(1.0);
        }

        while (opModeIsActive())
        {

        }

        stopDrivetrain();
    }
}
