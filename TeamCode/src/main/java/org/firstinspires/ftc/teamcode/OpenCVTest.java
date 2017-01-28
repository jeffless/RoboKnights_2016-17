package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Mat;

/**
 * Created by Jeffrey on 1/27/2017.
 */

@TeleOp(name = "OpenCV 5220", group = "Main") //change to autonomous if making this called a teleop changes something for the worse

public class OpenCVTest extends OpMode_5220
{
    @Override
    public void setup()
    {

    }

    public void initialize()
    {
        //super.initialize();

        initializeOpenCV();
    }

    public void main()
    {
        while(opModeIsActive())
        {
            if(newImage)
            {
                newImage = false;
            }
            telemetry.update();
        }

        endCamera();
    }
}
