package org.firstinspires.ftc.teamcode.stephens_stuff;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by stephenmcconnell on 9/12/17.
 */
@Disabled
public class Hardware
{
    public DcMotor frontLeftAngle   = null;
    public DcMotor frontRightAngle  = null;
    public DcMotor backLeftAngle    = null;
    public DcMotor backRightAngle   = null;

    HardwareMap hwMap               = null;
    private ElapsedTime period      = new ElapsedTime();

    public Hardware(){


    }

    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;

        // needs names for the wheeles in the diffrent drives

        frontLeftAngle  = hwMap.get(DcMotor.class, "");
        frontRightAngle = hwMap.get(DcMotor.class, "");
        backLeftAngle   = hwMap.get(DcMotor.class, "");
        backRightAngle  = hwMap.get(DcMotor.class, "");

    }
    //private Power(){}
}
