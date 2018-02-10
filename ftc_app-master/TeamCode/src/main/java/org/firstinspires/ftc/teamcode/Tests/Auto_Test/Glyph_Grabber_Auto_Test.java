package org.firstinspires.ftc.teamcode.Tests.Auto_Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware_Speedy;

import static java.lang.Thread.sleep;

/**
 * Created by stephenmcconnell on 1/5/18.
 */

@Autonomous(name="Auto Glyph Grabber Test", group="Auto Test")
@Disabled
public class Glyph_Grabber_Auto_Test extends LinearOpMode {
    Hardware_Speedy robot = new Hardware_Speedy();

    double left = 0;
    double right = 0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Say", "I think, therefore I am.");
        updateTelemetry(telemetry);

        waitForStart();

        while(opModeIsActive()) {
            sleep(3000);
            robot.rightTopGrabber.setPosition(.27);
            robot.leftTopGrabber.setPosition(.22);

            sleep(2000);

            robot.rightTopGrabber.setPosition(0);
            robot.leftTopGrabber.setPosition(0);
        }

    }
}
