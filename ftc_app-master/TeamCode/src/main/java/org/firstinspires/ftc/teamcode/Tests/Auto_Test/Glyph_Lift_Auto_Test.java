package org.firstinspires.ftc.teamcode.Tests.Auto_Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware_Speedy;

/**
 * Created by stephenmcconnell on 1/5/18.
 */

@Autonomous(name="Auto Glyph Lift Test", group="Auto Test")
@Disabled
public class Glyph_Lift_Auto_Test extends LinearOpMode {
    Hardware_Speedy robot = new Hardware_Speedy();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Say", "I think, therefore I am.");
        //       updateTelemetry(telemetry);

        waitForStart();

        //lift
        sleep(1000);
            robot.liftMotor.setPower(.3);
        sleep(1000);
            robot.liftMotor.setPower(0);
        
        sleep(2000);

        //lower
            robot.liftMotor.setPower(-.3);
        sleep(1000);
            robot.liftMotor.setPower(0);
    }
}
