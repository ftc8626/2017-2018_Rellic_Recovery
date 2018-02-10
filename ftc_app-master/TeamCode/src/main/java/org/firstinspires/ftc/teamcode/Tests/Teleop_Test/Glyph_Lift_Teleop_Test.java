package org.firstinspires.ftc.teamcode.Tests.Teleop_Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware_Speedy;

/**
 * Created by stephenmcconnell on 1/5/18.
 */

@TeleOp(name="Teleop Glyph Lift Test", group="Teleop Test")
@Disabled
public class Glyph_Lift_Teleop_Test extends OpMode {
    Hardware_Speedy robot = new Hardware_Speedy();

    boolean PRECISE_DRIVE = false;
    double left = 0;
    double right = 0;

    @Override
    public void init() {
        robot.init(hardwareMap);

        telemetry.addData("Say", "I think, therefore I am.");
        //       updateTelemetry(telemetry);

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        //dpad = lift
        if(gamepad1.dpad_up)
            robot.liftMotor.setPower(.3);
        else if (gamepad1.dpad_down)
            robot.liftMotor.setPower(-.3);
        else
            robot.liftMotor.setPower(0);

        if(gamepad1.guide){
            robot.leftTopGrabber.setPosition(.2);
            robot.rightTopGrabber.setPosition(.2);
            robot.rightJewel.setPosition(1);
//            robot.rightRamp.setPosition(0);
//            robot.leftRamp.setPosition(0);
        }

        telemetry.addData("left", "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        updateTelemetry(telemetry);
    }

    @Override
    public void stop() {}
}
