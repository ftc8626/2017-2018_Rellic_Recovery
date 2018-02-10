package org.firstinspires.ftc.teamcode.Tests.Teleop_Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware_Speedy;

/**
 * Created by stephenmcconnell on 1/5/18.
 */

@TeleOp(name="Teleop Drive Motor Test", group="Teleop Test")
@Disabled
public class Drive_Motor_Teleop_test extends OpMode {
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

        if(PRECISE_DRIVE == false){
            left = -.9 * gamepad1.left_stick_y;
            right = -.9 * gamepad1.right_stick_y;}
        else if(PRECISE_DRIVE == true){
            left = -.2 * (gamepad1.left_stick_y);
            right = -.2 * (gamepad1.right_stick_y);}


        //toggle precise drive
        //right trigger = sneaky
        //left trigger = speedy
        if(gamepad1.right_trigger > 0 && PRECISE_DRIVE == false)
            PRECISE_DRIVE = true;
        else if (gamepad1.left_trigger > 0 && PRECISE_DRIVE == true)
            PRECISE_DRIVE = false;

        robot.leftFrontMotor.setPower(left);
        robot.rightFrontMotor.setPower(right);

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
