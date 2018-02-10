package org.firstinspires.ftc.teamcode.Tests.Auto_Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware_Speedy;

/**
 * Created by stephenmcconnell on 1/5/18.
 */

@TeleOp(name="Auto Drive Motor Test", group="Auto Test")
@Disabled
public class Drive_Motor_Auto_test extends LinearOpMode {
    Hardware_Speedy robot = new Hardware_Speedy();

    double left = -.9 * .2;
    double right = -9 * .2;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Say", "I think, therefore I am.");
        updateTelemetry(telemetry);

        waitForStart();

        sleep(1000);
        telemetry.addData("Ramp", "Speed: Fast");
        updateTelemetry(telemetry);

        robot.leftFrontMotor.setPower(left);
        robot.rightFrontMotor.setPower(right);

        sleep(500);

        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);

        sleep(2000);
        left = -.2 * left;
        right = -.2 * right;
        telemetry.addData("Ramp", "Speed: Slow");
        updateTelemetry(telemetry);

        robot.leftFrontMotor.setPower(left);
        robot.rightFrontMotor.setPower(right);

        sleep(500);

        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
    }
}
