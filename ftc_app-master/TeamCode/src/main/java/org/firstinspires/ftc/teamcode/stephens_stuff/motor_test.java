package org.firstinspires.ftc.teamcode.stephens_stuff;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware_Speedy;

/**
 * Created by stephenmcconnell on 1/4/18.
 */

@Autonomous(name="Motor_Test", group="Test")
@Disabled
public class motor_test extends LinearOpMode{
    Hardware_Speedy robot = new Hardware_Speedy();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("MESSAGE:", "HIT PLAY");
        telemetry.update();

        waitForStart();

        telemetry.addData("power:", ".3");
        telemetry.update();

        robot.liftMotor.setPower(.3);
        sleep(1000);
        robot.liftMotor.setPower(0);
        sleep(1000);
        robot.liftMotor.setPower(-.3);
        sleep(1000);
        robot.liftMotor.setPower(0);

        sleep(3000);

        telemetry.addData("power:", ".5");
        telemetry.update();
        robot.liftMotor.setPower(.5);
        sleep(1000);
        robot.liftMotor.setPower(0);
        sleep(1000);
        robot.liftMotor.setPower(-.5);
        sleep(1000);
        robot.liftMotor.setPower(0);
    }
}
