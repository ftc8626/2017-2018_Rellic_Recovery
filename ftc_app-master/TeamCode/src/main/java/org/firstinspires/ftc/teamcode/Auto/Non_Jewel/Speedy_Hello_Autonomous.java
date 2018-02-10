package org.firstinspires.ftc.teamcode.Auto.Non_Jewel;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware_Speedy;

/**
 * Created by James on 11/10/17.
 */
@Autonomous(name="Speedy_Hello_Autonomous", group="Speedy")
public class Speedy_Hello_Autonomous extends LinearOpMode {

    Hardware_Speedy robot = new Hardware_Speedy();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;
    // static final double DRIVE_GEAR_REDUCTION = 2.0 ;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);
    //new variable for encoders
    static final double ENCODER_DRIVE = (COUNTS_PER_INCH) * (.8);

    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        robot.rightJewel.setPosition(.9);
        robot.rightJewel.setPosition(.8);
        robot.rightJewel.setPosition(.7);
        robot.rightJewel.setPosition(.6);
        robot.rightJewel.setPosition(.5);
        sleep(100);
        robot.rightJewel.setPosition(.6);
        robot.rightJewel.setPosition(.7);
        robot.rightJewel.setPosition(.8);
        robot.rightJewel.setPosition(.9);
        robot.rightJewel.setPosition(1);
        sleep(100);
        robot.rightJewel.setPosition(.9);
        robot.rightJewel.setPosition(.8);
        robot.rightJewel.setPosition(.7);
        robot.rightJewel.setPosition(.6);
        robot.rightJewel.setPosition(.5);
        sleep(100);
        robot.rightJewel.setPosition(.6);
        robot.rightJewel.setPosition(.7);
        robot.rightJewel.setPosition(.8);
        robot.rightJewel.setPosition(.9);
        robot.rightJewel.setPosition(1);
        sleep(100);
        robot.rightJewel.setPosition(.9);
        robot.rightJewel.setPosition(.8);
        robot.rightJewel.setPosition(.7);
        robot.rightJewel.setPosition(.6);
        robot.rightJewel.setPosition(.5);
        sleep(100);
        robot.rightJewel.setPosition(.6);
        robot.rightJewel.setPosition(.7);
        robot.rightJewel.setPosition(.8);
        robot.rightJewel.setPosition(.9);
        robot.rightJewel.setPosition(1);
        sleep(100);
    }
}