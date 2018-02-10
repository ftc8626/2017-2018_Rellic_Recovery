package org.firstinspires.ftc.teamcode.Auto.Non_Jewel;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware_Speedy;

/**
 * Created by Cameron on 9/15/17.
 */

@Autonomous(name="Autonomous_Speedy_Red_Right", group="Park")
@Disabled
public class Autonomous_Speedy_Red_Right extends LinearOpMode {

    Hardware_Speedy robot = new Hardware_Speedy();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440 ;
   // static final double DRIVE_GEAR_REDUCTION = 2.0 ;
    static final double WHEEL_DIAMETER_INCHES = 4.0 ;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);
    //new variable for encoders
    static final double ENCODER_DRIVE= (COUNTS_PER_INCH) * (.8);

    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    public final String BALANCE_BOARD = "BALANCE_BOARD";
    public final String BALANCE_BOARD_LEFT = "BALANCE_BOARD_LEFT";
    public final String BALANCE_BOARD_RIGHT = "BALANCE_BOARD_RIGHT";

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftFrontMotor.getCurrentPosition(),
                robot.rightFrontMotor.getCurrentPosition());
        telemetry.update();

        waitForStart();

        //path starts here

        robot.leftTopGrabber.setPosition(.22);
        robot.rightTopGrabber.setPosition(.27);
        robot.liftMotor.setPower(.3);
        sleep(1000);
        robot.liftMotor.setPower(0);

//        encoderDrive(DRIVE_SPEED, -27, -27, 5.0);
//        encoderDrive(DRIVE_SPEED, -11, 11, 4.0);
//        encoderDrive(DRIVE_SPEED, 7.5, 7.5, 5.0);
//        encoderDrive(DRIVE_SPEED, -11, 11, 4.0);
//        encoderDrive(DRIVE_SPEED, 10, 10, 5.0);
//        robot.leftTopGrabber.setPosition(0);
//        robot.rightTopGrabber.setPosition(0);
//        sleep(500);
//        encoderDrive(DRIVE_SPEED, -2, -2, 5.0);

       encoderDrive(DRIVE_SPEED, 32, 32, 5.0);
        encoderDrive(DRIVE_SPEED, -5, 5, 4.0);
        encoderDrive(DRIVE_SPEED, 7, 7, 5.0);
        encoderDrive(DRIVE_SPEED, 5, -5, 4.0);
        encoderDrive(DRIVE_SPEED, 12, 12, 5.0);
        robot.leftTopGrabber.setPosition(0);
        robot.rightTopGrabber.setPosition(0);
        sleep(500);
        encoderDrive(DRIVE_SPEED, -2, -2, 5.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();


    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {

            newLeftTarget = robot.leftFrontMotor.getCurrentPosition() + (int) (leftInches * ENCODER_DRIVE);
            newRightTarget = robot.rightFrontMotor.getCurrentPosition() + (int) (rightInches * ENCODER_DRIVE);
            robot.leftFrontMotor.setTargetPosition(newLeftTarget);
            robot.rightFrontMotor.setTargetPosition(newRightTarget);

            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.rightFrontMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy())) {

                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftFrontMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}
