package org.firstinspires.ftc.teamcode.Auto.Non_Jewel;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware_Speedy;
import org.firstinspires.ftc.teamcode.stephens_stuff.Hardware;

/**
 * Created by Cameron on 1/22/18.
 */
@Autonomous(name="Test_Gyro_Autonomous", group="Speedy")
@Disabled
public class Test_Gyro_Autonomous extends LinearOpMode {

    Hardware_Speedy robot = new Hardware_Speedy();

    private ElapsedTime runtime = new ElapsedTime();

    static final double MID_POWER =.4;
    static final double DRIVE_GAIN = .7;
    int currentHeading;
    int targetHeading;
    double headingError;
    double driveSteering;
    double leftPower;
    double rightPower;


    GyroSensor sensorGyro;  //General Gyro Sensor allows us to point to the sensor in the configuration file.
    ModernRoboticsI2cGyro gyro;  //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        sensorGyro = hardwareMap.gyroSensor.get("gyro");  //Point to the gyro in the configuration file
        gyro = (ModernRoboticsI2cGyro) sensorGyro;      //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()
        gyro.calibrate();  //Calibrate the sensor so it knows where 0 is and what still is. DO NOT MOVE SENSOR WHILE BLUE LIGHT IS SOLID

     //   gyro = hwMap.gyroSensor.get("gyro");

        telemetry.addData(">", "Calibrating Gyro");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            currentHeading = gyro.getHeading();
            if(currentHeading > 180)
                currentHeading = currentHeading - 360;

            headingError = targetHeading - currentHeading;

            driveSteering = headingError * DRIVE_GAIN;

            leftPower = MID_POWER + driveSteering;
            if (leftPower > 1)
                leftPower = 1;
            if (leftPower < 0)
                leftPower = 0;

            rightPower = MID_POWER + driveSteering;
            if (rightPower > 1)
                rightPower = 1;
            if (rightPower < 0)
                rightPower = 0;

            robot.leftBackMotor.setPower(leftPower);
            robot.leftFrontMotor.setPower(leftPower);
            robot.rightFrontMotor.setPower(rightPower);
            robot.rightBackMotor.setPower(rightPower);

            //path starts here
            while (currentHeading < 90)
                targetHeading = 90;
            while (currentHeading >= 90)
                targetHeading = 0;
        }

        robot.leftBackMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.rightBackMotor.setPower(0);

    }
}
