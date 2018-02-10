package org.firstinspires.ftc.teamcode.stephens_stuff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by kpingel on 10/13/17.
 */
    public class identifacation extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

   public void runOpMode() {


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AbmwI+L/////AAAAGaPUfjhXhEwGoXG0NiAuGEIJGYX4U81eB8tIXkWD6i6VJhkRtgv6AUcKi59jb4/RabdihUpiMzPx1CaCgO3g6K9HQ7g7mwpBlSCailOOI+y3/RP6e8wkhYMqqpIDHHWMYrLx3ajQjTAev2E/18m/Zz92WI8FBUAnieicbBOpM3sIe5GyAjM8RkhIOVrQoR1ziUpUMxOmM4e/PpF1XwXtntkJ2QfLiQAHFmDTkNJqkAVykgimYtGm4dB1eHYN7fkZ1IdG2Qx/vw6SCPNu+scDC2zcJTFXuIaEp1Ex7uzA3IA3nHoWmpHlf4KNOfg3nVFiJT+/1kYquP/zdQe1634coGlAhsr4BliL5Y+2sbDbQTuo";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);


        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);


            telemetry.addData("VuMark", "%s visible", vuMark);
    }
}
