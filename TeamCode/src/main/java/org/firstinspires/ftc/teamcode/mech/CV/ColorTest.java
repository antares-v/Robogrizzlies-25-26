package org.firstinspires.ftc.teamcode.mech.CV;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import org.firstinspires.ftc.teamcode.mech.movement.goBuildaPinPointDriver.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.mech.movement.goBuildaPinPointDriver.Pose2D;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Color Test")
public class ColorTest extends LinearOpMode {
    RevColorSensorV3 color;

    @Override public void runOpMode() {
        color = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        color.enableLed(true);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("R", color.red());
            telemetry.addData("G", color.green());
            telemetry.addData("B", color.blue());
            telemetry.addData("A", color.alpha());
            telemetry.update();
        }
    }
}