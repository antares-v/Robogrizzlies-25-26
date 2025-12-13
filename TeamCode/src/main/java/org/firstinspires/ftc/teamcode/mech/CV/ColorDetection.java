package org.firstinspires.ftc.teamcode.mech.CV;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.mech.movement.goBuildaPinPointDriver.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.mech.movement.goBuildaPinPointDriver.Pose2D;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
public class ColorDetection {
    public String getColor(ColorSensor sensor) {
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();
        if (red < 50 && green < 50 && blue < 50) {
            return "blank";
        }
        if (green > red && green > blue) {
            return "green";
        }
        else {
            return "purple";
        }
    }

}
