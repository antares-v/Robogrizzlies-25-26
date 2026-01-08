package org.firstinspires.ftc.teamcode.mech.CV;

import com.qualcomm.hardware.rev.RevColorSensorV3;

public class ColorDetection {
    // Tune values
    private static final int MIN_ALPHA = 200;     // blank threshold
    public void enableLed(RevColorSensorV3 sensor) {sensor.enableLed(true);}

    public String getColor(RevColorSensorV3 sensor) {
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();
        int a = sensor.alpha(); // overall intensity

        // Blank / nothing in front
        if (a < MIN_ALPHA) return "blank";

        // Green dominant
        if (g > r && g > b) {
            return "green";
        }

        // Purple, red+blue dominant
        else {
            return "purple";
        }
    }
}
// Blank - R:96 G:123 B:88 A:102
// Purple - R:785 G:925 B:1466 A:1061
// Green - R:289 G:1161 B:837 A:764
