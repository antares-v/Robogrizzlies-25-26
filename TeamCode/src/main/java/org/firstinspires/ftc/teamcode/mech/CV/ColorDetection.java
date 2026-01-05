package org.firstinspires.ftc.teamcode.mech.CV;

import com.qualcomm.hardware.rev.RevColorSensorV3;

public class ColorDetection {

    // Tune values
    private static final int MIN_ALPHA = 30;     // blank threshold
    private static final int DOMINANCE_MARGIN = 30; // channel must exceed others by this much

    public String getColor(RevColorSensorV3 sensor) {
        sensor.enableLed(true);

        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();
        int a = sensor.alpha(); // overall intensity

        // Blank / nothing in front
        if (a < MIN_ALPHA) return "blank";

        // Green dominant
        if (g > r + DOMINANCE_MARGIN && g > b + DOMINANCE_MARGIN) {
            return "green";
        }

        // Purple, red+blue dominant
        if ((r + b) > g + DOMINANCE_MARGIN) {
            return "purple";
        }

        return "blank";
    }
}
