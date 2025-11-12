package org.firstinspires.ftc.teamcode.mech.mainIntakeOuttake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

public class intermediatelauncher {
    //Define our motor for the wheel
    LinearOpMode l;
    private CRServo launcher;
    //Constructor For Wheel:
    public intermediatelauncher(LinearOpMode l, String launchername){
        launcher = l.hardwareMap.get(CRServo.class, launchername);
        launcher.setDirection(CRServo.Direction.FORWARD);
        launcher.setPower(0);
    }
    public void on(){
        launcher.setPower(1);
    }
    public void off(){
        launcher.setPower(0);
    }
}

/* DETAILED INFORMATION ABOUT POWER AND DISTANCE
Angle of launcher: 58 degrees

Data: Power, Time
    [1, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1] 3c

 */
