package org.firstinspires.ftc.teamcode.mech.mainIntakeOuttake;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
public class launcher {
    //Define our motor for the wheel
    LinearOpMode l;
    private DcMotorEx launcher1;
    //Constructor For Wheel:
    public launcher(LinearOpMode l){
        launcher1 = l.hardwareMap.get(DcMotorEx.class, "Launcher");
        launcher1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher1.setDirection(DcMotorEx.Direction.FORWARD);
    }
    public void on(){
        launcher1.setVelocity(1);
    }
    public void off(){
        launcher1.setVelocity(0);
    }
}

/* DETAILED INFORMATION ABOUT POWER AND DISTANCE
Angle of launcher: 58 degrees

Data: Power, Time
    [1, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1] 3c

 */
