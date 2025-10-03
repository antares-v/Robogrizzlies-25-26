package org.firstinspires.ftc.teamcode.mech.movement;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
public class launcher {
    //Define our motor for the wheel
    private DcMotor launcher;
    //Constructor For Wheel:
    public launcher(HardwareMap map){
        motor = map.get(DcMotor.class, "Launcher");
        motor.setDirection(DcMotor.Direction.FORWARD);
    }
}
