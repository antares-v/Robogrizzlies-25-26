package org.firstinspires.ftc.teamcode.mech.movement;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
public class wheel {
    //Define our motor for the wheel
    private DcMotor motor;
    //Constructor For Wheel:
    private Servo servo;
    public wheel(HardwareMap map, String s, boolean reverse){
        motor = map.get(DcMotor.class, s);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(reverse){
            motor.setDirection(DcMotor.Direction.REVERSE);
        }else{
            motor.setDirection(DcMotor.Direction.FORWARD);
        }
    }
    /*public servoV2(HardwareMap map, String s){
        servo = map.get(DcMotor.class, s);
    }*/
    public void setPower(double power){
        //if above 1
        motor.setPower(power);
    }
    public double getPower(){
        return motor.getPower();
    }
}
