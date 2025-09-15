package org.firstinspires.ftc.teamcode.mech.arm;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class motor {
    private DcMotor m;
    public motor(HardwareMap hardwareMap, String loc, boolean reverse) {
        m = hardwareMap.get(DcMotor.class, loc);
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (reverse) {
            m.setDirection(DcMotor.Direction.REVERSE);
        }
    }
    public void setPower(double p) {
        m.setPower(p);
    }
    public double getPower() {
        return m.getPower();
    }

    public void setPlace(int t, double power) {
        m.setTargetPosition(t);
        m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m.setPower(power);
    }
    public void reset(){
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public int get_pos(){
        return m.getCurrentPosition();
    }
}
