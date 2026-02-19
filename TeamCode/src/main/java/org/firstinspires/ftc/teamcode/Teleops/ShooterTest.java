package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ShooterTest {

    DcMotorEx shooter1;
    DcMotorEx shooter2;
    boolean running = false;

    // Tune this to your desired RPM
    private static final double TARGET_RPM = 300; // change this!
    private static final double TARGET_DEG_PER_SEC = TARGET_RPM * 6.0; // RPM * 360/60

    public ShooterTest(HardwareMap hardwareMap) {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        // Reset and set to velocity control mode
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Optional: set zero power behavior
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void toggleMotor() {
        if (running) {
            shooter1.setVelocity(0);
            shooter2.setVelocity(0);
            running = false;
        } else {
            shooter1.setVelocity(TARGET_DEG_PER_SEC, AngleUnit.DEGREES);
            shooter2.setVelocity(TARGET_DEG_PER_SEC, AngleUnit.DEGREES);
            running = true;
        }
    }
}