package org.firstinspires.ftc.teamcode.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config       //if you want config
@TeleOp       //if this is a teleop
//@Autonomous   //if this is an auto
public class TestOp extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    GamepadEx g1;
    public static double jump = .5;
    IMU myIMU;
    //this section allows us to access telemetry data from a browser
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    DcMotor myMotor;
    DcMotor myMotor2;
    DcMotor myMotor3;
    DcMotor myMotor4;
    double holdYaw = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step
        g1 = new GamepadEx(gamepad1);
        IMU.Parameters myIMUparameters;

        myMotor = hardwareMap.get(DcMotor.class,"myMotor");
        myMotor2 = hardwareMap.get(DcMotor. class, "myMotor2");
        myMotor3 = hardwareMap.get(DcMotor. class, "myMotor3");
        myMotor4 = hardwareMap.get(DcMotor. class, "myMotor4");
        myIMU = hardwareMap.get(IMU.class,"myIMU    ");

        myMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        myMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done

        myMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        myMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done

        myMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        myMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done

        myMotor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        myMotor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done

        myMotor2.setDirection(DcMotor.Direction.REVERSE);
        // Tell the driver that initialization is complete.
       RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot.LogoFacingDirection logodirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(logodirection,usbFacingDirection);
                myIMU.initialize(new IMU.Parameters(orientation));
                dashboardTelemetry.addData("Status", "Initialized");
        dashboardTelemetry.update();

    }


    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
        //myMotor.setPower(jump);
        //myMotor2.setPower(jump);

        myIMU.resetYaw();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        YawPitchRollAngles angles = myIMU.getRobotYawPitchRollAngles();
        double yaw = angles.getYaw();
        double pitch = angles.getPitch();
        double roll = angles.getRoll();
        g1.readButtons();
        dashboardTelemetry.update();
       // float x;
       // float y;

        double left_y = g1.getLeftY();
        double right_x = g1.getRightX();

        if (Math.abs(left_y) < 0.05) {
            left_y = 0;
        }

        if (Math.abs(right_x) < 0.05) {
            right_x = 0;
            if ( holdYaw + yaw < -2) {
                right_x = yaw / 10.0;
            }else if (holdYaw - yaw > 2){
                right_x = -yaw / 10.0;
            }
        }else{
            myIMU.resetYaw();
        }



        double leftpower = Range.clip(left_y+right_x,-1.0,1.0);
        double rightpower = Range.clip(left_y-right_x,-1.0,1.0);
        myMotor.setPower(leftpower);
        myMotor2.setPower(-rightpower);

        myMotor3.setPower(jump);
        myMotor4.setPower(jump);
/*
        if (Math.abs(turn) > 0.1) {
            myIMU.resetYaw();
        } else if (yaw < -2) {
            myMotor.setPower(angles.getYaw() / 40);
        } else if (yaw > 2) {
            myMotor.setPower(angles.getYaw() / 40);
        }
        else {
            myMotor.setPower(0);
        }
        if (Math.abs(turn) > 0) {
            myIMU.resetYaw();
        } else if (yaw < -2) {
            myMotor2.setPower(-angles.getYaw() / 40);
        } else if (yaw > 2) {
            myMotor2.setPower(-angles.getYaw() / 40);
        }
        else {
            myMotor2.setPower(0);
        }
*/


        dashboardTelemetry.addData("yaw", yaw);
        dashboardTelemetry.addData("pitch", pitch);
        dashboardTelemetry.addData("roll", roll);

        dashboardTelemetry.addData("motor ticks", jump);
        dashboardTelemetry.addData("motor ticks", position);
        dashboardTelemetry.addData("Status", "Run Time: " + runtime.toString());
        dashboardTelemetry.addData("angle",myIMU.getRobotYawPitchRollAngles().getYaw());
    }
    int position = myMotor.getCurrentPosition();

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        runtime.reset();
        myMotor.setPower(0);
        myMotor2.setPower(0);
        myMotor3.setPower(0);
        myMotor4.setPower(0);
    }
}