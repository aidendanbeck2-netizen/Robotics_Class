//package org.firstinspires.ftc.teamcode.Teleops;
//
//import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.Teleops.GoBildaPinpointDriver;
//
//public class Testcodeforwheels {
//
//    private GoBildaPinpointDriver pinpoint;
//
//    private DcMotor dcMotor;
//    private DcMotor dcMotor2;
//    private DcMotor dcMotor3;
//    private DcMotor dcMotor4;
//
//    public Testcodeforwheels(HardwareMap hardwareMap) {
//
//        dcMotor  = hardwareMap.get(DcMotor.class, "dcMotor");
//        dcMotor2 = hardwareMap.get(DcMotor.class, "dcMotor2");
//        dcMotor3 = hardwareMap.get(DcMotor.class, "dcMotor3");
//        dcMotor4 = hardwareMap.get(DcMotor.class, "dcMotor4");
//
//        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//
//        /*
//         * Set the odometry pod offsets in mm relative to the center of your robot.
//         * X offset: left of center = positive, right = negative
//         * Y offset: forward of center = positive, backward = negative
//         * Measure these carefully on your physical robot.
//         */
//        pinpoint.setOffsets(110, -84);
//
//        /*
//         * Set your encoder resolution in ticks per mm.
//         * Divide your encoder's CPR by the wheel circumference in mm.
//         * Examples:
//         *   REV Through-Bore: 13.26291192
//         *   Custom encoder: calculate and replace accordingly
//         */
//        pinpoint.setEncoderResolution(13.26291192);
//
//        // Flip either direction if your position readings go the wrong way
//        pinpoint.setEncoderDirections(
//                GoBildaPinpointDriver.EncoderDirection.FORWARD,
//                GoBildaPinpointDriver.EncoderDirection.FORWARD
//        );
//
//        // Zero out position and recalibrate the Pinpoint's internal IMU
//        pinpoint.resetPosAndIMU();
//
//        dcMotor.setDirection(DcMotor.Direction.REVERSE);
//        dcMotor2.setDirection(DcMotor.Direction.FORWARD);
//        dcMotor3.setDirection(DcMotor.Direction.REVERSE);
//        dcMotor4.setDirection(DcMotor.Direction.FORWARD);
//
//        dcMotor.setZeroPowerBehavior(BRAKE);
//        dcMotor2.setZeroPowerBehavior(BRAKE);
//        dcMotor3.setZeroPowerBehavior(BRAKE);
//        dcMotor4.setZeroPowerBehavior(BRAKE);
//    }
//
//    /**
//     * Returns heading in radians directly from the Pinpoint.
//     * No REV Hub IMU involved at all.
//     */
//    public double getYaw() {
//        return pinpoint.getHeading();
//    }
//
//    /**
//     * Returns the full Pose2D (X, Y, Heading) from Pinpoint.
//     */
//    public org.firstinspires.ftc.robotcore.external.navigation.Pose2D getPose() {
//        return pinpoint.getPosition();
//    }
//
//    /**
//     * Resets the Pinpoint's position and IMU to zero.
//     * Call this at the start of your OpMode.
//     */
//    public void resetPose() {
//        pinpoint.resetPosAndIMU();
//    }
//
//    /**
//     * Field-centric mecanum drive using Pinpoint heading.
//     * Call this every loop — update() is handled internally.
//     *
//     * @param forward  Joystick Y (positive = forward)
//     * @param strafe   Joystick X (positive = right)
//     * @param rx       Rotation (positive = clockwise)
//     */
//    public void mecanumDrive(double forward, double strafe, double rx) {
//        pinpoint.update();
//
//        double botHeading = getYaw();
//
//        double rotX = strafe  * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
//        double rotY = strafe  * Math.sin(-botHeading) + forward * Math.cos(-botHeading);
//        rotX = rotX * 1.1; // Counteract imperfect strafing
//
//        double denominator     = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//        double leftFrontPower  = (rotY + rotX + rx) / denominator;
//        double leftBackPower   = (rotY - rotX + rx) / denominator;
//        double rightFrontPower = (rotY - rotX - rx) / denominator;
//        double rightBackPower  = (rotY + rotX - rx) / denominator;
//
//        dcMotor.setPower(leftFrontPower);
//        dcMotor2.setPower(rightFrontPower);
//        dcMotor3.setPower(leftBackPower);
//        dcMotor4.setPower(rightBackPower);
//    }
//}