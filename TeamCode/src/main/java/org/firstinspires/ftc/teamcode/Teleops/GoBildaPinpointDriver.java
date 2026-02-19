//package org.firstinspires.ftc.teamcode.Teleops;
//
//import static com.qualcomm.robotcore.util.TypeConversion.byteArrayToInt;
//
//import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
//import com.qualcomm.robotcore.hardware.I2cAddr;
//import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
//import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
//import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
//import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
//import com.qualcomm.robotcore.util.TypeConversion;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//
//import java.nio.ByteBuffer;
//import java.nio.ByteOrder;
//import java.util.Arrays;
//
//
//@I2cDeviceType
//@DeviceProperties(
//        name = "goBILDA® Pinpoint Odometry Computer",
//        xmlTag = "goBILDAPinpoint",
//        description = "goBILDA® Pinpoint Odometry Computer (IMU Sensor Fusion for 2 Wheel Odometry)"
//)
//public class GoBildaPinpointDriver extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {
//
//    private int   deviceStatus   = 0;
//    private int   loopTime       = 0;
//    private int   xEncoderValue  = 0;
//    private int   yEncoderValue  = 0;
//    private float xPosition      = 0;
//    private float yPosition      = 0;
//    private float hOrientation   = 0;
//    private float xVelocity      = 0;
//    private float yVelocity      = 0;
//    private float hVelocity      = 0;
//
//    public static final byte DEFAULT_ADDRESS = 0x31;
//
//    public GoBildaPinpointDriver(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
//        super(deviceClient, deviceClientIsOwned);
//        this.deviceClient.setI2cAddress(I2cAddr.create7bit(DEFAULT_ADDRESS));
//        super.registerArmingStateCallback(false);
//    }
//
//    @Override
//    public Manufacturer getManufacturer() {
//        return Manufacturer.Other;
//    }
//
//    @Override
//    protected synchronized boolean doInitialize() {
//        ((LynxI2cDeviceSynch)(deviceClient)).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
//        return true;
//    }
//
//    @Override
//    public String getDeviceName() {
//        return "goBILDA® Pinpoint Odometry Computer";
//    }
//
//    private enum Register {
//        DEVICE_ID       (1),
//        DEVICE_VERSION  (2),
//        DEVICE_STATUS   (3),
//        DEVICE_CONTROL  (4),
//        LOOP_TIME       (5),
//        X_ENCODER_VALUE (6),
//        Y_ENCODER_VALUE (7),
//        X_POSITION      (8),
//        Y_POSITION      (9),
//        H_ORIENTATION   (10),
//        X_VELOCITY      (11),
//        Y_VELOCITY      (12),
//        H_VELOCITY      (13),
//        MM_PER_TICK     (14),
//        X_POD_OFFSET    (15),
//        Y_POD_OFFSET    (16),
//        YAW_SCALAR      (17),
//        BULK_READ       (18);
//
//        private final int bVal;
//        Register(int bVal) { this.bVal = bVal; }
//    }
//
//    public enum DeviceStatus {
//        NOT_READY                (0),
//        READY                    (1),
//        CALIBRATING              (1 << 1),
//        FAULT_X_POD_NOT_DETECTED (1 << 2),
//        FAULT_Y_POD_NOT_DETECTED (1 << 3),
//        FAULT_NO_PODS_DETECTED   (1 << 2 | 1 << 3),
//        FAULT_IMU_RUNAWAY        (1 << 4);
//
//        private final int status;
//        DeviceStatus(int status) { this.status = status; }
//    }
//
//    public enum EncoderDirection {
//        FORWARD,
//        REVERSED
//    }
//
//    public enum readData {
//        ONLY_UPDATE_HEADING
//    }
//
//    // -------------------------------------------------------------------------
//    // Private I2C helpers
//    // -------------------------------------------------------------------------
//
//    private void writeInt(final Register reg, int i) {
//        deviceClient.write(reg.bVal, TypeConversion.intToByteArray(i, ByteOrder.LITTLE_ENDIAN));
//    }
//
//    private int readInt(Register reg) {
//        return byteArrayToInt(deviceClient.read(reg.bVal, 4), ByteOrder.LITTLE_ENDIAN);
//    }
//
//    private float byteArrayToFloat(byte[] byteArray, ByteOrder byteOrder) {
//        return ByteBuffer.wrap(byteArray).order(byteOrder).getFloat();
//    }
//
//    private float readFloat(Register reg) {
//        return byteArrayToFloat(deviceClient.read(reg.bVal, 4), ByteOrder.LITTLE_ENDIAN);
//    }
//
//    private byte[] floatToByteArray(float value, ByteOrder byteOrder) {
//        return ByteBuffer.allocate(4).order(byteOrder).putFloat(value).array();
//    }
//
//    private void writeByteArray(Register reg, byte[] bytes) {
//        deviceClient.write(reg.bVal, bytes);
//    }
//
//    private void writeFloat(Register reg, float f) {
//        byte[] bytes = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putFloat(f).array();
//        deviceClient.write(reg.bVal, bytes);
//    }
//
//    private DeviceStatus lookupStatus(int s) {
//        if ((s & DeviceStatus.CALIBRATING.status) != 0)   return DeviceStatus.CALIBRATING;
//
//        boolean xPodDetected = (s & DeviceStatus.FAULT_X_POD_NOT_DETECTED.status) == 0;
//        boolean yPodDetected = (s & DeviceStatus.FAULT_Y_POD_NOT_DETECTED.status) == 0;
//
//        if (!xPodDetected && !yPodDetected) return DeviceStatus.FAULT_NO_PODS_DETECTED;
//        if (!xPodDetected)                  return DeviceStatus.FAULT_X_POD_NOT_DETECTED;
//        if (!yPodDetected)                  return DeviceStatus.FAULT_Y_POD_NOT_DETECTED;
//
//        if ((s & DeviceStatus.FAULT_IMU_RUNAWAY.status) != 0) return DeviceStatus.FAULT_IMU_RUNAWAY;
//        if ((s & DeviceStatus.READY.status) != 0)             return DeviceStatus.READY;
//
//        return DeviceStatus.NOT_READY;
//    }
//
//    // -------------------------------------------------------------------------
//    // Public API
//    // -------------------------------------------------------------------------
//
//    /**
//     * Call once per loop to read fresh data from the Pinpoint.
//     */
//    public void update() {
//        byte[] bArr  = deviceClient.read(Register.BULK_READ.bVal, 40);
//        deviceStatus  = byteArrayToInt(Arrays.copyOfRange  (bArr, 0,  4),  ByteOrder.LITTLE_ENDIAN);
//        loopTime      = byteArrayToInt(Arrays.copyOfRange  (bArr, 4,  8),  ByteOrder.LITTLE_ENDIAN);
//        xEncoderValue = byteArrayToInt(Arrays.copyOfRange  (bArr, 8,  12), ByteOrder.LITTLE_ENDIAN);
//        yEncoderValue = byteArrayToInt(Arrays.copyOfRange  (bArr, 12, 16), ByteOrder.LITTLE_ENDIAN);
//        xPosition     = byteArrayToFloat(Arrays.copyOfRange(bArr, 16, 20), ByteOrder.LITTLE_ENDIAN);
//        yPosition     = byteArrayToFloat(Arrays.copyOfRange(bArr, 20, 24), ByteOrder.LITTLE_ENDIAN);
//        hOrientation  = byteArrayToFloat(Arrays.copyOfRange(bArr, 24, 28), ByteOrder.LITTLE_ENDIAN);
//        xVelocity     = byteArrayToFloat(Arrays.copyOfRange(bArr, 28, 32), ByteOrder.LITTLE_ENDIAN);
//        yVelocity     = byteArrayToFloat(Arrays.copyOfRange(bArr, 32, 36), ByteOrder.LITTLE_ENDIAN);
//        hVelocity     = byteArrayToFloat(Arrays.copyOfRange(bArr, 36, 40), ByteOrder.LITTLE_ENDIAN);
//    }
//
//    /**
//     * Faster partial update — only refreshes the heading value.
//     */
//    public void update(readData data) {
//        if (data == readData.ONLY_UPDATE_HEADING) {
//            hOrientation = byteArrayToFloat(
//                    deviceClient.read(Register.H_ORIENTATION.bVal, 4),
//                    ByteOrder.LITTLE_ENDIAN
//            );
//        }
//    }
//
//    /**
//     * Sets the odometry pod positions relative to the robot's center (in mm).
//     * X offset: left of center is positive, right is negative.
//     * Y offset: forward of center is positive, backwards is negative.
//     */
//    public void setOffsets(double xOffset, double yOffset) {
//        writeFloat(Register.X_POD_OFFSET, (float) xOffset);
//        writeFloat(Register.Y_POD_OFFSET, (float) yOffset);
//    }
//
//    /**
//     * Recalibrates the internal IMU. Robot MUST be stationary. Takes ~0.25 seconds.
//     */
//    public void recalibrateIMU() { writeInt(Register.DEVICE_CONTROL, 1 << 0); }
//
//    /**
//     * Resets position to 0,0,0 and recalibrates the IMU. Robot MUST be stationary. Takes ~0.25 seconds.
//     */
//    public void resetPosAndIMU() { writeInt(Register.DEVICE_CONTROL, 1 << 1); }
//
//    /**
//     * Sets encoder directions. X pod should increase moving forward, Y pod should increase moving left.
//     */
//    public void setEncoderDirections(EncoderDirection xEncoder, EncoderDirection yEncoder) {
//        if (xEncoder == EncoderDirection.REVERSED) writeInt(Register.DEVICE_CONTROL, 1 << 4);
//        if (yEncoder == EncoderDirection.REVERSED) writeInt(Register.DEVICE_CONTROL, 1 << 2);
//    }
//
//    /**
//     * Sets encoder resolution in ticks per mm for custom (non-goBILDA) encoders.
//     * Divide your encoder's counts-per-revolution by the wheel circumference to get this value.
//     * Example: REV Through-Bore = 13.26291192 ticks/mm
//     * @param ticks_per_mm typically between 10 and 100
//     */
//    public void setEncoderResolution(double ticks_per_mm) {
//        writeByteArray(Register.MM_PER_TICK,
//                floatToByteArray((float) ticks_per_mm, ByteOrder.LITTLE_ENDIAN));
//    }
//
//    /**
//     * Scalar applied to the gyro yaw. Only tune if heading accuracy is off.
//     * Rotate robot 10 full turns, compare reported vs actual. Divide actual by reported for the scalar.
//     */
//    public void setYawScalar(double yawOffset) {
//        writeByteArray(Register.YAW_SCALAR,
//                floatToByteArray((float) yawOffset, ByteOrder.LITTLE_ENDIAN));
//    }
//
//    /**
//     * Overrides the current tracked position. Useful for field-coordinate tracking
//     * or fusing with external sensors like AprilTags.
//     */
//    public Pose2D setPosition(Pose2D pos) {
//        writeByteArray(Register.X_POSITION,
//                floatToByteArray((float) pos.getX(DistanceUnit.MM),        ByteOrder.LITTLE_ENDIAN));
//        writeByteArray(Register.Y_POSITION,
//                floatToByteArray((float) pos.getY(DistanceUnit.MM),        ByteOrder.LITTLE_ENDIAN));
//        writeByteArray(Register.H_ORIENTATION,
//                floatToByteArray((float) pos.getHeading(AngleUnit.RADIANS), ByteOrder.LITTLE_ENDIAN));
//        return pos;
//    }
//
//    // -------------------------------------------------------------------------
//    // Getters
//    // -------------------------------------------------------------------------
//
//    public int          getDeviceID()       { return readInt(Register.DEVICE_ID); }
//    public int          getDeviceVersion()  { return readInt(Register.DEVICE_VERSION); }
//    public float        getYawScalar()      { return readFloat(Register.YAW_SCALAR); }
//    public DeviceStatus getDeviceStatus()   { return lookupStatus(deviceStatus); }
//    public int          getLoopTime()       { return loopTime; }
//    public int          getEncoderX()       { return xEncoderValue; }
//    public int          getEncoderY()       { return yEncoderValue; }
//    public double       getPosX()           { return xPosition; }
//    public double       getPosY()           { return yPosition; }
//    public double       getHeading()        { return hOrientation; }
//    public double       getVelX()           { return xVelocity; }
//    public double       getVelY()           { return yVelocity; }
//    public double       getHeadingVelocity(){ return hVelocity; }
//
//    /** Avoid calling every loop — triggers its own I2C read. */
//    public float getXOffset() { return readFloat(Register.X_POD_OFFSET); }
//    /** Avoid calling every loop — triggers its own I2C read. */
//    public float getYOffset() { return readFloat(Register.Y_POD_OFFSET); }
//
//    public double getFrequency() {
//        return loopTime != 0 ? 1000000.0 / loopTime : 0;
//    }
//
//    public Pose2D getPosition() {
//        return new Pose2D(DistanceUnit.MM, xPosition, yPosition, AngleUnit.RADIANS, hOrientation);
//    }
//
//    public Pose2D getVelocity() {
//        return new Pose2D(DistanceUnit.MM, xVelocity, yVelocity, AngleUnit.RADIANS, hVelocity);
//    }
//}