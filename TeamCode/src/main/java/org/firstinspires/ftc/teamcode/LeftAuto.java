package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="LeftAuto", group="Linear OpMode")

public class LeftAuto extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor FR = null;
    private DcMotor BR = null;

    private DcMotor armSlidesM = null;
    Servo armAxelS;

    private DcMotor leftEncoderMotor = null;
    private double leftEncoderPos = 0;
    private double deltaLeftEncoder = 0;

    private DcMotor rightEncoderMotor = null;
    private double rightEncoderPos = 0;
    private double deltaRightEncoder = 0;

    private DcMotor centerEncoderMotor = null;
    private double centerEncoderPos = 0;
    private double deltaCenterEncoder = 0;

    private boolean rightStop = false;
    private boolean leftStop = false;

    private double tileMatLength = 12*2;  // Inches

    double localTargetTick;

    // Calculates the circumference for the Odometry pods
    // Divides by 25.4 to change mm to inches
    double OPcircumference = 2.0*Math.PI*(16.0/25.4);

    double yaw;

    double curAngle = 0;
    IMU imu;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        FL = hardwareMap.get(DcMotor.class, "left_front_drive");
        BL = hardwareMap.get(DcMotor.class, "left_back_drive");
        FR = hardwareMap.get(DcMotor.class, "right_front_drive");
        BR = hardwareMap.get(DcMotor.class, "right_back_drive");

        armSlidesM = hardwareMap.get(DcMotor.class, "arm_slides");
        armAxelS = hardwareMap.get(Servo.class, "arm_axel");

        leftEncoderMotor = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightEncoderMotor = hardwareMap.get(DcMotor.class, "right_front_drive");
        centerEncoderMotor = hardwareMap.get(DcMotor.class, "left_back_drive");

        imu = hardwareMap.get(IMU.class, "imu");

        leftEncoderMotor.setDirection(DcMotorSimple.Direction.FORWARD);  // Directions taken from BlackBoxBot.java
        rightEncoderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        centerEncoderMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        yaw = orientation.getYaw();

        resetTicks();
        setNormalDrive();  // Sets all motors to correct forward/reverse

        // Send telemetry message to signify robot waiting;
        telemIMUOrientation(orientation, yaw);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        resetTicks();

        while (opModeIsActive()) {

        /* Protoype Meet 1 Code

        !!! ALL TARGET TICKS ARE ESTIMATED !!!
        !!! NEED MORE HARDWARE BEFORE WE CAN TEST ANYTHING !!!
        !*!*!*! THIS IS JUST PROTOTYPING CODE, WILL NEED MAJOR TWEAKING !*!*!*!

        *** Variables needed ***
        String sampleColor = color sensor reading on arm
        String allianceColor = color sensor reading of alliance marker
        String oppenentColor;

        *** Chicken scratch code ***

        if allianceColor == "blue"{
        opponentColor = "red"
        }
        else{
        openentColor = "blue"
        }

        localTargetTick = InchesToTicks(tileMatLength);
        driveForward(localTargetTick, -0.5, 1);

        ARM CODE:
            -- Hang starting specimen
            -- Grab another Sample from sub

         if (sampleColor == "yellow"){
            move to bucket
            left = True (this will be for later)
            localTargetTick = InchesToTicks(tileMatLength*1.5);
            strafeLeft(localTargetTick, -0.4, 1);
         }

         else if(sampleColor == allianceColor){
            move to obs
            right = True (this will be for later)
            localTargetTick = InchesToTicks(tileMatLength*2);
            strafeRight(localTargetTick, -0.4, 1);
         }

         else if (sampleColor == oppenentColor){
            EITHER:
            -keep looking
            OR:
            -cycle left and right sides
         }

         if (left == True && camera doesnt detect alliance partner){
            cycle all three yellow samples into baskets (perferably high)
         }
         else if (right == True && camera doesnt detect alliance partner){
            cycle all three alliance colored samples into observation
         }


         Notes:
         Encapsulate in while loop and check if time is about to run out so there is enough time to park
         5-10 seconds should be when the park code executes

         Do same thing for right ?? or get rid of right and only have one autonoumous.
         Also integrate camera for positioning so we don't need exact starting locations
         I also think we should put the camera on a servor so it faces the direction the robot is facing.
         */

            curAngle = turnLeft(-0.3, 2, 90, orientation, curAngle);
            curAngle = turnRight(-0.3, 2, 180, orientation, curAngle);
            curAngle = turnLeft(-0.3, 2, 270, orientation, curAngle);
            curAngle = turnRight(-0.3, 2, 359, orientation,curAngle);
            curAngle = turnLeft(-0.3, 2, 180, orientation, curAngle);  // 360 and 0 degrees don't work with IMU
            curAngle = turnRight(-0.3, 2, 45, orientation, curAngle);

            telemIMUOrientation(orientation, yaw);

        /* Meet 0 Code
        localTargetTick = InchesToTicks(tileMatLength*0.12);
        strafeRight(localTargetTick, -0.4, 1);

        localTargetTick = (InchesToTicks(tileMatLength*0.8));
        driveForward(localTargetTick, -0.5, 1);

        localTargetTick = (InchesToTicks(tileMatLength*0.8));
        driveBackward(localTargetTick, -0.5, 1);

        localTargetTick = InchesToTicks(tileMatLength*0.8);
        strafeRight(localTargetTick, -0.4, 1);

        localTargetTick = (InchesToTicks(tileMatLength*3.5));
        driveBackward(localTargetTick, -0.5, 1);

        localTargetTick = InchesToTicks(tileMatLength*0.6);
        strafeLeft(localTargetTick, -0.4, 1);
         */


            telemAllTicks("None");
            break;
        }
    }

    public void driveForward(double targetTicks, double power, long sleep) {
        resetTicks();
        setAllPower(power);

        telemAllTicks("Forward");

        while (!(rightStop && leftStop)) {
            if (getRightTicks() >= targetTicks) {
                rightStop = true;
                stopRightPower();
            }
            if (getLeftTicks() >= targetTicks) {
                leftStop = true;
                stopLeftPower();
            }
            telemAllTicks("Forward");
        }

        telemAllTicks("Forward");

        rightStop = false;
        leftStop = false;

        stopAllPower();
        resetTicks();

        sleep(1000*sleep);
    }

    public void driveBackward(double targetTicks, double power, long sleep) {
        resetTicks();
        setAllPower(-power);

        telemAllTicks("Backward");

        while (!(rightStop && leftStop)) {
            if (Math.abs(getRightTicks()) >= targetTicks) {
                rightStop = true;
                stopRightPower();
            }
            if (Math.abs(getLeftTicks()) >= targetTicks) {
                leftStop = true;
                stopLeftPower();
            }
            telemAllTicks("Backward");
        }

        telemAllTicks("Backward");

        rightStop = false;
        leftStop = false;
        stopAllPower();
        resetTicks();

        sleep(1000*sleep);
    }

    public void strafeRight(double targetTicks, double power, long sleep) {
        setStrafingDrive();
        resetTicks();
        setAllPower(power);

        telemAllTicks("Right");

        while (getCenterTicks() < targetTicks){
            telemAllTicks("Right");
        }

        telemAllTicks("Right");

        stopAllPower();
        resetTicks();
        setNormalDrive();

        sleep(1000*sleep);
    }

    public void strafeLeft(double targetTicks, double power, long sleep) {
        setStrafingDrive();
        resetTicks();
        setAllPower(-power);

        telemAllTicks("Left");

        while (Math.abs(getCenterTicks()) < targetTicks){
            telemAllTicks("Left");
        }

        telemAllTicks("Left");

        stopAllPower();
        resetTicks();
        setNormalDrive();

        sleep(1000*sleep);
    }


    /* These turn functions where types without a complier so idk if its messed up
     */
    public double turnLeft(double power, long sleep, double angle, YawPitchRollAngles orientation, double curAngle){
        setRightPower(power);
        setLeftPower(-power);

        double yaw = orientation.getYaw();
        double targetAngle = curAngle + angle;

        while(leftYawConversion(yaw) <= targetAngle){
            orientation = imu.getRobotYawPitchRollAngles();
            yaw = orientation.getYaw();
            telemIMUOrientation(orientation, yaw);
        }

        stopAllPower();
        resetTicks();

        telemIMUOrientation(orientation, yaw);

        sleep(1000*sleep);

        return ((curAngle+yaw)%360);
    }

    public double turnRight(double power, long sleep, double angle, YawPitchRollAngles orientation, double curAngle){
        setLeftPower(power);
        setRightPower(-power);

        double yaw = orientation.getYaw();
        double targetAngle = curAngle + (-angle);

        while(rightYawConversion(yaw) <= targetAngle){
            orientation = imu.getRobotYawPitchRollAngles();
            yaw = orientation.getYaw();
            telemIMUOrientation(orientation, yaw);
        }

        stopAllPower();
        resetTicks();

        telemIMUOrientation(orientation, yaw);

        sleep(1000*sleep);

        return((curAngle + yaw) % 360);
    }



    public void setNormalDrive(){
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setStrafingDrive(){
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void resetTicks(){
        resetLeftTicks();
        resetRightTicks();
        resetCenterTicks();
    }

    public void setAllPower(double p){
        FL.setPower(p);
        FR.setPower(p);
        BL.setPower(p);
        BR.setPower(p);
    }

    public void stopAllPower(){
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    public void setLeftPower(double p){
        FL.setPower(p);
        BL.setPower(p);
    }

    public void setRightPower(double p){
        FR.setPower(p);
        BR.setPower(p);
    }

    public void stopLeftPower(){
        FL.setPower(0);
        BL.setPower(0);
    }

    public void stopRightPower(){
        FR.setPower(0);
        BR.setPower(0);
    }

    public void resetLeftTicks(){
        leftEncoderPos = leftEncoderMotor.getCurrentPosition();
    }

    public double getLeftTicks(){
        return (-(leftEncoderMotor.getCurrentPosition() - leftEncoderPos));
    }

    public void resetRightTicks(){
        rightEncoderPos = rightEncoderMotor.getCurrentPosition();
    }

    public double getRightTicks(){
        return (rightEncoderMotor.getCurrentPosition() - rightEncoderPos);
    }

    public void resetCenterTicks(){
        centerEncoderPos = centerEncoderMotor.getCurrentPosition();
    }

    public double getCenterTicks(){
        return (centerEncoderMotor.getCurrentPosition() - centerEncoderPos);
    }

    public void telemAllTicks(String dir){
        telemetry.addData("Direction", dir);
        telemetry.addData("Left pos", getLeftTicks());
        telemetry.addData("Right pos", getRightTicks());
        telemetry.addData("Center pos", getCenterTicks());
        telemetry.update();
    }

    public void telemIMUOrientation(YawPitchRollAngles orientation, double yaw){
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
        telemetry.addData("Var Yaw", yaw);
        telemetry.update();
    }

    public double rightYawConversion(double yaw){
        if (yaw <= -1){
            return (yaw*-1);
        }
        else{
            return (360 - yaw);
        }
    }

    public double leftYawConversion(double yaw){
        if (yaw <= -1){
            return (yaw+360);
        }
        else{
            return yaw;
        }
    }

    public double TicksToInches(double ticks){
        double rev = (double)ticks/2000;
        double inches = OPcircumference * rev;
        return inches;
    }
    public double InchesToTicks(double inches){
        double rev = inches/OPcircumference;
        double tick = 2000*rev;
        return tick;
    }
}
