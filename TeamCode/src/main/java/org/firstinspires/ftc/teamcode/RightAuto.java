package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name="RightAuto", group="Linear OpMode")

public class RightAuto extends LinearOpMode {

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

    private double theta = 0;

    private boolean rightStop = false;
    private boolean leftStop = false;

    private double tileMatLength = 12*2;  // Inches

    double localTargetTick;

    // Calculates the circumference for the Odometry pods
    // Divides by 25.4 to change mm to inches
    double OPcircumference = 2.0*Math.PI*(16.0/25.4);

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

        leftEncoderMotor.setDirection(DcMotorSimple.Direction.FORWARD);  // Directions taken from BlackBoxBot.java
        rightEncoderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        centerEncoderMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetTicks();
        setNormalDrive();  // Sets all motors to correct forward/reverse

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        resetTicks();

        localTargetTick = InchesToTicks(tileMatLength*0.9);
        strafeRight(localTargetTick, -0.4, 1);

        localTargetTick = (InchesToTicks(tileMatLength*2.85));
        driveForward(localTargetTick, -0.5, 1);

        localTargetTick = InchesToTicks(tileMatLength*0.8);
        strafeLeft(localTargetTick, -0.4, 1);

        localTargetTick = (InchesToTicks(tileMatLength*0.5));
        driveBackward(localTargetTick, -0.5, 1);

        localTargetTick = InchesToTicks(tileMatLength*0.9);
        strafeRight(localTargetTick, -0.4, 1);

        localTargetTick = (InchesToTicks(tileMatLength*4.0));
        driveBackward(localTargetTick, -0.5, 1);

        localTargetTick = InchesToTicks(tileMatLength*0.5);
        strafeLeft(localTargetTick, -0.4, 1);


        telemAllTicks("None");
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
