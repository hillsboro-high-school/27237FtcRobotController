package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name="Odometry Pods", group="Linear OpMode")

public class OdometryPods extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor FR = null;
    private DcMotor BR = null;

    private DcMotor leftEncoderMotor = null;
    private int leftEncoderPos = 0;
    private double deltaLeftEncoder = 0;

    private DcMotor rightEncoderMotor = null;
    private int rightEncoderPos = 0;
    private double deltaRightEncoder = 0;

    private DcMotor centerEncoderMotor = null;
    private int centerEncoderPos = 0;
    private double deltaCenterEncoder = 0;

    private double theta = 0;

    // Calculates the circumference for the Odometry pods
    // Divides by 25.4 to change mm to inches
    double OPcircumference = 2*Math.PI*(16/25.4);

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        FL = hardwareMap.get(DcMotor.class, "left_front_drive");
        BL = hardwareMap.get(DcMotor.class, "left_back_drive");
        FR = hardwareMap.get(DcMotor.class, "right_front_drive");
        BR = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftEncoderMotor = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightEncoderMotor = hardwareMap.get(DcMotor.class, "right_front_drive");
        centerEncoderMotor = hardwareMap.get(DcMotor.class, "left_back_drive");

        leftEncoderMotor.setDirection(DcMotorSimple.Direction.FORWARD);  // Directions taken from BlackBoxBot.java
        leftEncoderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftEncoderMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        resetTicks();

        int targetInches = 12*3;
        double targetPulses = InchesToPulse(targetInches);

        setPower(0.2);

        while(getLeftTicks() < targetPulses){
            telemetry.addData("Target Pulse", targetPulses);
            telemetry.addData("Left Pulse", getLeftTicks());
            telemetry.update();
        }

        stopPower();

        telemetry.addData("Target Pulse", targetPulses);
        telemetry.addData("Left Pulse", getLeftTicks());
        telemetry.update();



    }
    public void resetTicks(){
        resetLeftTicks();
        resetRightTicks();
        resetCenterTicks();
    }

    public void setPower(double p){
        FL.setPower(p);
        FR.setPower(p);
        BL.setPower(p);
        BR.setPower(p);
    }

    public void stopPower(){
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

    }

    public void resetLeftTicks(){
        leftEncoderPos = leftEncoderMotor.getCurrentPosition();
    }

    public int getLeftTicks(){
        return leftEncoderMotor.getCurrentPosition() - leftEncoderPos;
    }

    public void resetRightTicks(){
        rightEncoderPos = rightEncoderMotor.getCurrentPosition();
    }

    public int getRightTicks(){
        return rightEncoderMotor.getCurrentPosition() - rightEncoderPos;
    }

    public void resetCenterTicks(){
        rightEncoderPos = rightEncoderMotor.getCurrentPosition();
    }

    public int getCenterTicks(){
        return centerEncoderMotor.getCurrentPosition() - centerEncoderPos;
    }

    public double PulseToInches(int pulses){
        double rev = (double)pulses/2000;
        double inches = OPcircumference * rev;
        return inches;
    }
    public double InchesToPulse(int inches){
        double rev = inches/OPcircumference;
        double pulses = 2000*rev;
        return pulses;
    }
}
