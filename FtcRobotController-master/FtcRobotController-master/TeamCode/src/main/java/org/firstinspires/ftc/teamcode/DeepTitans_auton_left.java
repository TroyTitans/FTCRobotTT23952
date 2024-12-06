/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external "Robot" class that manages all hardware interactions.
 * Pure Drive or Strafe motions are maintained using two Odometry Wheels.
 * The IMU gyro is used to stabilize the heading during all motions
 */

@Autonomous(name="DeepTroyTitansAutonLeft", group = "TroyTitans", preselectTeleOp = "TroyTitansDeepTeleop")
public class DeepTroyTitansAutonLeft extends LinearOpMode
{
    // get an instance of the "Robot" class.
    private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);

    private DcMotor slider;
    private Servo gripper;
    private Servo wrist;
    private Servo subWrist;
    private DcMotor sliderSub;
    private Servo subIntake;

    int sliderhangposition;
    int sliderHangSpec;
    int sliderSamplePick;
    int sliderHighbasket;

    double wristUpPosition;
    double wristDownPosition;
    double wristScoringPosition;
    double wristHangSpecPosition;

    double subIntakeOff;
    double subWristUp;

    double gripperClosedPosition;
    double gripperOpenPosition;

    double drivePower;
    double strafePower;


    private void init_variables() {
        // Slider Position
        sliderhangposition = 1400;
        sliderHangSpec = 2500;
        // Vertical Slider Position
        sliderSamplePick = 0;
        sliderHighbasket = 3350;

        wristUpPosition = 0.85;
        wristDownPosition = 0;
        wristScoringPosition = 0.7;
        wristHangSpecPosition = 0.5;

        gripperClosedPosition = 1;
        gripperOpenPosition = 0;

        subIntakeOff = 0.5;
        subWristUp = 0;

        drivePower = 0.4;
        strafePower = 0.4;


    }


    @Override public void runOpMode()
    {
        // Initialize the robot hardware & Turn on telemetry
        robot.initialize(true,false);
        slider = hardwareMap.get(DcMotor.class, "slider");
        gripper = hardwareMap.get(Servo.class, "gripper");
        subIntake = hardwareMap.get(Servo.class, "subIntake");
        subWrist = hardwareMap.get(Servo.class, "subWrist");
        wrist = hardwareMap.get(Servo.class, "wrist");
        sliderSub = hardwareMap.get(DcMotor.class, "sliderSub");


        init_variables();
        Slider_init();
        SliderSub_init();
        subIntake.setPosition(subIntakeOff);
        subWrist.setPosition(subWristUp);
        wrist.setPosition(wristUpPosition);
        gripper.setPosition(gripperOpenPosition);

        sleep(3000);
        gripper.setPosition(gripperClosedPosition);

        waitForStart();
        robot.resetHeading();  // Reset heading to set a baseline for Auto

        // Run Auto if stop was not pressed.
        if (opModeIsActive())
        {
            Autonomus();
        }
    }


    private void setupForPickupSample() {
        wrist.setPosition(wristDownPosition);
        setSliderPosition(sliderSamplePick);
        sleep(300);
    }

    private void pickSample() {
        gripper.setPosition(gripperClosedPosition);
        sleep(300);
        wrist.setPosition(wristScoringPosition);
    }
    private void DropSample() {
        /*setSliderPosition(sliderHighbasket);
        sleep(2000);
        robot.drive( 5, drivePower, 0);
        wrist.setPosition(wristScoringPosition);*/
        gripper.setPosition(gripperOpenPosition);
        sleep(1000);
    }

    private void Autonomus() {
        // Set slider and wrist to hang position
        //Drive to sub to hang in high rung
        setSliderPosition(sliderhangposition);
        wrist.setPosition(wristScoringPosition);
        //sleep(1000);
        robot.drive(  25, 0.3, 0);
        robot.strafe( -6, strafePower, 0);
        robot.drive(  13, drivePower, 0);

        setSliderPosition(sliderHangSpec);
        sleep(500);
        gripper.setPosition(gripperOpenPosition);
        sleep(300);


        setSliderPosition(sliderSamplePick);
        wrist.setPosition(wristDownPosition);

        //move left to pick first sample
        robot.strafe( 57, strafePower, 0);
        robot.drive(  5, drivePower, 0);
        sleep(500);
        // robot.drive(  10, 0.80, 0.20);
        pickSample();

        setSliderPosition(sliderHighbasket);


        //turn to basket and move
        robot.turnTo(140, 0.45, 0);
        robot.drive( 33, drivePower, 0);

        //Open slider and drop in high basket
        DropSample();
        wrist.setPosition(wristUpPosition);
        robot.drive( -10, drivePower, 0);
        //reverse and close the slider
        robot.turnTo(0, 0.45, 0);
        setupForPickupSample();

        //pick second sample
        robot.strafe( -1.5, strafePower, 0);
        robot.drive( 15, drivePower, 0);
        sleep(500);
        pickSample();

        setSliderPosition(sliderHighbasket);

        // robot.strafe( 10, 0.80, 0.15);
        robot.turnTo(160, 0.45, 0);
        robot.drive( 27, drivePower, 0);

        //Open slider and drop in high basket
        DropSample();
        wrist.setPosition(wristUpPosition);
        robot.drive( -10, drivePower, 0);
        robot.turnTo(0, 0.45, 0);
        setupForPickupSample();
        wrist.setPosition(wristUpPosition);

        //Park

        robot.strafe( -34, strafePower, 0);
        robot.drive( 55, drivePower, 0);

       /* sleep(500);
        pickSample();

        robot.strafe( -6, strafePower, 0);
        setSliderPosition(sliderHighbasket);
        robot.turnTo(165, 0.45, 0);
        robot.drive( 28, drivePower, 0);
        //robot.turnTo(45, 0.45, 0);
        //Open slider and drop in high basket
        DropSample();*/

        robot.turnTo(-90, 0.45, 0);
        //turnoff all motors set to default
        wrist.setPosition(wristUpPosition);
        robot.drive( 25, drivePower, 0);
        slider.setPower(0);
        requestOpModeStop();
    }



    private void Slider_init() {
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setDirection(DcMotor.Direction.FORWARD);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider.setPower(0);
    }

    private void SliderSub_init() {
        sliderSub.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderSub.setDirection(DcMotor.Direction.FORWARD);
        sliderSub.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderSub.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderSub.setPower(0);
    }

    private void setSliderPosition(int sliderPosition) {
        slider.setTargetPosition(sliderPosition);
        slider.setPower(1);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}