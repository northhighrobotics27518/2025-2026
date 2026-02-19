package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "RedEncoderAuto", group = "StarterBot")
public class RedEncoderAuto extends LinearOpMode {

    // =============================
    // ======== CONSTANTS ==========
    // =============================

    static final double TICKS_PER_REV = 537.7;
    static final double WHEEL_DIAMETER_IN = 4.094;
    static final double TICKS_PER_INCH =
            TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN);

    static final double STRAFE_MULTIPLIER = 1.15;
    static final double TURN_TICKS_PER_DEGREE = 10.8;

    static final double LAUNCHER_TARGET_VELOCITY = 1145;
    final double LAUNCHER_MIN_VELOCITY = 1000;
    
    static final double FULL_SPEED = 1.0;
    static final double STOP_SPEED = 0.0;

    // ===== Servo speeds =====
    static final double SERVO_FORWARD = 1.0;
    static final double SERVO_BACKWARD = 0.0;
    static final double SERVO_STOP = 0.5;
    static final double SERVO_SLOW_FORWARD = 0.7;
    static final double SERVO_SLOW_BACKWARD = 0.3;

    // =============================
    // ======== HARDWARE ===========
    // =============================

    DcMotor leftFront, rightFront, leftBack, rightBack;
    DcMotorEx launcher;
    DcMotor intakeMotor;
    Servo intakeServo;

    @Override
    public void runOpMode() {

        leftFront  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFront = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBack   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBack  = hardwareMap.get(DcMotor.class, "right_back_drive");

        launcher    = hardwareMap.get(DcMotorEx.class, "launcher");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        
    
    intakeMotor.setDirection(DcMotor.Direction.REVERSE);

    leftFront.setDirection(DcMotor.Direction.REVERSE);
    leftBack.setDirection(DcMotor.Direction.REVERSE);

    rightFront.setDirection(DcMotor.Direction.FORWARD);
    rightBack.setDirection(DcMotor.Direction.FORWARD);


        DcMotor[] motors = {leftFront, rightFront, leftBack, rightBack};
        for (DcMotor m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeServo.setPosition(SERVO_STOP);

        telemetry.addLine("Initialized – RED SIDE");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        // =============================
        // ======== AUTONOMOUS =========
        // =============================

        // === STEP 1: Drive backwards ===
    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        
    encoderForward(0.3, 18);
    sleep(4000);//run launcher for 4 secs 
   //=== STEP 2: FEED the ring === 
   intakeServo.setPosition(SERVO_SLOW_BACKWARD);
   sleep(300); //run feeder for 3 secs
   
   intakeMotor.setPower(1.0); //move ballz up
   sleep(4000); //runs for 4000 
   intakeMotor.setPower(STOP_SPEED); 
   
   //===MOVE TO GET FIRST PRE-STAGE ARTIFACTS===
   intakeServo.setPosition(SERVO_STOP);

        // === MOVE TO first PRE-STAGE (RED) ===
        encoderTurn(0.35, -33);
        encoderForward(0.4, 15);
        sleep(100);
        encoderStrafe(0.45, 27);
        sleep(100);

        intakeMotor.setPower(FULL_SPEED);
        intakeServo.setPosition(SERVO_FORWARD);
        encoderForward(0.4, -25);
        sleep(100);
        encoderForward(0.4, -15);
        sleep(200);


        // === MOVE First PRE-STAGE TO DEPO (RED) ===
        encoderForward(0.5, 30);
        intakeMotor.setPower(STOP_SPEED);
        intakeServo.setPosition(SERVO_STOP);        
        sleep(100);
        encoderStrafe(0.45, -12);
        encoderTurn(0.45, 33);
        encoderForward(0.5, -19);

        intakeMotor.setPower(FULL_SPEED);
        intakeServo.setPosition(SERVO_BACKWARD);
        sleep(4000); //4 secs

        // === GET Second/Third PRE-STAGE (RED) ===
        intakeMotor.setPower(STOP_SPEED);
        intakeServo.setPosition(SERVO_STOP);

        encoderTurn(0.35, -36);
        encoderForward(0.4, 15);
        sleep(200);

        encoderStrafe(0.45, 47);

        intakeMotor.setPower(STOP_SPEED);
        intakeServo.setPosition(SERVO_STOP);

        // === STOP EVERYTHING ===
        launcher.setVelocity(0);

        telemetry.addLine("RED Auto Complete");
        telemetry.update();
        sleep(500);
    }

    // =============================
    // ===== ENCODER METHODS =======
    // =============================

    private void encoderForward(double speed, double inches) {
        int move = (int)(inches * TICKS_PER_INCH);
        setTargets(move, move, move, move);
        runToPosition(speed);
    }

    private void encoderStrafe(double speed, double inches) {
        int move = (int)(inches * TICKS_PER_INCH * STRAFE_MULTIPLIER);
        setTargets(-move, move, move, -move);
        runToPosition(speed);
    }

    private void encoderTurn(double speed, double degrees) {
        int move = (int)(degrees * TURN_TICKS_PER_DEGREE);
        setTargets(-move, move, -move, move);
        runToPosition(speed);
    }

    private void setTargets(int lf, int rf, int lb, int rb) {
        leftFront.setTargetPosition(leftFront.getCurrentPosition() + lf);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + rf);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() + lb);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() + rb);
    }

    private void runToPosition(double speed) {
        DcMotor[] motors = {leftFront, rightFront, leftBack, rightBack};
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(speed);
        }

        while (opModeIsActive()
                && leftFront.isBusy()
                && rightFront.isBusy()
                && leftBack.isBusy()
                && rightBack.isBusy()) {
            idle();
        }

        for (DcMotor m : motors) {
            m.setPower(0);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        sleep(75);
    }
}