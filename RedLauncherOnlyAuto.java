/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels", and two servos
 * which feed that launcher.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */

@Autonomous(name = "RedLauncherOnlyAuto", group = "StarterBot")
//@Disabled
public class RedLauncherOnlyAuto extends LinearOpMode {
    
    private static final double INTAKE_SPEED = 1.0; // 80% of max speed

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;
    
    final double INTAKE_TARGET_VELOCITY = 1125;
    final double INTAKE_MIN_VELOCITY = 1075;
    
    final double STOP_SPEED = 0;
    final double FULL_SPEED = 1.0;
    final double FEED_TIME_SECONDS = 0.5;

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intakeMotor = null;
    private Servo intakeServo = null;

    ElapsedTime feederTimer = new ElapsedTime();

    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    
    private void drive(double lfPower, double rfPower, double lbPower, double rbPower, int timeMs) {
        leftFrontDrive.setPower(lfPower);
        rightFrontDrive.setPower(rfPower);
        leftBackDrive.setPower(lbPower);
        rightBackDrive.setPower(rbPower);
        
        sleep(timeMs); //wait the given time
        
        //stop all motors after
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private LaunchState launchState;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    
     private static final double SERVO_FORWARD = 1.0;     // Full speed forward
    private static final double SERVO_BACKWARD = 0.0;    // Full speed backward
    private static final double SERVO_STOP = 0.5;       // Stopped
    private static final double SERVO_SLOW_FORWARD = 0.7;  // Slower forward
    private static final double SERVO_SLOW_BACKWARD = 0.3; // Slower backward

    @Override
    public void runOpMode() throws InterruptedException {
        
       launchState = LaunchState.IDLE;

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        
        launcher.setDirection(DcMotor.Direction.FORWARD);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeServo.setPosition(SERVO_STOP);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        waitForStart();

  if (opModeIsActive()); {
    //=== STEP 1: Drive Backwards For One Second ===
    drive(0.21,0.21, -0.21,-0.21,1000); 
    //=== STEP 1: Move forward ===
       launcher.setVelocity(LAUNCHER_TARGET_VELOCITY); 
    sleep(1500);//run launcher for 1500
   //=== STEP 2: FEED the ring === 
   intakeServo.setPosition(SERVO_SLOW_BACKWARD);
   sleep(300); //run feeder for 300
   
   intakeMotor.setPower(FULL_SPEED); //move ballz up
   sleep(4000); //runs for 4000 
   intakeMotor.setPower(STOP_SPEED); 
   
   //===MOVE TO GET FIRST PRE-STAGE ARTIFACTS===
   intakeServo.setPosition(SERVO_STOP);
   drive (.36,-.36,0,0,1000); //turns right (pivot point from back)
   drive (.27,.27,-.27,-.27,1000);//moves back
   sleep (200);
   drive (-.85,.85,-.85,.85,999); //strafe right
   sleep (200);
    intakeMotor.setPower(FULL_SPEED); //turns on intake
    intakeServo.setPosition(SERVO_FORWARD); //moves servo reverse to prevent ballz going thru
    drive (-.51,-.51,.51,.51,1000); //moves up to collect 2 ballz
    sleep(200);
    drive (-.25,-.25,.25,.25,1000);//moves up a bit more in case
    drive (-.1,-.1,.1,.1,1000);
    sleep (100);
   intakeMotor.setPower(STOP_SPEED);
   intakeServo.setPosition(SERVO_STOP);
   
   //===MOVES FIRST PRESTAGE ARTIFACTS TO DEPO==
    drive(.7,0.7,-.7,-.7,1000); //move back
    sleep(200);
    drive(0,0,0.315,-0.315, 1000); //turn left (pivot point from back)
    sleep(200);
    drive(-.74,-.74,.74,.74,1000);
    intakeMotor.setPower(FULL_SPEED);
    intakeServo.setPosition(SERVO_BACKWARD);
    sleep(4000);//4 secs 
    
    //===Get SECOND pre-stage #2===//
    intakeMotor.setPower (STOP_SPEED);
    intakeServo.setPosition(SERVO_STOP);
   drive(.43,-.43,0,0, 1000); //turn right (pivot from back)
   drive(.4,.4,-.4,-.4,1000);//back
   sleep(200);
   drive(-0.6,0.6,-0.6,0.6,1000); //strafe right
    drive(-0.55,0.55,-0.55,0.55,1000);
    drive(-0.5,0.5,-0.5,0.5,1000);
    
    intakeMotor.setPower(FULL_SPEED); //turns on intake
    intakeServo.setPosition(SERVO_FORWARD); //moves servo reverse to prevent ballz going thru
    drive (-.55,-.55,.55,.55,1000); //moves up to collect 2 ballz
    sleep(100);
    drive (-.25,-.25,.25,.25,1000);//moves up a bit more in case
    drive (-.1,-.1,.1,.1,1000);
    sleep(200);
    intakeMotor.setPower (STOP_SPEED);
    intakeServo.setPosition(SERVO_STOP);
   
      //=== STEP3: Stop Everything ===
   launcher.setVelocity(STOP_SPEED);
   telemetry.addData("Status", "Launch complete");
   telemetry.update();
   sleep(500);
}
    
   }}