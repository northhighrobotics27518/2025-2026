package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Dual Slide TEST")
public class ViperTest extends LinearOpMode  {

    DcMotorEx leftSlide;
    DcMotorEx rightSlide;

    @Override
    public void runOpMode() {

        leftSlide  = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        // Reverse ONE motor if slides are mirrored
        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx slide : new DcMotorEx[]{leftSlide, rightSlide}) {
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        waitForStart();

        while (opModeIsActive()) {

           int holdPosition = leftSlide.getCurrentPosition();

            if (Math.abs(gamepad1.left_stick_y) > 0.1) {
             // Manual control
             leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            double power = -gamepad1.left_stick_y * 0.6;
            leftSlide.setPower(power);
             rightSlide.setPower(power);

            holdPosition = leftSlide.getCurrentPosition();
            } else {
            // Hold position
            leftSlide.setTargetPosition(holdPosition);
            rightSlide.setTargetPosition(holdPosition);

            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftSlide.setPower(0.3);
            rightSlide.setPower(0.3);
}

            telemetry.addData("Left Encoder", leftSlide.getCurrentPosition());
            telemetry.addData("Right Encoder", rightSlide.getCurrentPosition());
            telemetry.addData("Difference",
            leftSlide.getCurrentPosition() - rightSlide.getCurrentPosition());
            telemetry.update();
        }
    }
}