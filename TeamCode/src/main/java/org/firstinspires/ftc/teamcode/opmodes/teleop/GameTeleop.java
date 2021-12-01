package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;
import static org.firstinspires.ftc.teamcode.Robot.Alliance.alliance;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.STOWED;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@TeleOp(name="Game Teleop")
public class GameTeleop extends LinearOpMode {
    Robot r;
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        r = new Robot(hardwareMap, gamepad1, gamepad2);
        drive = new SampleMecanumDrive(hardwareMap);


        waitForStart();

        r.getDeliveryControl().moveDelivery(STOWED);

        while (opModeIsActive()) {
            //Loop for main driving code (wheel control)
            r.driveControlLoop(drive, gamepad1);

            //Loop for control of the carousel
            r.carouselControlLoop(gamepad1);

            //Loop for the control of the delivery arm and intake.
            r.armControlLoopTeleOp(gamepad2, telemetry);

            //Switch the alliance in case of emergency
            //TODO: test the alliance switching
            r.switchAlliance(gamepad1);

            //Change the lights based on the presence of an element
            r.lightsLoop();

            telemetry.addData("Alliance", alliance);
            telemetry.addLine("Press A and B on gamepad 1 to switch alliance");
            telemetry.addData("Element loaded", r.getDeliveryControl().isElementLoaded());
            telemetry.addData("Distance readout", r.getDeliveryControl().getDistanceSensor().getDistance(MM));
            telemetry.update();

        }
    }

}
