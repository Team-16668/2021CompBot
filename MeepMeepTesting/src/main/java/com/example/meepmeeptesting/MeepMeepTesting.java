package com.example.meepmeeptesting;

import static com.noahbres.meepmeep.core.ExtensionsKt.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

public class MeepMeepTesting {
    public static void main(String[] args) {
        // System.setProperty("sun.java2d.opengl", "true");

        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep mm = new MeepMeep(800)
                // Set field image
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(61.58375, 52.48291908330528, toRadians(315.445), toRadians(278.4305333333333), 15)
                .setBotDimensions(12.5, 14.2)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(5, -38, toRadians(335)))
                                .splineTo(new Vector2d(8, -48), toRadians(270))
                                .splineToSplineHeading(new Pose2d(18, -67, toRadians(0)), toRadians(0))
                                .splineToConstantHeading(new Vector2d(46, -67), 0)
                                .build()
                )

//                .followTrajectorySequence(drive ->
//                    drive.trajectorySequenceBuilder(new Pose2d(42, -63, toRadians(0)))
//                            .setReversed(true)
//                            .splineToConstantHeading(new Vector2d(36, -67), Math.toRadians(180))
//                            .splineToConstantHeading(new Vector2d(18, -67), toRadians(180))
//                            //.splineToSplineHeading(new Pose2d(9, -60, toRadians(270)), toRadians(90))
//                            .splineToSplineHeading(new Pose2d(3, -38, toRadians(335)), toRadians(135))
//                            .setReversed(false)
//                            .build())
                .start();
    }
}