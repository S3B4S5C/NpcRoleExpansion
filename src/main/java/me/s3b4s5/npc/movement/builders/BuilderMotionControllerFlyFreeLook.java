package me.s3b4s5.npc.movement.builders;

import com.google.gson.JsonElement;
import com.hypixel.hytale.server.npc.asset.builder.BuilderDescriptorState;
import com.hypixel.hytale.server.npc.asset.builder.BuilderSupport;
import com.hypixel.hytale.server.npc.movement.controllers.MotionController;
import com.hypixel.hytale.server.npc.movement.controllers.MotionControllerFly;
import com.hypixel.hytale.server.npc.movement.controllers.builders.BuilderMotionControllerFly;
import me.s3b4s5.npc.movement.controllers.MotionControllerFlyFreeLook;

import javax.annotation.Nonnull;

/**
 * JSON builder for {@link MotionControllerFlyFreeLook}.
 *
 * <p>This builder registers a custom motion controller type intended for flying actors where
 * translation direction can be driven independently from look direction (yaw/pitch).</p>
 *
 * <h2>Usage</h2>
 * <p>Referenced from a role JSON inside {@code MotionControllerList}:</p>
 *
 * <pre>{@code
 * "MotionControllerList": [
 *   { "Type": "FlyFreeLook", ... }
 * ]
 * }</pre>
 *
 * <h2>Compatibility</h2>
 * <ul>
 *   <li>{@link #getClassType()} returns {@link MotionControllerFly} for compatibility with systems that
 *       check for the vanilla fly controller type, while the actual built instance is {@link MotionControllerFlyFreeLook}.</li>
 *   <li>This controller is designed to work well with body motions that provide explicit steering translation
 *       and optionally separate look control (e.g. {@code Watch} head motion).</li>
 * </ul>
 *
 * <h2>Configuration</h2>
 * <p>This builder currently delegates to {@link BuilderMotionControllerFly} and does not introduce additional
 * custom JSON keys beyond the standard fly parameters (speed, acceleration/deceleration, roll, turn speed, etc.).</p>
 *
 * <h2>Type id</h2>
 * <p>The registered JSON type string is {@code "FlyFreeLook"}.</p>
 */
public class BuilderMotionControllerFlyFreeLook extends BuilderMotionControllerFly {

    @Nonnull
    @Override
    public MotionControllerFlyFreeLook build(@Nonnull BuilderSupport builderSupport) {
        return new MotionControllerFlyFreeLook(builderSupport, this);
    }

    @Nonnull
    @Override
    public String getShortDescription() {
        return "Flight controller that can move independently from look direction (Summons)";
    }

    @Nonnull
    @Override
    public String getLongDescription() {
        return getShortDescription();
    }

    @Nonnull
    @Override
    public BuilderDescriptorState getBuilderDescriptorState() {
        return BuilderDescriptorState.WorkInProgress;
    }

    @Nonnull
    @Override
    public Class<? extends MotionController> getClassType() {
        return MotionControllerFly.class;
    }

    @Nonnull
    @Override
    public BuilderMotionControllerFlyFreeLook readConfig(@Nonnull JsonElement data) {
        super.readConfig(data);
        return this;
    }

    @Nonnull
    @Override
    public String getType() {
        return "FlyFreeLook";
    }
}