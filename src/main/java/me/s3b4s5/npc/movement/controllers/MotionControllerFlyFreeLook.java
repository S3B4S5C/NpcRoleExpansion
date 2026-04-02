package me.s3b4s5.npc.movement.controllers;

import com.hypixel.hytale.component.ComponentAccessor;
import com.hypixel.hytale.component.Ref;
import com.hypixel.hytale.math.util.MathUtil;
import com.hypixel.hytale.math.util.TrigMathUtil;
import com.hypixel.hytale.math.vector.Vector3d;
import com.hypixel.hytale.server.core.modules.physics.util.PhysicsMath;
import com.hypixel.hytale.server.core.universe.world.storage.EntityStore;
import com.hypixel.hytale.server.npc.asset.builder.BuilderSupport;
import com.hypixel.hytale.server.npc.movement.MotionKind;
import com.hypixel.hytale.server.npc.movement.Steering;
import com.hypixel.hytale.server.npc.movement.controllers.MotionControllerFly;
import com.hypixel.hytale.server.npc.movement.controllers.builders.BuilderMotionControllerFly;
import com.hypixel.hytale.server.npc.role.Role;
import com.hypixel.hytale.server.npc.util.NPCPhysicsMath;

import javax.annotation.Nonnull;

/**
 * Flight motion controller that decouples movement direction from look direction.
 *
 * <h2>What it changes compared to vanilla Fly</h2>
 * Vanilla flying controllers often assume that "where you look" and "where you move" are tightly coupled.
 * This controller allows:
 * <ul>
 *   <li>Body motion to drive translation vector (movement) independently.</li>
 *   <li>Head motion (yaw/pitch) to keep tracking a target while strafing / orbiting / backing away.</li>
 * </ul>
 *
 * <h2>How it works</h2>
 * {@link #computeMove} interprets {@link Steering} as:
 * <ul>
 *   <li>{@code steering.translation}: desired movement direction and magnitude (relative).</li>
 *   <li>{@code steering.yaw/pitch/roll}: desired facing (if present) or derived from translation when absent.</li>
 * </ul>
 * When translation is present, the controller computes a movement yaw/pitch from the translation vector, but it can keep
 * the facing yaw/pitch from steering (e.g. "Watch" head motion) so the entity can move sideways/backwards while looking at
 * the target.
 *
 * <h2>Speed limiting</h2>
 * The controller respects:
 * <ul>
 *   <li>Acceleration / deceleration constraints.</li>
 *   <li>Pitch-based speed limits (climb/sink angles).</li>
 *   <li>Minimum air speed and environmental constraints (water / gravity).</li>
 * </ul>
 *
 * <h2>Intended usage</h2>
 * This controller is a good match for behaviors like:
 * <ul>
 *   <li>Maintain distance on a ring (orbit) around a target.</li>
 *   <li>Strafing while watching.</li>
 *   <li>Drones that should track a target without constantly "turning into" the movement direction.</li>
 * </ul>
 *
 * <h2>Compatibility</h2>
 * Builder reports {@link MotionControllerFly} as the class type for compatibility with vanilla validation rules,
 * while {@link #getType()} returns a distinct type string for configuration.
 */
public class MotionControllerFlyFreeLook extends MotionControllerFly {
    public static final String TYPE = "FlyFreeLook";
    private static final double EPS_DIR = 1.0E-12;
    private static final double EPS_LEN = 1.0E-6;

    public MotionControllerFlyFreeLook(@Nonnull BuilderSupport builderSupport,
                                       @Nonnull BuilderMotionControllerFly builder) {
        super(builderSupport, builder);
    }

    @Nonnull
    @Override
    public String getType() {
        return TYPE;
    }

    @Override
    protected double computeMove(@Nonnull Ref<EntityStore> ref,
                                 @Nonnull Role role,
                                 @Nonnull Steering steering,
                                 double dt,
                                 @Nonnull Vector3d translation,
                                 @Nonnull ComponentAccessor<EntityStore> componentAccessor) {

        if (!this.forceVelocity.equals(Vector3d.ZERO) || !this.appliedVelocities.isEmpty()) {
            return super.computeMove(ref, role, steering, dt, translation, componentAccessor);
        }

        this.saveMotionKind();
        this.setMotionKind(this.inWater() ? MotionKind.MOVING : MotionKind.FLYING);

        this.moveProbe.probePosition(ref, this.collisionBoundingBox, this.position, this.collisionResult, componentAccessor);
        this.currentRelativeSpeed = steering.getSpeed();

        if (!this.isAlive(ref, componentAccessor)) {
            this.forceVelocity.assign(Vector3d.ZERO);
            this.appliedVelocities.clear();
        }

        double maxFallSpeed = this.moveProbe.isInWater() ? this.maxSinkSpeedFluid : this.maxFallSpeed;
        boolean onGround = this.onGround();

        if (NPCPhysicsMath.near(this.lastVelocity, Vector3d.ZERO)) {
            PhysicsMath.vectorFromAngles(this.getYaw(), this.getPitch(), this.lastVelocity);
            this.lastSpeed = 0.0;
        }

        if (this.canAct(ref, componentAccessor)) {
            translation.assign(steering.getTranslation());
            double relSpeed = steering.hasTranslation() ? translation.length() : 0.0;

            float yaw = PhysicsMath.normalizeAngle(this.getYaw());
            float pitch = PhysicsMath.normalizeTurnAngle(this.getPitch());

            double dirX = translation.x;
            double dirY = translation.y;
            double dirZ = translation.z;

            double dotXZ = dirX * dirX + dirZ * dirZ;
            double dotXYZ = dotXZ + dirY * dirY;

            boolean hasMove = dotXYZ >= EPS_DIR;

            float moveYaw;
            float movePitch;

            if (hasMove) {
                if (dotXZ >= EPS_DIR) {
                    moveYaw = PhysicsMath.headingFromDirection(dirX, dirZ);
                    movePitch = TrigMathUtil.atan2(dirY, Math.sqrt(dotXZ));
                } else {
                    moveYaw = yaw;
                    movePitch = (dirY >= 0.0) ? ((float) Math.PI / 2F) : (-(float) Math.PI / 2F);
                }
            } else {
                moveYaw = yaw;
                movePitch = (this.autoLevel ? 0.0F : pitch);
            }

            float expYaw = steering.hasYaw() ? steering.getYaw() : (hasMove ? moveYaw : yaw);
            float expPitch = steering.hasPitch() ? steering.getPitch()
                    : (hasMove ? movePitch : (this.autoLevel ? 0.0F : pitch));

            steering.clearYaw();
            steering.clearPitch();

            expPitch = MathUtil.clamp(expPitch, -this.maxSinkAngle, this.maxClimbAngle);

            float turnYaw = NPCPhysicsMath.turnAngle(yaw, expYaw);
            float turnPitch = NPCPhysicsMath.turnAngle(pitch, expPitch);

            float maxRotationAngle = (float) (this.getCurrentMaxBodyRotationSpeed() * dt);
            turnYaw = NPCPhysicsMath.clampRotation(turnYaw, maxRotationAngle);
            turnPitch = NPCPhysicsMath.clampRotation(turnPitch, maxRotationAngle);

            float newYaw = PhysicsMath.normalizeAngle(yaw + turnYaw);
            float newPitch = PhysicsMath.normalizeTurnAngle(pitch + turnPitch);

            float clampedMovePitch = MathUtil.clamp(movePitch, -this.maxSinkAngle, this.maxClimbAngle);

            double speedLimit = this.computeMaxSpeedFromPitch(clampedMovePitch);
            double desiredSpeed = relSpeed * speedLimit;

            double minSpeed = Math.max(this.minAirSpeed, this.lastSpeed - this.deceleration * dt);
            double maxSpeed = this.lastSpeed + this.acceleration * dt;
            desiredSpeed = (maxSpeed < minSpeed) ? minSpeed : MathUtil.clamp(desiredSpeed, minSpeed, maxSpeed);

            if (!hasMove || desiredSpeed <= 0.0) {
                translation.assign(Vector3d.ZERO);
            } else {
                PhysicsMath.vectorFromAngles(moveYaw, clampedMovePitch, translation);
                translation.normalize();
            }

            double mX = this.lastVelocity.z;
            double mZ = -this.lastVelocity.x;
            double mL = Math.sqrt(mX * mX + mZ * mZ);

            float desiredRoll = 0.0F;
            if (mL > EPS_LEN && !translation.equals(Vector3d.ZERO)) {
                float rollTurnCosine = (float) (NPCPhysicsMath.dotProduct(
                        mX, 0.0, mZ,
                        translation.x, translation.y, translation.z
                ) / mL);

                float maxRollTurnAngle = (float) ((double) this.maxTurnSpeed * dt);
                float maxRollTurnCosine = TrigMathUtil.sin(maxRollTurnAngle);
                float rollTurnStrength = (maxRollTurnCosine > 1.0E-6F) ? (rollTurnCosine / maxRollTurnCosine) : 0.0F;

                double speedFactor = desiredSpeed / Math.max(EPS_LEN, speedLimit);
                desiredRoll = this.maxRollAngle
                        * MathUtil.clamp(rollTurnStrength, -1.0F, 1.0F)
                        * MathUtil.clamp((float) speedFactor, 0.0F, 1.0F);
            }

            float dampedRoll = MathUtil.clamp(
                    this.rollDamping * this.lastRoll + (1.0F - this.rollDamping) * desiredRoll,
                    -this.maxRollAngle, this.maxRollAngle
            );

            float deltaRoll = (float) ((double) this.maxRollSpeed * dt);
            float constrainedRoll = MathUtil.clamp(dampedRoll, this.lastRoll - deltaRoll, this.lastRoll + deltaRoll);
            this.lastRoll = constrainedRoll;

            steering.setYaw(newYaw);
            steering.setPitch(newPitch);
            steering.setRoll(constrainedRoll);

            if (desiredSpeed == 0.0 || translation.equals(Vector3d.ZERO)) {
                translation.assign(Vector3d.ZERO);
            } else {
                translation.scale(desiredSpeed * this.effectHorizontalSpeedMultiplier);
            }

            this.lastVelocity.assign(translation);
            this.lastSpeed = desiredSpeed;

            translation.scale(dt);

            if (this.debugModeValidateMath && !NPCPhysicsMath.isValid(translation)) {
                throw new IllegalArgumentException(String.valueOf(translation));
            }

            return dt;
        }

        steering.setYaw(this.getYaw());
        steering.setPitch(this.getPitch());
        steering.setRoll(this.getRoll());

        if (onGround) {
            this.setMotionKind(MotionKind.STANDING);
            this.lastVelocity.assign(Vector3d.ZERO);
            this.lastSpeed = 0.0;
            return dt;
        }

        this.setMotionKind(MotionKind.DROPPING);

        translation.y = NPCPhysicsMath.gravityDrag(this.lastVelocity.y, this.gravity, dt, maxFallSpeed);
        double diffSpeed = maxFallSpeed - translation.y;

        if (!(diffSpeed <= 0.0) && !this.isObstructed()) {
            double scale = translation.x * translation.x + translation.z * translation.z;
            if (diffSpeed * diffSpeed < scale) {
                scale = Math.sqrt(scale / diffSpeed);
                translation.x = this.lastVelocity.x * scale;
                translation.z = this.lastVelocity.z * scale;
            } else {
                translation.x = this.lastVelocity.x;
                translation.z = this.lastVelocity.z;
            }
        } else {
            translation.x = 0.0;
            translation.z = 0.0;
        }

        this.lastVelocity.assign(translation);
        this.lastSpeed = this.lastVelocity.length();

        translation.scale(dt);

        if (this.debugModeValidateMath && !NPCPhysicsMath.isValid(translation)) {
            throw new IllegalArgumentException(String.valueOf(translation));
        }

        return dt;
    }
}
