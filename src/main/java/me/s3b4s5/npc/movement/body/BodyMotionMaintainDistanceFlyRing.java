package me.s3b4s5.npc.movement.body;

import com.hypixel.hytale.component.ComponentAccessor;
import com.hypixel.hytale.component.ComponentType;
import com.hypixel.hytale.component.Ref;
import com.hypixel.hytale.math.util.MathUtil;
import com.hypixel.hytale.math.vector.Vector3d;
import com.hypixel.hytale.math.vector.Vector3f;
import com.hypixel.hytale.server.core.asset.type.model.config.Model;
import com.hypixel.hytale.server.core.modules.entity.component.ModelComponent;
import com.hypixel.hytale.server.core.modules.entity.component.TransformComponent;
import com.hypixel.hytale.server.core.universe.world.storage.EntityStore;
import com.hypixel.hytale.server.npc.asset.builder.BuilderSupport;
import com.hypixel.hytale.server.npc.corecomponents.movement.BodyMotionFindBase;
import com.hypixel.hytale.server.npc.movement.Steering;
import com.hypixel.hytale.server.npc.movement.controllers.MotionController;
import com.hypixel.hytale.server.npc.movement.controllers.MotionControllerFly;
import com.hypixel.hytale.server.npc.movement.steeringforces.SteeringForcePursue;
import com.hypixel.hytale.server.npc.navigation.AStarBase;
import com.hypixel.hytale.server.npc.navigation.AStarNode;
import com.hypixel.hytale.server.npc.navigation.AStarWithTarget;
import com.hypixel.hytale.server.npc.role.Role;
import com.hypixel.hytale.server.npc.role.support.MarkedEntitySupport;
import com.hypixel.hytale.server.npc.sensorinfo.IPositionProvider;
import com.hypixel.hytale.server.npc.sensorinfo.InfoProvider;
import com.hypixel.hytale.server.npc.sensorinfo.parameterproviders.DoubleParameterProvider;
import com.hypixel.hytale.server.npc.sensorinfo.parameterproviders.ParameterProvider;
import com.hypixel.hytale.server.npc.util.NPCPhysicsMath;
import me.s3b4s5.npc.movement.builders.BuilderBodyMotionMaintainDistanceFlyRing;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;
import java.util.concurrent.ThreadLocalRandom;

/**
 * Maintains an NPC at a configurable XZ distance band around a target, using a flying motion controller.
 *
 * <h2>Concept</h2>
 * This body motion drives the NPC towards a point on a "ring" around the target:
 * <ul>
 *   <li>Choose a desired ring radius within {@code DesiredDistanceRangeXZ} (with hysteresis via {@code MoveThreshold}).</li>
 *   <li>Choose/maintain a ring angle ({@code orbitAngleRad}).</li>
 *   <li>Compute a 3D goal point: {@code targetPos + ringOffset(angle, radius)} plus a vertical offset.</li>
 * </ul>
 *
 * <h2>Target source</h2>
 * The target position is provided by the active {@link InfoProvider}.
 * This body motion requires {@code Feature.AnyPosition} and expects an upstream sensor (e.g. {@code Target}, {@code Leash})
 * to provide a position each tick.
 *
 * <h2>Pathing and obstacle handling</h2>
 * This class extends {@link BodyMotionFindBase} and can:
 * <ul>
 *   <li>Use probe movement to estimate progress to the current goal.</li>
 *   <li>If progress is too small, "replan" the ring angle by sampling multiple candidate angles.</li>
 *   <li>Throttle path recomputation unless the goal moves significantly.</li>
 * </ul>
 *
 * <h2>Required MotionController</h2>
 * This motion is designed to run with {@link MotionControllerFly}.
 * If another controller is active, the motion will not compute.
 */
public class BodyMotionMaintainDistanceFlyRing extends BodyMotionFindBase<AStarWithTarget> {

    protected static final ComponentType<EntityStore, TransformComponent> TRANSFORM_COMPONENT_TYPE =
            TransformComponent.getComponentType();

    protected static final ComponentType<EntityStore, ModelComponent> MODEL_COMPONENT_TYPE =
            ModelComponent.getComponentType();

    protected static final double EPS = 1.0e-6;
    private static final float FALLBACK_EYE_HEIGHT = 1.8f;

    private static final double ARRIVE_EPS_3D =
            Double.parseDouble(System.getProperty("hylamity.flyring.arrive_eps_3d", "0.45"));

    private static final double SOFT_STOP_EPS_3D =
            Double.parseDouble(System.getProperty("hylamity.flyring.soft_stop_eps_3d", "0.12"));

    private static final double SOFT_MIN_SPEED_SCALE =
            Double.parseDouble(System.getProperty("hylamity.flyring.soft_min_speed_scale", "0.18"));

    private static final double ANGLE_STEP_DEG =
            Double.parseDouble(System.getProperty("hylamity.flyring.angle_step_deg", "45"));

    private static final int ANGLE_TRIES =
            Integer.parseInt(System.getProperty("hylamity.flyring.angle_tries", "8"));

    private static final double REPLAN_COOLDOWN_SECONDS =
            Double.parseDouble(System.getProperty("hylamity.flyring.replan_cd", "0.25"));

    private static final double MIN_PROGRESS_SQ =
            Double.parseDouble(System.getProperty("hylamity.flyring.min_progress_sq", "0.02"));

    private static final double GOAL_RECOMPUTE_SQ =
            Double.parseDouble(System.getProperty("hylamity.flyring.goal_recompute_sq", "0.36"));

    private static final double GOAL_WAIT_SQ =
            Double.parseDouble(System.getProperty("hylamity.flyring.goal_wait_sq", "0.04"));

    private static final double SPEED_SCALE_RESPONSE =
            Double.parseDouble(System.getProperty("hylamity.flyring.speed_scale_response", "8.0"));

    private final SteeringForcePursue seek = new SteeringForcePursue();

    protected final double[] initialDesiredDistanceRangeXZ;
    protected final double moveThreshold;
    @Nonnull
    protected final double[] thresholdDistanceRangeSquared;

    protected final double targetDistanceFactor;
    protected final double relativeForwardsSpeed;
    protected final double relativeBackwardsSpeed;
    protected final double moveTowardsSlowdownThreshold;

    protected final int minRangeProviderSlot;
    protected final int maxRangeProviderSlot;

    protected final double[] desiredDistanceRangeXZ = new double[2];

    protected boolean approaching;
    protected boolean movingAway;

    protected final Vector3d targetPosition = new Vector3d();
    protected final Vector3d targetLookPosition = new Vector3d();

    protected final Vector3d desiredRingPoint = new Vector3d();
    protected final Vector3d probeTmp = new Vector3d();
    protected final Vector3d goalPosition = new Vector3d();
    protected final Vector3d lastPathedGoal = new Vector3d(Double.MIN_VALUE, Double.MIN_VALUE, Double.MIN_VALUE);

    protected DoubleParameterProvider cachedMinRangeProvider;
    protected DoubleParameterProvider cachedMaxRangeProvider;
    protected boolean initialised;

    private double orbitAngleRad = Double.NaN;
    private double replanCooldown = 0.0;

    private boolean waitForGoalMovement = false;
    private double dtCache = 0.0;

    private final double[] yOffsets;
    private final double[] yOffsetChangeIntervalRange;
    private double currentYOffset;
    private double yOffsetTimer;

    private double speedScale = 1.0;

    private final boolean hardStop;
    private final double stopEps3d;

    private final boolean lockBehindTarget;
    private final double behindBlend;
    private final double minTargetSpeed;

    private final Vector3d lastTargetPos = new Vector3d(Double.NaN, Double.NaN, Double.NaN);
    private boolean lastTargetValid = false;

    private double desiredAnchorAngleRad = Double.NaN;

    public BodyMotionMaintainDistanceFlyRing(@Nonnull BuilderBodyMotionMaintainDistanceFlyRing builder,
                                             @Nonnull BuilderSupport support) {
        super(builder, support, new AStarWithTarget());

        this.hardStop = builder.isHardStop(support);
        this.stopEps3d = this.hardStop ? ARRIVE_EPS_3D : Math.max(0.01, SOFT_STOP_EPS_3D);

        this.lockBehindTarget = builder.isLockBehindTarget(support);
        this.behindBlend = clamp(builder.getBehindBlend(support), 0.0, 1.0);
        this.minTargetSpeed = Math.max(0.0, builder.getMinTargetSpeed(support));

        this.initialDesiredDistanceRangeXZ = builder.getDesiredDistanceRangeXZ(support);
        this.desiredDistanceRangeXZ[0] = this.initialDesiredDistanceRangeXZ[0];
        this.desiredDistanceRangeXZ[1] = this.initialDesiredDistanceRangeXZ[1];

        this.yOffsets = builder.getYOffsets(support);
        this.yOffsetChangeIntervalRange = builder.getYOffsetChangeIntervalRange(support);
        this.currentYOffset = pickYOffset(this.yOffsets);
        this.yOffsetTimer = nextYOffsetInterval(this.yOffsetChangeIntervalRange);

        this.targetDistanceFactor = builder.getTargetDistanceFactor(support);
        this.moveThreshold = builder.getMoveThreshold(support);

        double min = Math.max(0.0, this.initialDesiredDistanceRangeXZ[0] - this.moveThreshold);
        double max = this.initialDesiredDistanceRangeXZ[1] + this.moveThreshold;
        this.thresholdDistanceRangeSquared = new double[2];
        this.thresholdDistanceRangeSquared[0] = min * min;
        this.thresholdDistanceRangeSquared[1] = max * max;

        this.relativeForwardsSpeed = builder.getRelativeForwardsSpeed(support);
        this.relativeBackwardsSpeed = builder.getRelativeBackwardsSpeed(support);
        this.moveTowardsSlowdownThreshold = builder.getMoveTowardsSlowdownThreshold(support);

        this.minRangeProviderSlot = support.getParameterSlot(BuilderBodyMotionMaintainDistanceFlyRing.MIN_RANGE_PARAMETER);
        this.maxRangeProviderSlot = support.getParameterSlot(BuilderBodyMotionMaintainDistanceFlyRing.MAX_RANGE_PARAMETER);

        double slow = Math.max(this.moveTowardsSlowdownThreshold, this.stopEps3d * 2.0);
        this.seek.setDistances(slow, this.stopEps3d);
        this.seek.setFalloff(1.0);
    }

    private static double clamp(double v, double min, double max) {
        return v < min ? min : (Math.min(v, max));
    }

    private static double wrapRadians(double a) {
        double twoPi = Math.PI * 2.0;
        a %= twoPi;
        if (a < 0) a += twoPi;
        return a;
    }

    private static double shortestAngleDelta(double from, double to) {
        double diff = wrapRadians(to - from);
        if (diff > Math.PI) diff -= (Math.PI * 2.0);
        return diff;
    }

    private static double lerpAngle(double from, double to, double t) {
        return wrapRadians(from + shortestAngleDelta(from, to) * clamp(t, 0.0, 1.0));
    }

    private void ensureOrbitAngleInit() {
        if (!Double.isNaN(this.orbitAngleRad)) return;

        if (this.lockBehindTarget && !Double.isNaN(this.desiredAnchorAngleRad)) {
            this.orbitAngleRad = this.desiredAnchorAngleRad;
            return;
        }

        this.orbitAngleRad = ThreadLocalRandom.current().nextDouble(0.0, Math.PI * 2.0);
    }

    private void updateYOffset(double dt) {
        if (this.yOffsets == null || this.yOffsets.length <= 1) return;

        double maxI = this.yOffsetChangeIntervalRange != null && this.yOffsetChangeIntervalRange.length >= 2
                ? this.yOffsetChangeIntervalRange[1]
                : 0.0;
        if (maxI <= 0.0) return;

        this.yOffsetTimer -= dt;
        if (this.yOffsetTimer > 0.0) return;

        this.currentYOffset = pickYOffset(this.yOffsets);
        this.yOffsetTimer = nextYOffsetInterval(this.yOffsetChangeIntervalRange);
    }

    private static double pickYOffset(double[] ys) {
        if (ys == null || ys.length == 0) return 0.0;
        int i = ThreadLocalRandom.current().nextInt(ys.length);
        return ys[i];
    }

    private static double nextYOffsetInterval(double[] range) {
        if (range == null || range.length < 2) return 0.0;
        double a = Math.max(0.0, range[0]);
        double b = Math.max(0.0, range[1]);
        if (b < a) {
            double t = a;
            a = b;
            b = t;
        }
        if (b <= 0.0) return 0.0;
        if (b == a) return b;
        return ThreadLocalRandom.current().nextDouble(a, b);
    }

    private void updateBehindAnchor(@Nonnull Vector3d selfPos) {
        this.desiredAnchorAngleRad = Double.NaN;
        if (!this.lockBehindTarget) return;

        double fallbackRadialAngle = Math.atan2(
                selfPos.z - this.targetPosition.z,
                selfPos.x - this.targetPosition.x
        );

        if (!this.lastTargetValid || Double.isNaN(this.lastTargetPos.x)) {
            this.desiredAnchorAngleRad = fallbackRadialAngle;
            return;
        }

        double dx = this.targetPosition.x - this.lastTargetPos.x;
        double dz = this.targetPosition.z - this.lastTargetPos.z;

        double dist = Math.sqrt(dx * dx + dz * dz);
        double dt = Math.max(this.dtCache, 1.0e-6);
        double speed = dist / dt;

        if (dist <= EPS || speed < this.minTargetSpeed) {
            this.desiredAnchorAngleRad = fallbackRadialAngle;
            return;
        }

        double bx = -dx / dist;
        double bz = -dz / dist;

        this.desiredAnchorAngleRad = Math.atan2(bz, bx);

        if (Double.isNaN(this.orbitAngleRad)) {
            this.orbitAngleRad = this.desiredAnchorAngleRad;
        } else if (this.behindBlend > 0.0) {
            this.orbitAngleRad = lerpAngle(this.orbitAngleRad, this.desiredAnchorAngleRad, this.behindBlend);
        } else {
            this.orbitAngleRad = this.desiredAnchorAngleRad;
        }
    }

    private void computeRingPoint(@Nonnull Vector3d out, @Nonnull Vector3d targetPos, double ringDist, double angleRad) {
        double cx = Math.cos(angleRad) * ringDist;
        double sz = Math.sin(angleRad) * ringDist;
        out.assign(targetPos.x + cx, targetPos.y + this.currentYOffset, targetPos.z + sz);
    }

    private double probeProgressSq(@Nonnull Ref<EntityStore> ref,
                                   @Nonnull MotionController motionController,
                                   @Nonnull Vector3d from,
                                   @Nonnull Vector3d to,
                                   @Nonnull ComponentAccessor<EntityStore> componentAccessor,
                                   @Nonnull Vector3d outProbe) {
        this.probeMoveData.setPosition(from).setTargetPosition(to);
        motionController.probeMove(ref, this.probeMoveData, componentAccessor);
        outProbe.assign(this.probeMoveData.probePosition);
        double dx = outProbe.x - from.x;
        double dy = outProbe.y - from.y;
        double dz = outProbe.z - from.z;
        return dx * dx + dy * dy + dz * dz;
    }

    private void maybeReplanAngle(@Nonnull Ref<EntityStore> ref,
                                  @Nonnull MotionController motionController,
                                  @Nonnull Vector3d selfPos,
                                  @Nonnull ComponentAccessor<EntityStore> componentAccessor,
                                  double ringDist) {
        if (this.replanCooldown > 0.0) return;

        double base = (!Double.isNaN(this.desiredAnchorAngleRad) ? this.desiredAnchorAngleRad : this.orbitAngleRad);
        if (Double.isNaN(base)) base = this.orbitAngleRad;

        double step = Math.toRadians(ANGLE_STEP_DEG);

        double bestAngle = base;
        double bestProgSq = -1.0;

        for (int i = 0; i <= ANGLE_TRIES; i++) {
            double candAngle;
            if (i == 0) candAngle = base;
            else {
                int k = (i + 1) / 2;
                double sign = (i % 2 == 1) ? 1.0 : -1.0;
                candAngle = wrapRadians(base + sign * k * step);
            }

            computeRingPoint(this.desiredRingPoint, this.targetPosition, ringDist, candAngle);
            double progSq = probeProgressSq(ref, motionController, selfPos, this.desiredRingPoint, componentAccessor, this.probeTmp);

            if (progSq > bestProgSq) {
                bestProgSq = progSq;
                bestAngle = candAngle;
            }
        }

        this.orbitAngleRad = bestAngle;
        this.replanCooldown = REPLAN_COOLDOWN_SECONDS;
    }

    private boolean canReachGoal(@Nonnull Ref<EntityStore> ref,
                                 @Nonnull MotionController motionController,
                                 @Nonnull Vector3d from,
                                 @Nonnull Vector3d to,
                                 @Nonnull ComponentAccessor<EntityStore> componentAccessor) {
        this.probeMoveData.setPosition(from).setTargetPosition(to);
        motionController.probeMove(ref, this.probeMoveData, componentAccessor);
        double epsSq = this.stopEps3d * this.stopEps3d;
        double d2 = this.probeMoveData.probePosition.distanceSquaredTo(to);
        return d2 <= epsSq;
    }

    @Override
    public boolean computeSteering(@Nonnull Ref<EntityStore> ref,
                                   @Nonnull Role role,
                                   @Nullable InfoProvider infoProvider,
                                   double dt,
                                   @Nonnull Steering desiredSteering,
                                   @Nonnull ComponentAccessor<EntityStore> componentAccessor) {
        this.dtCache = dt;
        boolean out = super.computeSteering(ref, role, infoProvider, dt, desiredSteering, componentAccessor);
        if (out && infoProvider != null && infoProvider.hasPosition()) {
            this.lookAtTarget(ref, role, desiredSteering, componentAccessor);
        }
        return out;
    }

    @Override
    protected boolean canComputeMotion(@Nonnull Ref<EntityStore> ref,
                                       @Nonnull Role role,
                                       @Nullable InfoProvider infoProvider,
                                       @Nonnull ComponentAccessor<EntityStore> componentAccessor) {
        MotionController motionController = role.getActiveMotionController();
        if (!motionController.matchesType(MotionControllerFly.class)) {
            role.setBackingAway(false);
            return false;
        }

        if (!this.initialised) {
            if (infoProvider != null) {
                ParameterProvider p = infoProvider.getParameterProvider(this.minRangeProviderSlot);
                if (p instanceof DoubleParameterProvider) this.cachedMinRangeProvider = (DoubleParameterProvider) p;

                p = infoProvider.getParameterProvider(this.maxRangeProviderSlot);
                if (p instanceof DoubleParameterProvider) this.cachedMaxRangeProvider = (DoubleParameterProvider) p;
            }
            this.initialised = true;
        }

        boolean recalcMin = false;

        if (this.cachedMinRangeProvider != null) {
            double value = this.cachedMinRangeProvider.getDoubleParameter();
            this.desiredDistanceRangeXZ[0] = (value != -Double.MAX_VALUE) ? value : this.initialDesiredDistanceRangeXZ[0];
            recalcMin = true;
        }

        if (this.cachedMaxRangeProvider != null) {
            double value = this.cachedMaxRangeProvider.getDoubleParameter();
            this.desiredDistanceRangeXZ[1] = (value != -Double.MAX_VALUE) ? value : this.initialDesiredDistanceRangeXZ[1];
            double max = this.desiredDistanceRangeXZ[1] + this.moveThreshold;
            this.thresholdDistanceRangeSquared[1] = max * max;
        }

        if (this.desiredDistanceRangeXZ[0] > this.desiredDistanceRangeXZ[1]) {
            this.desiredDistanceRangeXZ[0] = this.desiredDistanceRangeXZ[1];
            recalcMin = true;
        }

        if (recalcMin) {
            double min = Math.max(0.0, this.desiredDistanceRangeXZ[0] - this.moveThreshold);
            this.thresholdDistanceRangeSquared[0] = min * min;
        }

        if (infoProvider == null || !infoProvider.hasPosition()) return false;

        TransformComponent selfTc = componentAccessor.getComponent(ref, TRANSFORM_COMPONENT_TYPE);
        if (selfTc == null) return false;

        IPositionProvider positionProvider = infoProvider.getPositionProvider();
        if (positionProvider == null) return false;
        if (!positionProvider.providePosition(this.targetPosition)) return false;

        updateYOffset(this.dtCache);

        this.targetLookPosition.assign(this.targetPosition);
        applyEyeHeightOrFallback(role, componentAccessor);

        if (this.replanCooldown > 0.0) {
            this.replanCooldown -= this.dtCache;
            if (this.replanCooldown < 0.0) this.replanCooldown = 0.0;
        }

        Vector3d selfPos = selfTc.getPosition();

        updateBehindAnchor(selfPos);

        double dxT = selfPos.x - this.targetPosition.x;
        double dzT = selfPos.z - this.targetPosition.z;
        double distXZSq = dxT * dxT + dzT * dzT;

        boolean tooFar = distXZSq > this.thresholdDistanceRangeSquared[1];
        boolean tooClose = distXZSq < this.thresholdDistanceRangeSquared[0];

        this.approaching = tooFar;
        this.movingAway = tooClose;
        role.setBackingAway(tooClose);

        double t = tooFar ? (1.0 - this.targetDistanceFactor) : (tooClose ? this.targetDistanceFactor : 0.5);
        double ringDist = MathUtil.lerp(this.desiredDistanceRangeXZ[0], this.desiredDistanceRangeXZ[1], clamp(t, 0.0, 1.0));

        ensureOrbitAngleInit();

        computeRingPoint(this.desiredRingPoint, this.targetPosition, ringDist, this.orbitAngleRad);

        double progSq = probeProgressSq(ref, motionController, selfPos, this.desiredRingPoint, componentAccessor, this.probeTmp);
        if (progSq <= MIN_PROGRESS_SQ) {
            maybeReplanAngle(ref, motionController, selfPos, componentAccessor, ringDist);
            computeRingPoint(this.desiredRingPoint, this.targetPosition, ringDist, this.orbitAngleRad);
        }

        this.goalPosition.assign(this.desiredRingPoint);

        this.targetDeltaSquared = motionController.waypointDistanceSquared(this.goalPosition, this.lastPathedGoal);

        this.lastTargetPos.assign(this.targetPosition);
        this.lastTargetValid = true;

        return true;
    }

    @Nullable
    private static Ref<EntityStore> resolveTargetRef(@Nonnull Role role) {
        MarkedEntitySupport mes = role.getMarkedEntitySupport();
        Ref<EntityStore> r = null;
        try {
            r = mes.getMarkedEntityRef(MarkedEntitySupport.DEFAULT_TARGET_SLOT);
        } catch (Throwable ignored) {}
        return (r != null && r.isValid()) ? r : null;
    }

    private void applyEyeHeightOrFallback(@Nonnull Role role,
                                          @Nonnull ComponentAccessor<EntityStore> componentAccessor) {
        Ref<EntityStore> targetRef = resolveTargetRef(role);
        if (targetRef == null) {
            this.targetLookPosition.y += FALLBACK_EYE_HEIGHT;
            return;
        }

        ModelComponent mc = componentAccessor.getComponent(targetRef, MODEL_COMPONENT_TYPE);
        if (mc == null) {
            this.targetLookPosition.y += FALLBACK_EYE_HEIGHT;
            return;
        }

        Model m = mc.getModel();
        if (m == null) {
            this.targetLookPosition.y += FALLBACK_EYE_HEIGHT;
            return;
        }

        float eye = m.getEyeHeight(targetRef, componentAccessor);
        this.targetLookPosition.y += (eye > 0.0f) ? eye : FALLBACK_EYE_HEIGHT;
    }

    @Override
    protected boolean shouldDeferPathComputation(@Nonnull MotionController motionController,
                                                 Vector3d position,
                                                 @Nonnull ComponentAccessor<EntityStore> componentAccessor) {
        if (this.throttleCount > this.throttleIgnoreCount) {
            double d2 = this.goalPosition.distanceSquaredTo(this.lastPathedGoal);
            if (d2 < EPS || (this.waitForGoalMovement && d2 < GOAL_WAIT_SQ)) {
                return true;
            }
        }
        this.waitForGoalMovement = false;
        this.lastPathedGoal.assign(this.goalPosition);
        return false;
    }

    @Override
    protected boolean mustAbortThrottling(MotionController motionController, Ref<EntityStore> ref) {
        return this.targetDeltaSquared > GOAL_RECOMPUTE_SQ;
    }

    @Override
    protected boolean mustRecomputePath(@Nonnull MotionController activeMotionController) {
        if (super.mustRecomputePath(activeMotionController)) {
            return true;
        }
        if (this.targetDeltaSquared > GOAL_RECOMPUTE_SQ) {
            this.resetThrottleCount();
            return true;
        }
        return false;
    }

    @Override
    protected AStarBase.Progress startComputePath(@Nonnull Ref<EntityStore> ref,
                                                  Role role,
                                                  @Nonnull MotionController activeMotionController,
                                                  @Nonnull Vector3d position,
                                                  @Nonnull ComponentAccessor<EntityStore> componentAccessor) {
        return this.aStar.initComputePath(
                ref,
                position,
                this.lastPathedGoal,
                this,
                activeMotionController,
                this.probeMoveData,
                this.sharedNodePoolProvider,
                componentAccessor
        );
    }

    @Override
    protected boolean canSwitchToSteering(@Nonnull Ref<EntityStore> ref,
                                          @Nonnull MotionController motionController,
                                          @Nonnull ComponentAccessor<EntityStore> componentAccessor) {
        TransformComponent tc = componentAccessor.getComponent(ref, TransformComponent.getComponentType());
        if (tc == null) return false;
        Vector3d pos = tc.getPosition();

        double gate = (ARRIVE_EPS_3D * ARRIVE_EPS_3D * 25.0);
        double w2 = motionController.waypointDistanceSquared(pos, this.goalPosition);
        if (w2 > gate) return false;

        return canReachGoal(ref, motionController, pos, this.goalPosition, componentAccessor);
    }

    @Override
    protected boolean shouldSkipSteering(@Nonnull Ref<EntityStore> ref,
                                         @Nonnull MotionController activeMotionController,
                                         @Nonnull Vector3d position,
                                         @Nonnull ComponentAccessor<EntityStore> componentAccessor) {
        this.probeMoveData.setPosition(position).setTargetPosition(this.goalPosition);
        activeMotionController.probeMove(ref, this.probeMoveData, componentAccessor);
        return !this.isGoalReached(ref, activeMotionController, this.probeMoveData.probePosition, this.goalPosition, componentAccessor);
    }

    @Override
    protected boolean computeSteering(@Nonnull Ref<EntityStore> ref,
                                      @Nonnull Role role,
                                      @Nonnull Vector3d position,
                                      @Nonnull Steering desiredSteering,
                                      @Nonnull ComponentAccessor<EntityStore> componentAccessor) {
        this.seek.setPositions(position, this.goalPosition);
        MotionController motionController = role.getActiveMotionController();
        this.seek.setComponentSelector(motionController.getComponentSelector());
        double desiredAltitudeWeight = this.desiredAltitudeWeight >= 0.0 ? this.desiredAltitudeWeight : motionController.getDesiredAltitudeWeight();

        if (!this.seek.compute(desiredSteering)) {
            return false;
        }

        double speed = this.approaching || !this.movingAway ? this.relativeForwardsSpeed : this.relativeBackwardsSpeed;
        desiredSteering.scaleTranslation(speed);

        double distSq = position.distanceSquaredTo(this.goalPosition);
        double dist = Math.sqrt(Math.max(0.0, distSq));
        double slowDist = Math.max(this.stopEps3d, this.moveTowardsSlowdownThreshold);

        double desiredScale = 1.0;
        if (!this.movingAway && slowDist > EPS) {
            desiredScale = clamp(dist / slowDist, 0.0, 1.0);
            if (!this.hardStop) {
                desiredScale = Math.max(desiredScale, clamp(SOFT_MIN_SPEED_SCALE, 0.0, 1.0));
            }
        }

        double alpha = 1.0 - Math.exp(-SPEED_SCALE_RESPONSE * Math.max(0.0, this.dtCache));
        this.speedScale = this.speedScale + (desiredScale - this.speedScale) * clamp(alpha, 0.0, 1.0);

        desiredSteering.scaleTranslation(this.speedScale);

        boolean approachDesiredHeight = !motionController.is2D() && desiredAltitudeWeight > 0.0;
        boolean withinRange = approachDesiredHeight && motionController.getDesiredVerticalRange(ref, componentAccessor).isWithinRange();
        if (approachDesiredHeight && !withinRange) {
            MotionController.VerticalRange desiredAltitudeRange = motionController.getDesiredVerticalRange(ref, componentAccessor);
            if (desiredAltitudeRange.current > desiredAltitudeRange.max) {
                desiredSteering.setY(-this.computeDesiredYTranslation(desiredSteering, motionController.getMaxSinkAngle(), desiredAltitudeWeight));
            } else if (desiredAltitudeRange.current < desiredAltitudeRange.min) {
                desiredSteering.setY(this.computeDesiredYTranslation(desiredSteering, motionController.getMaxClimbAngle(), desiredAltitudeWeight));
            }
        }

        motionController.requireDepthProbing();
        return true;
    }

    private void lookAtTarget(@Nonnull Ref<EntityStore> ref,
                              @Nonnull Role role,
                              @Nonnull Steering steering,
                              @Nonnull ComponentAccessor<EntityStore> componentAccessor) {
        TransformComponent tc = componentAccessor.getComponent(ref, TransformComponent.getComponentType());
        if (tc == null) return;

        Vector3d pos = tc.getPosition();
        Vector3f rot = tc.getRotation();

        double dx = this.targetLookPosition.x - pos.x;
        double dy = this.targetLookPosition.y - pos.y;
        double dz = this.targetLookPosition.z - pos.z;

        steering.setYaw(NPCPhysicsMath.headingFromDirection(dx, dz, rot.getYaw()));
        steering.setPitch(NPCPhysicsMath.pitchFromDirection(dx, dy, dz, rot.getPitch()));
    }

    @Override
    protected boolean isGoalReached(@Nonnull Ref<EntityStore> ref,
                                    @Nonnull MotionController motionController,
                                    @Nonnull Vector3d position,
                                    @Nonnull ComponentAccessor<EntityStore> componentAccessor) {
        return this.isGoalReached(ref, motionController, position, this.goalPosition, componentAccessor);
    }

    protected boolean isGoalReached(@Nonnull Ref<EntityStore> ref,
                                    @Nonnull MotionController motionController,
                                    @Nonnull Vector3d position,
                                    @Nonnull Vector3d targetPosition,
                                    @Nonnull ComponentAccessor<EntityStore> componentAccessor) {
        double eps = this.stopEps3d;
        return motionController.waypointDistanceSquared(position, targetPosition) <= (eps * eps);
    }

    @Override
    public boolean isGoalReached(@Nonnull Ref<EntityStore> ref,
                                 AStarBase aStarBase,
                                 @Nonnull AStarNode aStarNode,
                                 @Nonnull MotionController motionController,
                                 @Nonnull ComponentAccessor<EntityStore> componentAccessor) {
        AStarWithTarget withTarget = (AStarWithTarget) aStarBase;
        return this.isGoalReached(ref, motionController, aStarNode.getPosition(), withTarget.getTargetPosition(), componentAccessor);
    }

    @Override
    public float estimateToGoal(@Nonnull AStarBase aStarBase,
                                @Nonnull Vector3d fromPosition,
                                MotionController motionController) {
        return (float) ((AStarWithTarget) aStarBase).getTargetPosition().distanceTo(fromPosition);
    }

    @Override
    public void findBestPath(@Nonnull AStarBase aStarBase, MotionController controller) {
        aStarBase.buildBestPath(AStarNode::getEstimateToGoal, (oldV, v) -> v < oldV, Float.MAX_VALUE);
    }

    @Override
    protected void onBlockedPath() {
        super.onBlockedPath();
        if (this.replanCooldown <= 0.0) {
            double step = Math.toRadians(ANGLE_STEP_DEG);
            double sign = ThreadLocalRandom.current().nextBoolean() ? 1.0 : -1.0;
            this.orbitAngleRad = wrapRadians(this.orbitAngleRad + sign * step);
            this.replanCooldown = REPLAN_COOLDOWN_SECONDS;
        }
    }

    @Override
    protected void onNoPathFound(MotionController motionController) {
        super.onNoPathFound(motionController);
        this.waitForGoalMovement = true;
        if (this.replanCooldown <= 0.0) {
            this.orbitAngleRad = ThreadLocalRandom.current().nextDouble(0.0, Math.PI * 2.0);
            this.replanCooldown = REPLAN_COOLDOWN_SECONDS;
        }
    }

    @Override
    public void deactivate(@Nonnull Ref<EntityStore> ref,
                           @Nonnull Role role,
                           @Nonnull ComponentAccessor<EntityStore> componentAccessor) {
        super.deactivate(ref, role, componentAccessor);
        role.setBackingAway(false);
        this.replanCooldown = 0.0;
        this.orbitAngleRad = Double.NaN;
        this.goalPosition.assign(0, 0, 0);
        this.lastPathedGoal.assign(Double.MIN_VALUE, Double.MIN_VALUE, Double.MIN_VALUE);
        this.waitForGoalMovement = false;
        this.targetLookPosition.assign(0, 0, 0);
        this.speedScale = 1.0;
        this.yOffsetTimer = nextYOffsetInterval(this.yOffsetChangeIntervalRange);
        this.currentYOffset = pickYOffset(this.yOffsets);

        this.lastTargetValid = false;
        this.lastTargetPos.assign(Double.NaN, Double.NaN, Double.NaN);
        this.desiredAnchorAngleRad = Double.NaN;
    }
}