package me.s3b4s5.npc.movement.builders;

import com.google.gson.JsonElement;
import com.hypixel.hytale.server.npc.asset.builder.BuilderDescriptorState;
import com.hypixel.hytale.server.npc.asset.builder.BuilderSupport;
import com.hypixel.hytale.server.npc.asset.builder.Feature;
import com.hypixel.hytale.server.npc.asset.builder.holder.BooleanHolder;
import com.hypixel.hytale.server.npc.asset.builder.holder.DoubleHolder;
import com.hypixel.hytale.server.npc.asset.builder.holder.NumberArrayHolder;
import com.hypixel.hytale.server.npc.asset.builder.validators.DoubleRangeValidator;
import com.hypixel.hytale.server.npc.asset.builder.validators.DoubleSequenceValidator;
import com.hypixel.hytale.server.npc.asset.builder.validators.DoubleSingleValidator;
import com.hypixel.hytale.server.npc.corecomponents.movement.builders.BuilderBodyMotionFindBase;
import com.hypixel.hytale.server.npc.movement.controllers.MotionControllerFly;
import com.hypixel.hytale.server.npc.util.expression.ExecutionContext;
import com.hypixel.hytale.server.npc.util.expression.Scope;
import com.hypixel.hytale.server.npc.validators.NPCLoadTimeValidationHelper;
import me.s3b4s5.npc.movement.body.BodyMotionMaintainDistanceFlyRing;

import javax.annotation.Nonnull;
import java.util.List;

/**
 * JSON builder for {@link BodyMotionMaintainDistanceFlyRing}.
 *
 * <p>This builder defines the configuration schema for the {@code "MaintainDistanceFlyRing"} BodyMotion type.
 * The BodyMotion is intended to be referenced from role JSON as:</p>
 *
 * <pre>{@code
 * "BodyMotion": {
 *   "Type": "MaintainDistanceFlyRing",
 *   ...
 * }
 * }</pre>
 *
 * <h2>Required features / compatibility</h2>
 * <ul>
 *   <li><b>Required feature:</b> {@code Feature.AnyPosition}. An upstream sensor must provide a position
 *       each tick (commonly {@code Target} or {@code Leash}).</li>
 *   <li><b>Required motion controller:</b> {@link MotionControllerFly}. This motion is designed for flying actors;
 *       validation enforces Fly compatibility.</li>
 * </ul>
 *
 * <h2>Distance / ring parameters</h2>
 * <ul>
 *   <li><b>{@code DesiredDistanceRangeXZ} (required)</b>
 *     <ul>
 *       <li>Type: array [min, max] (weakly monotonic).</li>
 *       <li>Meaning: desired radius band around the target on the XZ plane.</li>
 *       <li>Example: {@code [3.0, 4.2]}.</li>
 *     </ul>
 *   </li>
 *
 *   <li><b>{@code TargetDistanceFactor}</b> (default: {@code 0.5}, range: 0..1)
 *     <ul>
 *       <li>How the motion picks a radius inside {@code DesiredDistanceRangeXZ} when adjusting.</li>
 *       <li>0 = bias towards the nearest edge of the band, 1 = furthest edge, 0.5 = midpoint.</li>
 *     </ul>
 *   </li>
 *
 *   <li><b>{@code MoveThreshold}</b> (default: {@code 0.7}, &gt; 0)
 *     <ul>
 *       <li>Hysteresis around the band to prevent constant in/out oscillation.</li>
 *       <li>Larger values = less twitchy corrections, but slower to react to small deviations.</li>
 *     </ul>
 *   </li>
 * </ul>
 *
 * <h2>Vertical offsets</h2>
 * <ul>
 *   <li><b>{@code YOffsets} (required)</b>
 *     <ul>
 *       <li>Type: array of non-negative doubles.</li>
 *       <li>Meaning: candidate vertical offsets above the target.</li>
 *       <li>Example: {@code [2.6, 3.0]}.</li>
 *       <li>If empty, falls back to {@link #DEFAULT_YOFFSETS}.</li>
 *     </ul>
 *   </li>
 *
 *   <li><b>{@code YOffsetChangeIntervalRange}</b> (default: {@code [2.0, 4.0]})
 *     <ul>
 *       <li>How often (seconds) the motion picks a new value from {@code YOffsets}.</li>
 *       <li>Example: {@code [2.0, 4.0]} means “every 2–4 seconds”.</li>
 *     </ul>
 *   </li>
 * </ul>
 *
 * <h2>Speed shaping</h2>
 * <ul>
 *   <li><b>{@code RelativeForwardsSpeed}</b> (default: {@code 1.0}, range: (0..2])
 *     <ul>
 *       <li>Max relative translation scaling when the NPC needs to move <i>towards</i> the desired ring.</li>
 *       <li>Values &gt; 1 allow faster catch-up; values &lt; 1 make movement more gentle.</li>
 *     </ul>
 *   </li>
 *
 *   <li><b>{@code RelativeBackwardsSpeed}</b> (default: {@code 0.6}, range: (0..2])
 *     <ul>
 *       <li>Max relative translation scaling when the NPC needs to move <i>away</i> (too close).</li>
 *       <li>Lower values help avoid snapping/overshooting when backing away.</li>
 *     </ul>
 *   </li>
 *
 *   <li><b>{@code MoveTowardsSlowdownThreshold}</b> (default: {@code 12.0}, &ge; 0)
 *     <ul>
 *       <li>Distance from the goal where the motion starts reducing speed while approaching.</li>
 *       <li>Higher values = earlier and smoother braking (less “hard stop” after catch-up).</li>
 *     </ul>
 *   </li>
 * </ul>
 *
 * <h2>Arrival behavior</h2>
 * <ul>
 *   <li><b>{@code HardStop}</b> (default: {@code false})
 *     <ul>
 *       <li>If true: allow speed to go to (near) zero when reaching the ring point (classic arrive).</li>
 *       <li>If false: apply a minimum speed floor (soft-follow) to reduce stop-and-go behavior.</li>
 *     </ul>
 *   </li>
 * </ul>
 *
 * <h2>Behind-anchoring (optional)</h2>
 * <p>These options bias the ring angle so the NPC prefers staying behind the target's movement direction.
 * Useful for "pet/summon follow" behavior to avoid lateral ring picks.</p>
 *
 * <ul>
 *   <li><b>{@code LockBehindTarget}</b> (default: {@code true})
 *     <ul>
 *       <li>When enabled, the ring angle is blended towards an angle behind the target velocity.</li>
 *       <li>If the target is not moving fast enough, the motion uses a radial fallback (stable side selection).</li>
 *     </ul>
 *   </li>
 *
 *   <li><b>{@code BehindBlend}</b> (default: {@code 0.25}, range: 0..1)
 *     <ul>
 *       <li>How quickly the orbit angle rotates towards the behind-anchor each tick.</li>
 *       <li>0 = no smoothing (snap), 1 = snap immediately (in practice), ~0.1–0.35 = smooth follow.</li>
 *     </ul>
 *   </li>
 *
 *   <li><b>{@code MinTargetSpeed}</b> (default: {@code 0.15}, &ge; 0)
 *     <ul>
 *       <li>Minimum target speed (blocks/sec) required to treat velocity direction as meaningful.</li>
 *       <li>Below this value the behind-anchor uses a fallback to avoid random lateral flips.</li>
 *     </ul>
 *   </li>
 * </ul>
 *
 * <h2>Dynamic parameters</h2>
 * <p>The base class provides parameter slots. This builder exposes:</p>
 * <ul>
 *   <li>{@link #MIN_RANGE_PARAMETER} and {@link #MAX_RANGE_PARAMETER} which can override the configured
 *       {@code DesiredDistanceRangeXZ} at runtime (if upstream parameters are bound).</li>
 * </ul>
 */
public class BuilderBodyMotionMaintainDistanceFlyRing extends BuilderBodyMotionFindBase {

    public static final String TYPE = "MaintainDistanceFlyRing";
    public static final String MIN_RANGE_PARAMETER = "MinRange";
    public static final String MAX_RANGE_PARAMETER = "MaxRange";

    public static final double[] DEFAULT_YOFFSET_CHANGE_INTERVAL_RANGE = new double[]{2.0, 4.0};
    public static final double[] DEFAULT_YOFFSETS = new double[]{3.0};

    protected final NumberArrayHolder desiredDistanceRangeXZ = new NumberArrayHolder();
    protected final NumberArrayHolder yOffsets = new NumberArrayHolder();
    protected final NumberArrayHolder yOffsetChangeIntervalRange = new NumberArrayHolder();

    protected final DoubleHolder targetDistanceFactor = new DoubleHolder();
    protected final DoubleHolder moveThreshold = new DoubleHolder();
    protected final DoubleHolder relativeForwardsSpeed = new DoubleHolder();
    protected final DoubleHolder relativeBackwardsSpeed = new DoubleHolder();
    protected final DoubleHolder moveTowardsSlowdownThreshold = new DoubleHolder();

    protected final BooleanHolder hardStop = new BooleanHolder();
    protected final BooleanHolder lockBehindTarget = new BooleanHolder();
    protected final DoubleHolder behindBlend = new DoubleHolder();
    protected final DoubleHolder minTargetSpeed = new DoubleHolder();

    @Nonnull
    @Override
    public BodyMotionMaintainDistanceFlyRing build(@Nonnull BuilderSupport builderSupport) {
        return new BodyMotionMaintainDistanceFlyRing(this, builderSupport);
    }

    @Nonnull
    @Override
    public String getShortDescription() {
        return "Maintain XZ distance in a ring around a target while keeping a vertical offset (fly).";
    }

    @Nonnull
    @Override
    public String getLongDescription() {
        return getShortDescription();
    }

    @Nonnull
    @Override
    public BuilderDescriptorState getBuilderDescriptorState() {
        return BuilderDescriptorState.Stable;
    }

    @Nonnull
    @Override
    public BuilderBodyMotionMaintainDistanceFlyRing readConfig(@Nonnull JsonElement data) {
        super.readConfig(data);

        this.requireDoubleRange(
                data,
                "DesiredDistanceRangeXZ",
                this.desiredDistanceRangeXZ,
                DoubleSequenceValidator.betweenWeaklyMonotonic(-Double.MAX_VALUE, Double.MAX_VALUE),
                BuilderDescriptorState.Stable,
                "Desired XZ distance range to remain at (ring radius).",
                null
        );

        this.requireDoubleRange(
                data,
                "YOffsets",
                this.yOffsets,
                DoubleSequenceValidator.betweenWeaklyMonotonic(0.0, Double.MAX_VALUE),
                BuilderDescriptorState.Stable,
                "List of vertical offsets relative to the target position to randomly pick from.",
                null
        );

        this.getDoubleRange(
                data,
                "YOffsetChangeIntervalRange",
                this.yOffsetChangeIntervalRange,
                DEFAULT_YOFFSET_CHANGE_INTERVAL_RANGE,
                DoubleSequenceValidator.betweenWeaklyMonotonic(0.0, Double.MAX_VALUE),
                BuilderDescriptorState.Stable,
                "How often (seconds) to pick a new Y offset from YOffsets.",
                null
        );

        this.getDouble(
                data,
                "TargetDistanceFactor",
                this.targetDistanceFactor,
                0.5,
                DoubleRangeValidator.between(0.0, 1.0),
                BuilderDescriptorState.Stable,
                "Factor used to pick a target distance within the desired XZ range.",
                null
        );

        this.getDouble(
                data,
                "MoveThreshold",
                this.moveThreshold,
                0.7,
                DoubleSingleValidator.greater0(),
                BuilderDescriptorState.Stable,
                "Extra threshold around desired XZ range before movement triggers (hysteresis).",
                null
        );

        this.getDouble(
                data,
                "RelativeForwardsSpeed",
                this.relativeForwardsSpeed,
                1.0,
                DoubleRangeValidator.fromExclToIncl(0.0, 2.0),
                BuilderDescriptorState.Stable,
                "Maximum relative speed when moving towards the target (XZ).",
                null
        );

        this.getDouble(
                data,
                "RelativeBackwardsSpeed",
                this.relativeBackwardsSpeed,
                0.6,
                DoubleRangeValidator.fromExclToIncl(0.0, 2.0),
                BuilderDescriptorState.Stable,
                "Maximum relative speed when moving away from the target (XZ).",
                null
        );

        this.getDouble(
                data,
                "MoveTowardsSlowdownThreshold",
                this.moveTowardsSlowdownThreshold,
                12.0,
                DoubleSingleValidator.greaterEqual0(),
                BuilderDescriptorState.Stable,
                "Distance from stopping point where NPC starts slowing down while moving towards target.",
                null
        );

        this.getBoolean(
                data,
                "HardStop",
                this.hardStop,
                false,
                BuilderDescriptorState.Stable,
                "Hard-stop at ring point.",
                null
        );

        this.getBoolean(
                data,
                "LockBehindTarget",
                this.lockBehindTarget,
                true,
                BuilderDescriptorState.Stable,
                "Anchor the ring angle behind the target movement direction.",
                null
        );

        this.getDouble(
                data,
                "BehindBlend",
                this.behindBlend,
                0.25,
                DoubleRangeValidator.between(0.0, 1.0),
                BuilderDescriptorState.Stable,
                "Blend factor to rotate ring angle towards behind anchor.",
                null
        );

        this.getDouble(
                data,
                "MinTargetSpeed",
                this.minTargetSpeed,
                0.15,
                DoubleSingleValidator.greaterEqual0(),
                BuilderDescriptorState.Stable,
                "Minimum target speed (blocks/sec) to use velocity-based behind anchor.",
                null
        );

        this.requireFeature(Feature.AnyPosition);
        return this;
    }

    @Override
    public boolean validate(String configName,
                            @Nonnull NPCLoadTimeValidationHelper validationHelper,
                            ExecutionContext context,
                            Scope globalScope,
                            @Nonnull List<String> errors) {
        boolean result = super.validate(configName, validationHelper, context, globalScope, errors);
        validationHelper.requireMotionControllerType(MotionControllerFly.class);
        return result;
    }

    public double[] getDesiredDistanceRangeXZ(@Nonnull BuilderSupport support) {
        return this.desiredDistanceRangeXZ.get(support.getExecutionContext());
    }

    public double[] getYOffsets(@Nonnull BuilderSupport support) {
        double[] v = this.yOffsets.get(support.getExecutionContext());
        return (v == null || v.length == 0) ? DEFAULT_YOFFSETS : v;
    }

    public double[] getYOffsetChangeIntervalRange(@Nonnull BuilderSupport support) {
        double[] v = this.yOffsetChangeIntervalRange.get(support.getExecutionContext());
        return (v == null || v.length < 2) ? DEFAULT_YOFFSET_CHANGE_INTERVAL_RANGE : v;
    }

    public double getTargetDistanceFactor(@Nonnull BuilderSupport support) {
        return this.targetDistanceFactor.get(support.getExecutionContext());
    }

    public double getMoveThreshold(@Nonnull BuilderSupport support) {
        return this.moveThreshold.get(support.getExecutionContext());
    }

    public double getRelativeForwardsSpeed(@Nonnull BuilderSupport support) {
        return this.relativeForwardsSpeed.get(support.getExecutionContext());
    }

    public double getRelativeBackwardsSpeed(@Nonnull BuilderSupport support) {
        return this.relativeBackwardsSpeed.get(support.getExecutionContext());
    }

    public double getMoveTowardsSlowdownThreshold(@Nonnull BuilderSupport support) {
        return this.moveTowardsSlowdownThreshold.get(support.getExecutionContext());
    }

    public boolean isHardStop(@Nonnull BuilderSupport support) {
        return this.hardStop.get(support.getExecutionContext());
    }

    public boolean isLockBehindTarget(@Nonnull BuilderSupport support) {
        return this.lockBehindTarget.get(support.getExecutionContext());
    }

    public double getBehindBlend(@Nonnull BuilderSupport support) {
        return this.behindBlend.get(support.getExecutionContext());
    }

    public double getMinTargetSpeed(@Nonnull BuilderSupport support) {
        return this.minTargetSpeed.get(support.getExecutionContext());
    }
}