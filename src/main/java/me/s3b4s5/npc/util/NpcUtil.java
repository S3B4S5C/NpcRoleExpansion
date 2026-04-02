package me.s3b4s5.npc.util;

import com.hypixel.hytale.component.*;
import com.hypixel.hytale.logger.HytaleLogger;
import com.hypixel.hytale.math.vector.Vector3d;
import com.hypixel.hytale.math.vector.Vector3f;
import com.hypixel.hytale.server.core.asset.type.entityeffect.config.EntityEffect;
import com.hypixel.hytale.server.core.asset.type.entityeffect.config.RemovalBehavior;
import com.hypixel.hytale.server.core.asset.type.model.config.Model;
import com.hypixel.hytale.server.core.entity.effect.EffectControllerComponent;
import com.hypixel.hytale.server.core.modules.entity.component.ModelComponent;
import com.hypixel.hytale.server.core.modules.entity.component.TransformComponent;
import com.hypixel.hytale.server.core.modules.entitystats.EntityStatMap;
import com.hypixel.hytale.server.core.modules.entitystats.EntityStatValue;
import com.hypixel.hytale.server.core.modules.entitystats.asset.EntityStatType;
import com.hypixel.hytale.server.core.modules.interaction.interaction.config.RootInteraction;
import com.hypixel.hytale.server.core.universe.PlayerRef;
import com.hypixel.hytale.server.core.universe.Universe;
import com.hypixel.hytale.server.core.universe.world.World;
import com.hypixel.hytale.server.core.universe.world.storage.EntityStore;
import com.hypixel.hytale.server.npc.NPCPlugin;
import com.hypixel.hytale.server.npc.asset.builder.BuilderInfo;
import com.hypixel.hytale.server.npc.entities.NPCEntity;

import javax.annotation.Nullable;
import java.nio.file.Path;
import java.util.Locale;
import java.util.Set;
import java.util.UUID;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;
import java.util.function.Consumer;

/**
 * Utility helpers for working with NPC entities across the project.
 *
 * <p>This class is intentionally small and stateless: it provides common operations needed by multiple
 * systems without introducing additional dependencies or persistent state.</p>
 *
 * <h2>Responsibilities</h2>
 * <ul>
 *   <li><b>Spawning from roles</b>: Build and spawn an NPC using an NPC role id via a provided factory.</li>
 *   <li><b>Leash steering</b>: Update an {@link NPCEntity}'s leash point and leash-facing orientation.</li>
 *   <li><b>Health queries</b>: Read the {@code Health} stat from {@link EntityStatMap} to determine if an entity is alive.</li>
 *   <li><b>Effect helpers</b>: Query and apply {@link EntityEffect} instances through {@link EffectControllerComponent}.</li>
 *   <li><b>Attack profile helpers</b>: Determine whether a role is melee/ranged by reading its Role JSON and
 *       resolving tags from the referenced {@link RootInteraction}. (Supports Variant + Reference inheritance.)</li>
 *   <li><b>Small math helpers</b>: Convenience transforms such as XZ yaw-facing.</li>
 * </ul>
 *
 * <h2>CommandBuffer vs Store</h2>
 * <ul>
 *   <li>Reads generally attempt {@link CommandBuffer} first (most up-to-date changes in the current tick),
 *       falling back to {@link Store} if necessary.</li>
 *   <li>Writes are performed through {@link CommandBuffer} (or scheduled world-thread work when spawning).</li>
 * </ul>
 */
public final class NpcUtil {

    private NpcUtil() {}

    /**
     * Factory that builds an {@link Holder} containing an {@link NPCEntity} configured from a role id.
     *
     * <p>Implementations are expected to:
     * <ul>
     *   <li>Resolve/validate the role id</li>
     *   <li>Create and attach the required components (NPCEntity + Transform + etc.)</li>
     *   <li>Return {@code null} if the role cannot be built</li>
     * </ul>
     *
     * <p>This is designed to accept existing helpers as method references (e.g. {@code SomeUtil::buildNpcRoleHolder}).</p>
     */
    @FunctionalInterface
    public interface RoleHolderFactory {
        @Nullable
        Holder<EntityStore> build(
                Store<EntityStore> store,
                String npcRoleId,
                Vector3d spawnPos,
                @Nullable Vector3f rotation
        );
    }

    /**
     * Builds an NPC entity holder from an NPC role id and schedules it to be spawned on the {@link World} thread.
     *
     * <p>This method is generic and does not know about gameplay logic. Callers provide:
     * <ul>
     *   <li>a {@link RoleHolderFactory} to build the base NPC holder from a role id</li>
     *   <li>an optional {@code configureHolder} callback to attach additional components before spawn</li>
     *   <li>optional logger + tag for consistent diagnostics</li>
     * </ul>
     *
     * @param store           Entity store where the NPC will be spawned.
     * @param world           World used to schedule {@code store.addEntity(...)} on the correct thread.
     * @param npcRoleId       NPC role id (asset id / role key).
     * @param spawnPos        World-space spawn position.
     * @param rotation        Optional initial rotation (implementation-defined; often yaw-facing).
     * @param reason          Spawn reason passed to {@link Store#addEntity}.
     * @param holderFactory   Factory that builds the base NPC holder.
     * @param configureHolder Optional callback to attach additional components to the holder.
     * @return {@code true} if the holder was built and the spawn was scheduled; {@code false} otherwise.
     */
    public static boolean spawnNpcFromRole(
            Store<EntityStore> store,
            World world,
            String npcRoleId,
            Vector3d spawnPos,
            @Nullable Vector3f rotation,
            AddReason reason,
            RoleHolderFactory holderFactory,
            @Nullable Consumer<Holder<EntityStore>> configureHolder
    ) {
        if (store == null || world == null) return false;
        if (npcRoleId == null || npcRoleId.isBlank()) return false;
        if (spawnPos == null) return false;
        if (holderFactory == null) return false;

        final Holder<EntityStore> h;
        try {
            h = holderFactory.build(store, npcRoleId, spawnPos, rotation);
        } catch (Throwable t) {
            return false;
        }

        if (h == null) return false;

        if (configureHolder != null) {
            try {
                configureHolder.accept(h);
            } catch (Throwable t) {
                return false;
            }
        }

        world.execute(() -> {
            try {
                store.addEntity(h, reason);
            } catch (Throwable _) {
            }
        });

        return true;
    }

    /**
     * Sets an NPC's leash point and aligns its leash-facing yaw/pitch to match a "look" entity.
     *
     * <p>The leash heading/pitch are copied from the {@link TransformComponent} rotation of {@code lookRef}
     * (if available).</p>
     *
     * @param selfRef    The NPC entity reference that will be leashed.
     * @param lookRef    Entity reference used to copy yaw/pitch for leash-facing.
     * @param store      Entity store for component reads.
     * @param cb         Command buffer for up-to-date reads and writes.
     * @param leashPoint World-space point the NPC should leash towards.
     */
    public static void setLeashToPoint(
            Ref<EntityStore> selfRef,
            Ref<EntityStore> lookRef,
            Store<EntityStore> store,
            CommandBuffer<EntityStore> cb,
            Vector3d leashPoint
    ) {
        ComponentType<EntityStore, NPCEntity> npcType = NPCEntity.getComponentType();
        if (npcType == null) return;

        NPCEntity npc = cb.getComponent(selfRef, npcType);
        if (npc == null) npc = store.getComponent(selfRef, npcType);
        if (npc == null) return;

        npc.getLeashPoint().assign(leashPoint);

        TransformComponent lt = store.getComponent(lookRef, TransformComponent.getComponentType());
        if (lt != null) {
            Vector3f rot = lt.getRotation();
            npc.setLeashHeading(rot.getYaw());
            npc.setLeashPitch(rot.getPitch());
        }
    }

    /**
     * Returns whether an entity is considered alive by checking the {@code Health} stat.
     *
     * <p>If the entity does not have an {@link EntityStatMap}, this returns {@code true} to avoid
     * treating unknown entities as dead.</p>
     *
     * @param ref   Entity reference to check.
     * @param store Entity store for component access.
     * @return {@code true} if Health exists and is greater than a small epsilon, or if no stat map exists.
     */
    public static boolean isAlive(Ref<EntityStore> ref, Store<EntityStore> store) {
        EntityStatMap statMap = store.getComponent(ref, EntityStatMap.getComponentType());
        if (statMap == null) return true;

        final int idx = EntityStatType.getAssetMap().getIndex("Health");
        if (idx < 0) return false;

        EntityStatValue value = statMap.get(idx);
        if (value == null) return false;

        return value.get() > 0.001f;
    }

    /**
     * Reads the {@code Health} stat value for an entity.
     *
     * @param ref   Entity reference.
     * @param store Entity store for component access.
     * @return Current health value, or {@code -1} if the stat cannot be read.
     */
    public static float getHealth(Ref<EntityStore> ref, Store<EntityStore> store) {
        EntityStatMap statMap = store.getComponent(ref, EntityStatMap.getComponentType());
        if (statMap == null) return -1f;

        final int idx = EntityStatType.getAssetMap().getIndex("Health");
        if (idx < 0) return -1f;

        EntityStatValue value = statMap.get(idx);
        if (value == null) return -1f;

        return value.get();
    }

    /**
     * Returns true if the player identified by {@code uuid} currently has {@code effectId} active.
     *
     * @param uuid     Player UUID.
     * @param store    Entity store for reads.
     * @param cb       Command buffer for latest reads.
     * @param effectId Effect asset id.
     * @return {@code true} if the effect is active, otherwise {@code false}.
     */
    public static boolean hasEffect(
            UUID uuid,
            Store<EntityStore> store,
            CommandBuffer<EntityStore> cb,
            String effectId
    ) {
        PlayerRef player = Universe.get().getPlayer(uuid);
        if (player == null) return false;

        Ref<EntityStore> ref = player.getReference();
        if (ref == null || !ref.isValid()) return false;

        if (effectId == null || effectId.isBlank()) return false;

        int idx = EntityEffect.getAssetMap().getIndex(effectId);
        if (idx == Integer.MIN_VALUE) return false;

        ComponentType<EntityStore, EffectControllerComponent> type = EffectControllerComponent.getComponentType();
        EffectControllerComponent ecc = cb.getComponent(ref, type);
        if (ecc == null) ecc = store.getComponent(ref, type);
        if (ecc == null) return false;

        return ecc.getActiveEffects().containsKey(idx);
    }

    /**
     * Applies an effect to an entity by effect id.
     *
     * @param ref      Target entity reference.
     * @param store    Entity store for reads.
     * @param cb       Command buffer for reads/writes.
     * @param effectId Effect asset id.
     * @return {@code true} if the effect was applied successfully; {@code false} otherwise.
     */
    public static boolean applyEffect(
            Ref<EntityStore> ref,
            Store<EntityStore> store,
            CommandBuffer<EntityStore> cb,
            String effectId
    ) {
        if (ref == null || !ref.isValid()) return false;
        if (effectId == null || effectId.isBlank()) return false;

        int idx = EntityEffect.getAssetMap().getIndex(effectId);
        if (idx == Integer.MIN_VALUE) return false;

        EntityEffect effect = EntityEffect.getAssetMap().getAsset(idx);
        if (effect == null) return false;

        ComponentType<EntityStore, EffectControllerComponent> type = EffectControllerComponent.getComponentType();

        EffectControllerComponent ecc = cb.getComponent(ref, type);
        if (ecc == null) ecc = store.getComponent(ref, type);
        if (ecc == null) ecc = new EffectControllerComponent();

        boolean ok = ecc.addEffect(ref, idx, effect, cb);
        cb.putComponent(ref, type, ecc);
        return ok;
    }

    /**
     * Removes (cancels) an active effect from an entity by effect id.
     *
     * <p>If the entity has no {@link EffectControllerComponent}, this returns {@code false} (nothing to remove).</p>
     *
     * @param ref      Target entity reference.
     * @param store    Entity store for reads.
     * @param cb       Command buffer for reads/writes.
     * @param effectId Effect asset id.
     * @return {@code true} if an active effect was removed; {@code false} otherwise.
     */
    public static boolean removeEffect(
            Ref<EntityStore> ref,
            Store<EntityStore> store,
            CommandBuffer<EntityStore> cb,
            String effectId
    ) {
        if (ref == null || !ref.isValid()) return false;
        if (effectId == null || effectId.isBlank()) return false;

        int idx = EntityEffect.getAssetMap().getIndex(effectId);
        if (idx == Integer.MIN_VALUE) return false;

        ComponentType<EntityStore, EffectControllerComponent> type = EffectControllerComponent.getComponentType();
        EffectControllerComponent ecc = cb.getComponent(ref, type);
        if (ecc == null) ecc = store.getComponent(ref, type);
        if (ecc == null) return false;

        if (!ecc.getActiveEffects().containsKey(idx)) return false;

        ComponentAccessor<EntityStore> accessor;
        accessor = cb;

        ecc.removeEffect(ref, idx, RemovalBehavior.COMPLETE, accessor);
        accessor.putComponent(ref, type, ecc);
        return true;
    }

    /**
     * Computes a rotation that faces from {@code from} towards {@code to} on the XZ plane.
     *
     * <p>Pitch and roll are set to 0. Yaw is returned in degrees.</p>
     *
     * @param from Origin position.
     * @param to   Target position.
     * @return Rotation vector where yaw faces the target.
     */
    public static Vector3f computeYawFacing(Vector3d from, Vector3d to) {
        double dx = to.x - from.x;
        double dz = to.z - from.z;
        float yaw = (float) Math.toDegrees(Math.atan2(dx, dz));
        return new Vector3f(0f, yaw, 0f);
    }

    /**
     * Returns an entity's current position from its {@link TransformComponent}.
     *
     * @param ref   entity reference
     * @param store entity store
     * @return position vector if available; otherwise {@code null}
     */
    @Nullable
    public static Vector3d getPosition(Ref<EntityStore> ref, Store<EntityStore> store) {
        if (ref == null || !ref.isValid() || store == null) return null;
        TransformComponent t = store.getComponent(ref, TransformComponent.getComponentType());
        return t != null ? t.getPosition() : null;
    }

    /**
     * Computes horizontal distance (XZ plane) between two positions.
     *
     * @param a first position
     * @param b second position
     * @return distance on XZ plane, or {@link Double#NaN} if either is null
     */
    public static double horizontalDistanceXZ(Vector3d a, Vector3d b) {
        if (a == null || b == null) return Double.NaN;
        double dx = b.x - a.x;
        double dz = b.z - a.z;
        return Math.sqrt(dx * dx + dz * dz);
    }

    /**
     * Returns an entity's eye Y position ({@code position.y + eyeHeight}).
     *
     * <p>Eye height is obtained from {@link Model#getEyeHeight(Ref, ComponentAccessor)} when possible.
     * If the model or accessor can't be resolved, {@code fallbackEyeHeight} is used.</p>
     *
     * @param ref entity reference
     * @param store entity store
     * @param cb command buffer for latest component reads
     * @param fallbackEyeHeight fallback eye height when model data isn't available
     * @return eye Y coordinate, or {@link Double#NaN} if no transform exists
     */
    public static double getEyeY(
            Ref<EntityStore> ref,
            Store<EntityStore> store,
            CommandBuffer<EntityStore> cb,
            float fallbackEyeHeight
    ) {
        TransformComponent tc = store.getComponent(ref, TransformComponent.getComponentType());
        if (tc == null) return Double.NaN;
        return tc.getPosition().y + getEyeHeight(ref, store, cb, fallbackEyeHeight);
    }

    /**
     * Returns the eye height of an entity based on its {@link ModelComponent}.
     *
     * <p>This attempts reads from {@link CommandBuffer} first, then {@link Store}. It also attempts to
     * obtain a usable {@link ComponentAccessor} from either {@code cb} or {@code store}.</p>
     *
     * @param ref entity reference
     * @param store entity store
     * @param cb command buffer
     * @param fallbackEyeHeight fallback eye height when model data isn't available
     * @return eye height (never negative); may return {@code fallbackEyeHeight}
     */
    public static float getEyeHeight(
            Ref<EntityStore> ref,
            Store<EntityStore> store,
            CommandBuffer<EntityStore> cb,
            float fallbackEyeHeight
    ) {
        ModelComponent mc = cb.getComponent(ref, ModelComponent.getComponentType());
        if (mc == null) mc = store.getComponent(ref, ModelComponent.getComponentType());
        if (mc == null) return fallbackEyeHeight;

        Model m = mc.getModel();
        if (m == null) return fallbackEyeHeight;

        ComponentAccessor<EntityStore> accessor;
        accessor = cb;

        try {
            float eye = m.getEyeHeight(ref, accessor);
            return eye > 0.0f ? eye : fallbackEyeHeight;
        } catch (Throwable ignored) {
            return fallbackEyeHeight;
        }
    }

    /**
     * Coarse combat profile derived from role configuration.
     *
     * <p><b>UNKNOWN</b> is a valid outcome and should be handled by callers.</p>
     */
    public enum AttackProfile {
        MELEE,
        RANGED,
        UNKNOWN
    }

    private static final ConcurrentMap<Integer, AttackProfile> ROLE_ATTACK_PROFILE_CACHE = new ConcurrentHashMap<>();
    private static final ConcurrentMap<Integer, Set<String>> ROLE_ATTACK_ROOT_IDS_CACHE = new ConcurrentHashMap<>();

    /**
     * Clears cached attack-profile information.
     *
     * <p>Call this when you hot-reload NPC role assets, otherwise cached results may become stale.</p>
     */
    public static void clearAttackProfileCaches() {
        ROLE_ATTACK_PROFILE_CACHE.clear();
        ROLE_ATTACK_ROOT_IDS_CACHE.clear();
    }

    /**
     * Resolves a role's {@link AttackProfile} by:
     * <ol>
     *   <li>Extracting candidate attack root interaction ids from the role JSON (via {@link NpcRoleJsonUtil}).</li>
     *   <li>Loading each {@link RootInteraction} and reading {@code Tags.Attack} for "Melee"/"Ranged".</li>
     *   <li>Degrading to {@link AttackProfile#UNKNOWN} on conflicts or missing tags.</li>
     * </ol>
     *
     * <p>This method caches results by {@code roleIndex}.</p>
     *
     * @param roleIndex role index from {@link NPCEntity#getRoleIndex()}
     * @param logger unused (kept for signature compatibility)
     * @param logTag unused (kept for signature compatibility)
     * @return resolved profile (may be {@link AttackProfile#UNKNOWN})
     */
    public static AttackProfile getAttackProfileForRole(
            int roleIndex,
            @Nullable HytaleLogger logger,
            @Nullable String logTag
    ) {
        if (roleIndex < 0) return AttackProfile.UNKNOWN;

        AttackProfile cached = ROLE_ATTACK_PROFILE_CACHE.get(roleIndex);
        if (cached != null) return cached;

        BuilderInfo bi = NPCPlugin.get().getRoleBuilderInfo(roleIndex);
        if (bi == null) {
            ROLE_ATTACK_PROFILE_CACHE.put(roleIndex, AttackProfile.UNKNOWN);
            return AttackProfile.UNKNOWN;
        }

        Path rolePath = bi.getPath();
        if (rolePath == null) {
            ROLE_ATTACK_PROFILE_CACHE.put(roleIndex, AttackProfile.UNKNOWN);
            return AttackProfile.UNKNOWN;
        }

        Set<String> attackRootIds = NpcRoleJsonUtil.resolveAttackRootInteractionIds(
                roleIndex,
                bi.getKeyName(),
                rolePath
        );

        ROLE_ATTACK_ROOT_IDS_CACHE.put(roleIndex, attackRootIds);

        AttackProfile profile = classifyFromRootInteractions(attackRootIds);
        ROLE_ATTACK_PROFILE_CACHE.put(roleIndex, profile);

        return profile;
    }

    /**
     * Convenience overload when you already have the NPC component.
     *
     * @param npc NPC component
     * @param logger unused (kept for signature compatibility)
     * @param logTag unused (kept for signature compatibility)
     * @return resolved profile (may be {@link AttackProfile#UNKNOWN})
     */
    public static AttackProfile getAttackProfileForNpc(
            @Nullable NPCEntity npc,
            @Nullable HytaleLogger logger,
            @Nullable String logTag
    ) {
        if (npc == null) return AttackProfile.UNKNOWN;
        return getAttackProfileForRole(npc.getRoleIndex(), logger, logTag);
    }

    /**
     * Returns cached attack root interaction ids for a role, if present.
     *
     * @param roleIndex role index
     * @return cached id set, or {@code null} if not cached
     */
    @Nullable
    public static Set<String> getCachedAttackRootInteractionIds(int roleIndex) {
        return ROLE_ATTACK_ROOT_IDS_CACHE.get(roleIndex);
    }

    /**
     * Classifies a role profile by inspecting all resolved root interaction ids.
     *
     * <p>If both melee and ranged are detected across ids, returns {@link AttackProfile#UNKNOWN}.</p>
     *
     * @param attackRootIds set of candidate root interaction ids
     * @return resolved profile
     */
    private static AttackProfile classifyFromRootInteractions(Set<String> attackRootIds) {
        boolean sawMelee = false;
        boolean sawRanged = false;

        for (String id : attackRootIds) {
            AttackProfile p = classifyFromSingleRootInteraction(id);
            if (p == AttackProfile.MELEE) sawMelee = true;
            if (p == AttackProfile.RANGED) sawRanged = true;
            if (sawMelee && sawRanged) return AttackProfile.UNKNOWN;
        }

        if (sawRanged) return AttackProfile.RANGED;
        if (sawMelee) return AttackProfile.MELEE;
        return AttackProfile.UNKNOWN;
    }

    /**
     * Classifies a single {@link RootInteraction} by reading {@code Tags.Attack}.
     *
     * <p>Only explicit "Melee" or "Ranged" tags are recognized; otherwise returns UNKNOWN.</p>
     *
     * @param rootInteractionId root interaction asset id
     * @return profile derived from tags
     */
    private static AttackProfile classifyFromSingleRootInteraction(String rootInteractionId) {
        if (rootInteractionId == null || rootInteractionId.isBlank()) return AttackProfile.UNKNOWN;

        RootInteraction ri = RootInteraction.getAssetMap().getAsset(rootInteractionId);
        if (ri == null || ri.getData() == null) return AttackProfile.UNKNOWN;

        String[] attackTags = ri.getData().getRawTags().get("Attack");
        if (attackTags == null) return AttackProfile.UNKNOWN;

        for (String t : attackTags) {
            if (t == null) continue;
            String s = t.toLowerCase(Locale.ROOT);
            if (s.equals("ranged")) return AttackProfile.RANGED;
            if (s.equals("melee")) return AttackProfile.MELEE;
        }

        return AttackProfile.UNKNOWN;
    }
}