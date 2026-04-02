package me.s3b4s5.npc.util;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.hypixel.hytale.server.npc.NPCPlugin;
import com.hypixel.hytale.server.npc.asset.builder.BuilderInfo;
import me.s3b4s5.util.JsonUtil;

import javax.annotation.Nullable;
import java.nio.file.Path;
import java.util.LinkedHashSet;
import java.util.Locale;
import java.util.Set;

/**
 * Role-JSON specific utilities for NPC combat configuration.
 *
 * <p>This class focuses on extracting "attack root interaction ids" from NPC Role JSON assets.
 * It exists to keep {@link NpcUtil} smaller and to isolate JSON parsing/resolution logic.
 *
 * <h2>What it resolves</h2>
 * A role can define the attack root interaction id in multiple shapes/locations:
 * <ul>
 *   <li>{@code Parameters.Attack} as a string or as {@code { "Value": "..." }}</li>
 *   <li>Top-level {@code Attack} as a string or as {@code { "Value": "..." }}</li>
 *   <li>{@code Modify.Attack} / {@code Modify.Parameters.Attack}</li>
 *   <li>Variant overrides inside {@code Variants[*].Modify.*}</li>
 * </ul>
 *
 * <h2>Inheritance rules</h2>
 * If no Attack id is found locally, resolution falls back in this order:
 * <ol>
 *   <li>{@code Reference} (Variant roles inheriting defaults from a referenced template role)</li>
 *   <li>{@code VariantOf} / {@code BaseRole} / {@code Parent} (older parent-like patterns)</li>
 * </ol>
 *
 * <h2>Safety</h2>
 * <ul>
 *   <li>Cycle protection via a {@code visited} set.</li>
 *   <li>Depth limit to avoid runaway chains (default: 12).</li>
 *   <li>No exceptions: failures simply return an empty set.</li>
 * </ul>
 */
public final class NpcRoleJsonUtil {

    private NpcRoleJsonUtil() {}

    /**
     * Resolves all plausible attack root interaction ids for the role represented by {@code roleIndex/rolePath}.
     *
     * <p>The returned set may contain 0, 1, or multiple ids. Callers should treat multiple ids as
     * "possible attacks" and handle conflicts safely (e.g. degrade to UNKNOWN profile if tags disagree).
     *
     * @param roleIndex role index (used only for identification; may be used by callers for caching)
     * @param roleName optional role name (helps build a stable visited key)
     * @param rolePath role JSON path
     * @return a set of attack root interaction ids (may be empty)
     */
    public static Set<String> resolveAttackRootInteractionIds(
            int roleIndex,
            @Nullable String roleName,
            Path rolePath
    ) {
        Set<String> out = new LinkedHashSet<>(2);
        Set<String> visited = new LinkedHashSet<>();
        String seedName = (roleName == null || roleName.isBlank()) ? ("roleIndex#" + roleIndex) : roleName;
        resolveAttackIdsRecursive(out, seedName, rolePath, visited, 0);
        return out;
    }

    private static void resolveAttackIdsRecursive(
            Set<String> out,
            String currentRoleName,
            Path currentPath,
            Set<String> visited,
            int depth
    ) {
        if (depth > 12) return;

        String visitKey = currentRoleName == null ? ("<null>@" + currentPath) : currentRoleName.toLowerCase(Locale.ROOT);
        if (!visited.add(visitKey)) return;

        JsonObject root = JsonUtil.readJsonObject(currentPath);
        if (root == null) return;

        int before = out.size();

        addIfNotBlank(out, readAttackIdFromParameters(root));
        addIfNotBlank(out, readAttackIdDirect(root));

        JsonObject modify = JsonUtil.obj(root, "Modify");
        if (modify != null) {
            addIfNotBlank(out, readAttackIdDirect(modify));
            addIfNotBlank(out, readAttackIdFromParameters(modify));
        }

        JsonElement variantsEl = root.get("Variants");
        if (variantsEl != null) {
            if (variantsEl.isJsonArray()) {
                JsonArray arr = variantsEl.getAsJsonArray();
                for (int i = 0; i < arr.size(); i++) {
                    JsonObject v = arr.get(i).isJsonObject() ? arr.get(i).getAsJsonObject() : null;
                    if (v == null) continue;

                    resolveAttackIdsFromVariant(out, visited, depth, v);
                }
            } else if (variantsEl.isJsonObject()) {
                JsonObject vObj = variantsEl.getAsJsonObject();
                for (String k : vObj.keySet()) {
                    JsonElement e = vObj.get(k);
                    if (!e.isJsonObject()) continue;
                    JsonObject v = e.getAsJsonObject();

                    resolveAttackIdsFromVariant(out, visited, depth, v);
                }
            }
        }

        if (out.size() == before) {
            String refName = JsonUtil.str(root, "Reference");
            if (refName != null && !refName.isBlank()) {
                resolveFromRoleNameIfNeeded(out, refName, visited, depth + 1);
            }

            if (out.size() == before) {
                String parentName = JsonUtil.str(root, "VariantOf");
                if (parentName == null) parentName = JsonUtil.str(root, "BaseRole");
                if (parentName == null) parentName = JsonUtil.str(root, "Parent");

                if (parentName != null && !parentName.isBlank()) {
                    resolveFromRoleNameIfNeeded(out, parentName, visited, depth + 1);
                }
            }
        }
    }

    private static void resolveAttackIdsFromVariant(Set<String> out, Set<String> visited, int depth, JsonObject v) {
        JsonObject vModify = JsonUtil.obj(v, "Modify");
        String vAttack = (vModify != null) ? readAttackIdDirect(vModify) : null;
        String vParamAttack = (vModify != null) ? readAttackIdFromParameters(vModify) : null;

        addIfNotBlank(out, vAttack);
        addIfNotBlank(out, vParamAttack);

        if ((vAttack == null || vAttack.isBlank()) && (vParamAttack == null || vParamAttack.isBlank())) {
            String vRef = JsonUtil.str(v, "Reference");
            if (vRef != null && !vRef.isBlank()) {
                resolveFromRoleNameIfNeeded(out, vRef, visited, depth + 1);
            }
        }
    }

    private static void resolveFromRoleNameIfNeeded(
            Set<String> out,
            String roleName,
            Set<String> visited,
            int depth
    ) {
        int idx = NPCPlugin.get().getIndex(roleName);
        if (idx < 0) return;

        BuilderInfo bi = NPCPlugin.get().getRoleBuilderInfo(idx);
        if (bi == null || bi.getPath() == null) return;

        resolveAttackIdsRecursive(out, bi.getKeyName(), bi.getPath(), visited, depth);
    }

    @Nullable
    private static String readAttackIdFromParameters(JsonObject root) {
        JsonObject params = JsonUtil.obj(root, "Parameters");
        if (params == null) return null;
        return readAttackIdValue(params.get("Attack"));
    }

    @Nullable
    private static String readAttackIdDirect(JsonObject root) {
        if (root == null) return null;
        return readAttackIdValue(root.get("Attack"));
    }

    @Nullable
    private static String readAttackIdValue(@Nullable JsonElement el) {
        if (el == null) return null;

        if (el.isJsonPrimitive() && el.getAsJsonPrimitive().isString()) {
            return el.getAsString();
        }

        if (el.isJsonObject()) {
            JsonObject o = el.getAsJsonObject();
            JsonElement v = o.get("Value");
            if (v != null && v.isJsonPrimitive() && v.getAsJsonPrimitive().isString()) {
                return v.getAsString();
            }
        }

        return null;
    }

    private static void addIfNotBlank(Set<String> out, @Nullable String s) {
        if (s != null && !s.isBlank()) out.add(s);
    }
}