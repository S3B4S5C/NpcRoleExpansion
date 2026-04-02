package me.s3b4s5.registries;

import com.hypixel.hytale.event.EventRegistry;
import com.hypixel.hytale.server.core.asset.LoadAssetEvent;
import com.hypixel.hytale.server.core.plugin.JavaPlugin;
import com.hypixel.hytale.server.npc.NPCPlugin;
import com.hypixel.hytale.server.npc.asset.builder.BuilderFactory;
import com.hypixel.hytale.server.npc.asset.builder.BuilderManager;
import com.hypixel.hytale.server.npc.instructions.BodyMotion;
import com.hypixel.hytale.server.npc.movement.controllers.MotionController;
import me.s3b4s5.NpcRoleExpansion;
import me.s3b4s5.npc.movement.builders.BuilderBodyMotionMaintainDistanceFlyRing;
import me.s3b4s5.npc.movement.builders.BuilderMotionControllerFlyFreeLook;
import me.s3b4s5.npc.movement.controllers.MotionControllerFlyFreeLook;

public class NpcBuilderRegistry {
    public NpcBuilderRegistry() {
    }

    public static void register(JavaPlugin plugin) {
        EventRegistry events = plugin.getEventRegistry();

        short prio = (short) (NPCPlugin.PRIORITY_LOAD_NPC - 1);

        events.register(prio, LoadAssetEvent.class, ev -> {
            NPCPlugin npc = NPCPlugin.get();
            if (npc == null) {
                NpcRoleExpansion.LOGGER.atWarning().log("NPCPlugin not available yet; cannot register NPC builders.");
                return;
            }

            BuilderManager bm = npc.getBuilderManager();

            BuilderFactory<BodyMotion> bodyMotionFactory = bm.getFactory(BodyMotion.class);
            bodyMotionFactory.add(
                    BuilderBodyMotionMaintainDistanceFlyRing.TYPE,
                    BuilderBodyMotionMaintainDistanceFlyRing::new
            );

            BuilderFactory<MotionController> motionFactory = bm.getFactory(MotionController.class);
            motionFactory.add(
                    MotionControllerFlyFreeLook.TYPE,
                    BuilderMotionControllerFlyFreeLook::new
            );

            NpcRoleExpansion.LOGGER.atInfo().log(
                    "Registered NPC builders: BodyMotion=%s, MotionController=%s",
                    BuilderBodyMotionMaintainDistanceFlyRing.TYPE,
                    MotionControllerFlyFreeLook.TYPE
            );
        });
    }
}
