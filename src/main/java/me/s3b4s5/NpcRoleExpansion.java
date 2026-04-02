package me.s3b4s5;

import com.hypixel.hytale.logger.HytaleLogger;
import com.hypixel.hytale.server.core.plugin.JavaPlugin;
import com.hypixel.hytale.server.core.plugin.JavaPluginInit;
import me.s3b4s5.registries.NpcBuilderRegistry;
import org.checkerframework.checker.nullness.compatqual.NonNullDecl;

public class NpcRoleExpansion extends JavaPlugin {

    public static HytaleLogger LOGGER = HytaleLogger.forEnclosingClass();

    public NpcRoleExpansion(@NonNullDecl JavaPluginInit init) {
        super(init);
    }

    @Override
    protected void setup() {
        NpcBuilderRegistry.register(this);
    }
}