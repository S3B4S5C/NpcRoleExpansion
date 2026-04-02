package me.s3b4s5.util;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import javax.annotation.Nullable;
import java.io.BufferedReader;
import java.nio.file.Files;
import java.nio.file.Path;

public final class JsonUtil {

    private JsonUtil() {}

    @Nullable
    public static JsonObject readJsonObject(Path path) {
        try (BufferedReader br = Files.newBufferedReader(path)) {
            JsonElement parsed = JsonParser.parseReader(br);
            return (parsed != null && parsed.isJsonObject()) ? parsed.getAsJsonObject() : null;
        } catch (Throwable t) {
            return null;
        }
    }

    @Nullable
    public static JsonObject obj(JsonObject parent, String key) {
        if (parent == null || key == null) return null;
        JsonElement e = parent.get(key);
        return (e != null && e.isJsonObject()) ? e.getAsJsonObject() : null;
    }

    @Nullable
    public static String str(JsonObject parent, String key) {
        if (parent == null || key == null) return null;
        JsonElement e = parent.get(key);
        return (e != null && e.isJsonPrimitive() && e.getAsJsonPrimitive().isString()) ? e.getAsString() : null;
    }
}