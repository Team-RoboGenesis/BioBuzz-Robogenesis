package org.firstinspires.ftc.teamcode.tests;

import android.content.Context;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.RandomAccessFile;

/*
This is a SD card manipulator for FTC vibe coded by team #26235 robogenisis

how to use:

init: FileUtil.init(hardwareMap.appContext);

reading: FileUtil.readRange(starting byte, ending byte);
appending: FileUtil.appendLine(String);
writing from an index: FileUtil.writeStringAtIndex(String, starting byte);
truncating writing from an index: FileUtil.overwriteFromIndex(String, starting byte);
*/

public class fileUtil {

    private static Context context;
    private static String fileName = "sd_log.txt"; // default fallback

    // ✅ Initialize with custom file name
    public static void init(Context ctx, String fileName) {
        context = ctx;

        // Ensure it ends with .txt
        if (!fileName.endsWith(".txt")) {
            fileName = fileName + ".txt";
        }

        fileUtil.fileName = fileName;
    }

    // ✅ Get file (uses chosen name)
    private static File getFile() throws IOException {
        if (context == null) {
            throw new IllegalStateException("FileUtil not initialized");
        }

        File[] dirs = context.getExternalFilesDirs(null);

        File targetDir;

        if (dirs.length > 1 && dirs[1] != null) {
            targetDir = dirs[1]; // SD card
        } else {
            targetDir = dirs[0]; // internal fallback
        }

        if (targetDir == null) {
            throw new IOException("No storage directory available");
        }

        File file = new File(targetDir, fileName);

        if (!file.exists()) {
            file.createNewFile();
        }

        return file;
    }

    // ✅ Append line
    public static void appendLine(String line) {
        try {
            FileWriter writer = new FileWriter(getFile(), true);
            writer.write(line + "\n");
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // ✅ Read range
    public static String readRange(long startIndex, long endIndex) {
        try (RandomAccessFile raf = new RandomAccessFile(getFile(), "r")) {

            long fileLength = raf.length();

            if (startIndex >= fileLength) return "";
            if (endIndex > fileLength) endIndex = fileLength;
            if (endIndex < startIndex) return "";

            int length = (int)(endIndex - startIndex);
            byte[] buffer = new byte[length];

            raf.seek(startIndex);
            raf.readFully(buffer);

            return new String(buffer);

        } catch (IOException e) {
            e.printStackTrace();
            return "";
        }
    }

    // ✅ Write at index
    public static void writeStringAtIndex(String data, long index) {
        try (RandomAccessFile raf = new RandomAccessFile(getFile(), "rw")) {
            raf.seek(index);
            raf.write(data.getBytes());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // 🔥 Overwrite from index
    public static void overwriteFromIndex(String data, long index) {
        try (RandomAccessFile raf = new RandomAccessFile(getFile(), "rw")) {

            long fileLength = raf.length();
            if (index > fileLength) {
                index = fileLength;
            }

            raf.setLength(index);
            raf.seek(index);
            raf.write(data.getBytes());

        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}