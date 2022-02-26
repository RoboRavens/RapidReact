package frc.util;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.TimeZone;

public class FileWriter {
    private static String FOLDER_NAME = "/home/lvuser/Logs";

    private String _fileName;
    private PrintWriter _writer;

    public FileWriter(String intent) {
        try {
			File file = new File(FOLDER_NAME);
			if (!file.exists()) {
				if (file.mkdir()) {
					System.out.println("Log Directory is created!");
				} else {
					System.out.println("Failed to create Log directory!");
				}
			}
            
			Date date = new Date();
			SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd HH-mm-ss");
			dateFormat.setTimeZone(TimeZone.getTimeZone("EST5EDT"));
            _fileName = "/home/lvuser/Logs/" + dateFormat.format(date) + "-" + intent + ".txt";
			_writer = new PrintWriter(_fileName, "UTF-8");
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (UnsupportedEncodingException e) {
			e.printStackTrace();
		}
    }

    public void write(String content) {
        _writer.print(content);
        _writer.flush();
    }

    public void writeLine(String content) {
        _writer.println(content);
        _writer.flush();
    }
}
