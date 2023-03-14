import java.io.File;
import java.io.FileInputStream;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Scanner;

import javax.swing.JFileChooser;
import javax.swing.filechooser.FileNameExtensionFilter;
import javax.swing.filechooser.FileSystemView;

import com.fazecast.jSerialComm.SerialPort;

class SerialMonitor {

    static final int EEPROM_SIZE = 0x2000;
    static final int SERIAL_BLOCK_SIZE = 60;

    public static void main (String[] args) {
        System.out.println("Available devices: ");

        SerialPort[] devicePorts = SerialPort.getCommPorts();
        for (int i = 0; i < devicePorts.length; i++){
            SerialPort each_port = devicePorts[i];
            System.out.println("[" + i + "]: " +each_port.getSystemPortName());
        }
        System.out.print("Select a COM port: ");
        Scanner sc = new Scanner(System.in);
        int index = sc.nextInt();
        if(index > devicePorts.length){
            System.err.println("Index out of bounds");
            sc.close();
            return;
        }

        SerialPort selectedPort = devicePorts[index];
        selectedPort.setBaudRate(115200);

        selectedPort.openPort();
        selectedPort.setComPortTimeouts(SerialPort.TIMEOUT_READ_SEMI_BLOCKING, 0, 0);

        if(!waitForMessageAndTimeout(selectedPort, "bootup")){
            selectedPort.closePort();
            sc.close();
            return;
        }

        System.out.println("****************" + selectedPort.getSystemPortName() + "****************");
        
        while(true){
            int functionSelection = 0;
            do{
                System.out.println("Select a function: ");
                System.out.println("1: Write file to EEPROM");
                System.out.println("2: Read from EEPROM");
                System.out.println("3: Erase EEPROM");
                System.out.println("4: Exit");
                functionSelection = sc.nextInt();
            }while(functionSelection == 0 || functionSelection > 4);
    
            switch(functionSelection){
                case 1:{
                    writeToEEPROM(selectedPort, 0);
                    break;
                }
                case 2:{
                    readEEPROM(selectedPort, 0, 128);
                    break;
                }
                case 3:{
                    eraseEEPROM(selectedPort);
                    break;
                }
                case 4:{
                    System.out.println("Bye!");
                    sc.close();
                    selectedPort.closePort();
                    return;
                }
            }
        }
    }

    public static void readEEPROM(SerialPort port, int from, int to){
        try{
            String header = "read_" + from + " " + to + "+\n";
            port.writeBytes(header.getBytes(), header.length());
            
            InputStream in = port.getInputStream();
            if(!waitForMessageAndTimeout(port, "reading")){
                in.close();
                System.out.println("Could not read from EEPROM");
                return;
            }
            
            int add = from;
            // Every byte is equal to two chars in text
            while(add != (to+1)*2){
                System.out.print((char)in.read());
                if((add-from)%2 == 1) System.out.print(" ");
                if((add-from)%16 == 15) System.out.println();
                add++;
            }

            System.out.println();

            waitForMessageAndTimeout(port, "done");

            in.close();
        }catch(Exception e){
            e.printStackTrace();
        }
    }

    public static void writeToEEPROM(SerialPort port, int startDirection){
        JFileChooser j = new JFileChooser("./6502", FileSystemView.getFileSystemView());
        j.setAcceptAllFileFilterUsed(false);
        j.setDialogTitle("Select a .hex file");
        FileNameExtensionFilter restrict = new FileNameExtensionFilter("Only binary files", "hex", "out");
        j.addChoosableFileFilter(restrict);

        int r = j.showOpenDialog(null);
        if(r == JFileChooser.APPROVE_OPTION){

            // First, erase the existing data in EEPROM. Fill it with 0s.
            eraseEEPROM(port);

            // Start the data sending
            File hexFile = j.getSelectedFile();
            long fileSize = hexFile.length();

            System.out.println("Writing file " + hexFile.getName() + " (" + fileSize + " B)");

            OutputStream out = port.getOutputStream();
            try {
                // Send the command
                FileInputStream fileStream = new FileInputStream(hexFile);
                String header = "write_0 " + fileSize + "\n";
                out.write(header.getBytes());

                // Hold for device confirmation
                waitForMessageAndTimeout(port, "writing");

                //Start writing
                int writeDirection = startDirection;
                int packetCount = 0;
                long totalPackets = fileSize/SERIAL_BLOCK_SIZE;
                while(fileStream.available() > 0){
                    out.write(fileStream.read());
                    writeDirection++;
                    if((writeDirection-startDirection)%SERIAL_BLOCK_SIZE == 0){ 
                        System.out.println("Packet " + packetCount++ +"/" + totalPackets);
                        waitForMessageAndTimeout(port, "ok");
                    }
                }

                fileStream.close();
                out.close();

                if(waitForMessageAndTimeout(port, "done")){
                    System.out.println("Done writing!");
                }else{
                    System.out.println("Couldn't write to the EEPROM");
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }else{
            System.out.println("User cancelled the operation.");
        }
    }

    public static boolean checkWrite(SerialPort port, File file){
        
        return true;
    }

    public static void eraseEEPROM(SerialPort port){
        OutputStream out = port.getOutputStream();
        try {
            String header = "erase\n";
            out.write(header.getBytes());
            out.close();

            if(!waitForMessageAndTimeout(port, "erasing")){
                System.out.println("Could not erase EEPROM");
                return;
            }

            waitForMessage(port, "done");
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static boolean waitForMessageAndTimeout(SerialPort port, String message){
        int tryCount = 0;
        System.out.print("Waiting for <" + message + "> message");
        long startTime = System.currentTimeMillis();
        while(port.bytesAvailable() < message.length() || (message.equals("") && port.bytesAvailable()==0)){
            if((System.currentTimeMillis()-startTime)>1000){
                System.out.print(".");
                if(tryCount++ >= 10){
                    System.err.print(" < Timeout! Received: ");
                    byte[] conf = new byte[port.bytesAvailable()];
                    port.readBytes(conf, conf.length);
                    System.out.println(new String(conf));
                    return false;
                }
                startTime = System.currentTimeMillis();
            }
        }

        if(message.equals("")){
            System.out.println( " < Received");
            return true;
        }

        byte[] conf = new byte[message.length()];
        port.readBytes(conf, message.length());
        String confStr = new String(conf);
        if(confStr.compareTo(message) == 0){
            System.out.println( " < Received");
            return true;
        }else{
            System.err.println("\nCould not connect, wrong confirmation package -> " + confStr + " [" + confStr.length() + "]");
            return false;
        }
    }

    public static void waitForMessage(SerialPort port, String message){
        System.out.print("Waiting for <" + message + "> message");
        String receivedMessage = "";
        while(!receivedMessage.equals(message)){
            if(port.bytesAvailable() > 0){
                byte[] conf = new byte[port.bytesAvailable()];
                port.readBytes(conf, conf.length);
                receivedMessage += new String(conf);

                if(receivedMessage.endsWith("\n")){
                    System.out.print(receivedMessage);
                    receivedMessage = "";
                }
            }
        }
    }
}

