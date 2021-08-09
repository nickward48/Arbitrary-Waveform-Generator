using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Threading;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.IO;
using System.IO.Ports;
using System.Windows.Forms;
using Microsoft.WindowsAPICodePack.Dialogs;

using System.Security.Cryptography.X509Certificates;
using System.Diagnostics.Tracing;
using System.Linq.Expressions;

namespace AWG_UI
{
    /// <summary>
    /// Program for Arbitrary Waveform Generator User Interface
    /// Meant to have user specifiy waveforms then send data to microcontroller
    /// </summary>
    /// 

    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            //FINDING LOCAL COMPORTS AND ADDING THEM TO STRING "ports"
            InitializeComponent();
            string[] ports = SerialPort.GetPortNames();


            //ADD ALL COMPORTS TO TO COMBO BOX FOR USER SELECTION
            foreach (string item in ports)
            {
                cBoxComPort.Items.Add(item);
            }

            SerialPort serialPort1; //declare serial port
            serialPort1 = new SerialPort(); //define serial port
        }


        //**************************************** WAVEFORM SELECTION *****************************************
        #region Waveform Selection
        private void ApplyButton1_Click(object sender, RoutedEventArgs e)
        {
            //Validating textbox inputs
            //-------------------- Waveform selection --------------------
            //Amplitude textbox
            int parsedValue;
            if (!int.TryParse(AmplitudeTextbox.Text, out parsedValue))
            {
                System.Windows.MessageBox.Show("Amplitude textbox must only contain numbers only");
                return;
            }

            else
            {
                //check for value within +/- 5 V
                if (parsedValue > 5 || parsedValue < -5)
                {
                    System.Windows.MessageBox.Show("Amplitude textbox must be within +/- 5 ");
                    return;
                }

                else
                {
                    //Send data
                }

            }

            //Output frequency textbox
            if (!int.TryParse(FrequencyTextbox1.Text, out parsedValue))
            {
                System.Windows.MessageBox.Show("Frequency textbox must only contain numbers only");
                return;
            }

            else
            {
                //Send data
            }

            //Number of sample points textbox

            if (!int.TryParse(SampleTextbox1.Text, out parsedValue))
            {
                System.Windows.MessageBox.Show("Sample points textbox must only contain numbers only");
                return;
            }

            else
            {
                //validating max number of sample points
                if (int.Parse(SampleTextbox1.Text) > (1000000 / int.Parse(FrequencyTextbox1.Text)))
                {
                    System.Windows.MessageBox.Show($"Based on frequency of {this.FrequencyTextbox1.Text}, the number of sample points must be {1000000 / int.Parse(FrequencyTextbox1.Text)} or less.");
                    return;
                }

                else
                {
                    //send data
                    //need to add checks for sending certain waveforms
                    //Ex: If sine is checked, send sine, if none is checked need to ask user for input

                    // Set a variable to the Documents path.
                    string docPath = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);

                    // Set a variable to the Documents path.
                    string[] lines = { this.FrequencyTextbox1.Text, this.SampleTextbox1.Text };

                    // Write the string array to a new file named "WriteLines.txt".
                    using (StreamWriter outputFile = new StreamWriter(System.IO.Path.Combine(docPath, "Sample.txt"), true))
                    {
                        foreach (string line in lines)
                            outputFile.WriteLine(line);
                    }

                    System.Windows.MessageBox.Show("Data sent!");

                }
            }



        }

        private void ResetButton1_Click(object sender, RoutedEventArgs e)
        {

        }

        #endregion

        //**************************************** USER DEFINED WAVEFORM ***********************************************
        #region Arbitrary Waveform Upload and Send

        //adding Global Class in order to access file path and file content throughout program
        public static class MyGlobals
        {
            public static string filePath = string.Empty;
            public static SerialPort serialPort1 = new SerialPort(); //declare and define serial port
        }

        public void FolderButton_Click(object sender, RoutedEventArgs e)
        {

            //Validating textbox inputs
            //-------------------- Waveform selection --------------------
            //Amplitude textbox

            //Open file specified by user
            try
            {
                using (OpenFileDialog openFileDialog = new OpenFileDialog())
                {
                    openFileDialog.InitialDirectory = "c:\\";
                    openFileDialog.Filter = "txt files (*.txt)|*.txt"; //ALLOW ONLY .TXT FILES
                    openFileDialog.FilterIndex = 2;
                    openFileDialog.RestoreDirectory = true;

                    if (openFileDialog.ShowDialog() == System.Windows.Forms.DialogResult.OK)
                    {
                        //Get the path of specified file
                        MyGlobals.filePath = openFileDialog.FileName;
                    }
                }
            }

            //Give error if cannot open file
            catch (Exception err)
            {
                System.Windows.Forms.MessageBox.Show(err.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }

        }

        //converts data received in file to data necessasry for DAC
        // 2/23/21 changed from returning an int to returning byte array
        public byte[] convertToDAC(String lineFromFile)
        {
            int startingIndex = 0;
            int endingIndex = 0;
            double voltageRef = 2.5; //voltage reference on DAC for conversion 
            //double voltageDAC = 0; //decimal voltage data
            int convertVoltage = 0; //initialize voltage converted from file to int (needs to be int in order to perform 1s complement)
            byte[] returnVoltageDAC;

            startingIndex = lineFromFile.IndexOf(' '); //find first space in the line -- this is the index right before the voltage value in the users input file.

            endingIndex = lineFromFile.IndexOf(';'); //gets ending index, marked by a ";"

            String voltageDataString = lineFromFile.Substring(startingIndex+1, (endingIndex-1) - startingIndex); //voltage data from file is now stored as substring

            double voltageData = Convert.ToDouble(voltageDataString); //now converting previous substring to double
            //System.Windows.Forms.MessageBox.Show(voltageDataString);

            //Now using equation (Vout * 32768)/Vref = D where Vout is voltage from file (AKA voltageData) and D is the data loaded into the DAC's busses (voltageDAC)
            if (voltageData >= 0)
            {
                //doing multiple steps to truncate data from file and then convert to 16 bit integer to avoid any exceptions or unallowed conversions
                voltageData = (voltageData * 32768) / voltageRef;
                if (voltageData == 65536)
                {
                    voltageData--;
                }
                voltageData = Math.Round(voltageData);
                convertVoltage = Convert.ToInt32(voltageData); //16 bit integer stored in int variable, need to only send first 2 bytes
                returnVoltageDAC = BitConverter.GetBytes(convertVoltage); //getting bytes to return
            }
            //System.Windows.Forms.MessageBox.Show(convertVoltage.ToString);

            //negative voltage reference for bipolar output
            //NOTE: DAC has all zero binary input for lowest voltage level (-5 V), so need to perform 1s complement
            else
            {
                voltageData = (voltageData * 32768) / -voltageRef;
                if (voltageData == 65536)
                {
                    voltageData--;
                }
                voltageData = Math.Round(voltageData);
                convertVoltage = Convert.ToInt32(voltageData);
                convertVoltage = ~convertVoltage; //perform 1s complement
                returnVoltageDAC = BitConverter.GetBytes(convertVoltage); //getting bytes to return
            }
     
            //Appropriate binary value to load to DAC will now be returned
            return returnVoltageDAC;
        }

        //function to return address of data
        public byte[] dataAddress(String lineFromFile)
        {
            int startingIndex = 0;
            int endingIndex = 0;

            endingIndex = lineFromFile.IndexOf(','); //gets ending index, marked by a ","

            String addressDataString = lineFromFile.Substring(startingIndex, (endingIndex) - startingIndex);
            Int16 addressDataInt = Convert.ToInt16(addressDataString);

            byte[] addressToReturn = BitConverter.GetBytes(addressDataInt);

            return addressToReturn;

        }

        public void ApplyButton2_Click(object sender, RoutedEventArgs e)
        {

            //Validating textbox inputs
            //-------------------- Waveform selection --------------------
            //Amplitude textbox
            int parsedValue;

            //Output frequency textbox
            if (!int.TryParse(FrequencyTextbox2.Text, out parsedValue))
            {
                System.Windows.MessageBox.Show("Frequency textbox must contain numbers only");
                return;
            }

            //Number of sample points textbox
            //number of sample points will be defined by user
            // microcontroller program will perform operation 64kb/(# of sample points defined) = floor(distance between consecutive sample points)

            if (!int.TryParse(SampleTextbox2.Text, out parsedValue))
            {
                System.Windows.MessageBox.Show("Sample points textbox must contain numbers only");
                return;
            }

            else
            {
                //validating max number of sample points
                if (int.Parse(SampleTextbox2.Text) > (1000000 / int.Parse(FrequencyTextbox2.Text)))
                {
                    System.Windows.MessageBox.Show($"Based on frequency of {this.FrequencyTextbox2.Text}, the number of sample points must be {1000000 / int.Parse(FrequencyTextbox2.Text)} or less.");
                    return;
                }

                else
                {

                    //************ SENDING DATA ******************
                    //if serial port is open, send data uploaded by user
                    if (MyGlobals.serialPort1.IsOpen)
                    {

                        using (StreamReader sr = new StreamReader(MyGlobals.filePath)) //open user specified file path
                        {
                            String line;
                            decimal skipLines;
                            skipLines = 65535 / int.Parse(SampleTextbox2.Text);
                            skipLines = Math.Round(skipLines);
                            byte[] DAC_Data; //creating byte array for data that will be returned by convertToDac function
                            byte[] data_Address; //creating byte array for data that will be returned by dataAddress function

                            //int firstLine = 1; //sends first line before starting counting
                            //int linesSent = 0; //counts number of samples sent
                            int lineCounter = 0; //used to count number of lines between samples sent

                            byte[] beginTransfer = new byte[1];
                            beginTransfer[0] = 0; //for communication with the MCU
                            byte[] numSamples = BitConverter.GetBytes(UInt16.Parse(SampleTextbox2.Text));
                            byte[] definedFrequency = BitConverter.GetBytes(UInt16.Parse(FrequencyTextbox2.Text));
                            
                            
                            if (BitConverter.IsLittleEndian)
                            {

                                //If computer architecture is Little Endian, reverse the entire array
                                Array.Reverse(numSamples, 0, numSamples.Length);
                                Array.Reverse(definedFrequency, 0, numSamples.Length);

                            }
                            

                            //Thread.Sleep(300);                    
                            //NOTE: Need to send Length-1 because in C languages, all Strings (character arrays) are null terminated (have \0 value at end)
                            MyGlobals.serialPort1.Write(numSamples, 0, 2); //Write the number of sample points to the MCU in Byte Array.
                            MyGlobals.serialPort1.Write(definedFrequency, 0, 2);


                            //Sending contents of file
                            
                            while (sr.EndOfStream == false)
                            {
                                if (lineCounter != 1023) //change to 1023 in final version
                                {
                                    line = sr.ReadLine(); //read line from file
                                    DAC_Data = convertToDAC(line); //converting line from file to form readable by DAC
                                    data_Address = dataAddress(line); // getting index from line

                                    //reverse Byte arrays if computer architecture is little endian
                                    if (BitConverter.IsLittleEndian)
                                    {
                                        //MyGlobals.serialPort1.WriteLine("True");

                                        Array.Reverse(DAC_Data, 0, DAC_Data.Length);
                                        Array.Reverse(data_Address, 0, data_Address.Length);
                                    }

                                    MyGlobals.serialPort1.Write(data_Address, 0, 2); //write address to port
                                    MyGlobals.serialPort1.Write(DAC_Data, 2, 2); //write data to port

                                    //MyGlobals.serialPort1.WriteLine("Line from file: " + line);
                                    //MyGlobals.serialPort1.WriteLine("Converted file value for DAC (HEX): " + BitConverter.ToString(DAC_Data) );
                                    //MyGlobals.serialPort1.WriteLine("Address: " + BitConverter.ToString(data_Address));

                                    lineCounter++;
                                }
                                
                                else
                                {
                                    while (beginTransfer[0] != 66) //waiting to receive command 'B' from MCU to begin transfer
                                    {
                                        MyGlobals.serialPort1.Read(beginTransfer, 0, 1);
                                    }

                                    System.Windows.MessageBox.Show("Command received");

                                    beginTransfer[0] = 0;
                                    lineCounter = 0;
                                }
                                

                                // **NOTE** This program is little endian
                                // MSB is last byte in sequence (Ex: (39322)decimal = (999A)hex, will be stored in array as (9A-99) )
                                // 9A = bufferToSend[0], 99 = bufferToSend[1]
                                //Can use Array.Reverse to reverse bits if using Little Endian architecture
                                //Check BitConverter Documentation if confused
                                
                                
                              
                              

                            }
                            
                        }
                    }
                }
            }
        }

        //OPENING SERIAL PORT
        private void openBtn_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                //*************** MICROCONTROLLER NEEDS TO BE SET TO RECEIVE DATA IN SAME WAY AS DEFINED  BELOW *****************************
                MyGlobals.serialPort1.PortName = cBoxComPort.Text;
                MyGlobals.serialPort1.BaudRate = 9600; //Currently set to typical max Baud Rate for serial ports
                MyGlobals.serialPort1.DataBits = 8; //8 is max number of bits possible for data field.
                MyGlobals.serialPort1.StopBits = StopBits.One; //One stop bit
                MyGlobals.serialPort1.Parity = Parity.None; //No parity bits will be used

                MyGlobals.serialPort1.Open();
                progressBar.Value = 100;

            }

            //Give user error if COM Port cannot be opened
            catch (Exception err)
            {
                System.Windows.Forms.MessageBox.Show(err.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void closeBtn_Click(object sender, RoutedEventArgs e)
        {
            if (MyGlobals.serialPort1.IsOpen)
            {
                MyGlobals.serialPort1.Close();
                progressBar.Value = 0;
            }
        }

        private void ResetButton2_Click(object sender, RoutedEventArgs e)
        {
            /*
             * 
             * 
             * if (File.Exists(path))
                 {
                     File.Delete(path);
                 }
             * 
             * 
             */


        }

        private void cBoxComPort_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {

        }
        #endregion
    }
}
