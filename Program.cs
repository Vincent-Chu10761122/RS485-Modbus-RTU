// See https://aka.ms/new-console-template for more information
using System;
using System.IO.Ports;

namespace ModbusRTU 
{ 
    public class Program 
    { 
        static void Main(string[] args) 
        { 
            SerialPort port = new SerialPort("COM6", 9600, Parity.None, 8, StopBits.One);
            port.Open();
            Console.WriteLine($"port open");

            byte[] request = new byte[] {0x01, 0x03, 0x9C, 0x41, 0x00, 0x2E, 0xBB, 0x92};
            // byte slaveId = 1;
            // byte functionCode = 3;
            // ushort startingAddress = 0;
            // ushort numRegisters = 10;

            // byte[] message = new byte[] {slaveId, functionCode, (byte)(startingAddress >> 8), (byte)startingAddress, (byte)(numRegisters >> 8), (byte)numRegisters };
            // Console.WriteLine($"crc");
            // byte[] crc = CalculateCRC(message);
            // Console.WriteLine($"crc end");
            // byte[] request = new byte[message.Length + crc.Length];
            // message.CopyTo(request, 0);
            // crc.CopyTo(request, message.Length);

            port.Write(request, 0, request.Length);
            Console.WriteLine($"write");
            Thread.Sleep(3000);
            byte[] response = new byte[1024];
            port.Read(response, 0, response.Length);
            Console.WriteLine($"response {response.Length}");

            for (int i = 0; i < 97; i++)
            {
                ushort registerValue = (ushort)(response[i * 2] << 8 | response[i * 2 + 1]);
                Console.WriteLine($"Register {i}: {registerValue}");
            }

            port.Close();
        }

        static byte[] CalculateCRC(byte[] data)
        {
        ushort crc = 0xFFFF;

        for (int i = 0; i < data.Length; i++)
        {
            crc ^= data[i];

            for (int j = 0; j < 8; j++)
            {
                if ((crc & 0x0001) == 0x0001)
                {
                    crc >>= 1;
                    crc ^= 0xA001;
                }
                else
                {
                    crc >>= 1;
                }
            }
        }

        return new byte[] { (byte)(crc & 0xFF), (byte)(crc >> 8) };
        }
    }

    
}