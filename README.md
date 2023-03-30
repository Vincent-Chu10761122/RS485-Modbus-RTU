# RS485-Modbus-RTUusing System;
using System.IO.Ports;

namespace ModbusRTU
{
    class Program
    {
        static void Main(string[] args)
        {
            SerialPort port = new SerialPort("COM1", 9600, Parity.None, 8, StopBits.One);
            port.Open();

            byte slaveId = 1;
            byte functionCode = 3;
            ushort startingAddress = 0;
            ushort numRegisters = 10;

            byte[] message = new byte[] { slaveId, functionCode, (byte)(startingAddress >> 8), (byte)startingAddress, (byte)(numRegisters >> 8), (byte)numRegisters };
            byte[] crc = CalculateCRC(message);

            byte[] request = new byte[message.Length + crc.Length];
            message.CopyTo(request, 0);
            crc.CopyTo(request, message.Length);

            port.Write(request, 0, request.Length);

            byte[] response = new byte[numRegisters * 2];
            port.Read(response, 0, response.Length);

            for (int i = 0; i < numRegisters; i++)
            {
                ushort registerValue = (ushort)(response[i * 2] << 8 | response[i * 2 + 1]);
                Console.WriteLine($"Register {startingAddress + i}: {registerValue}");
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
