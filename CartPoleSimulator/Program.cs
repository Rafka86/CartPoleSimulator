using System;
using System.Net;
using System.Net.Sockets;
using Rafka.MathLib.Utils;
using Rafka.MathLib.Real;
using Rafka.MathLib.Real.Numerics;
using static System.Console;
using static System.Math;

namespace CartPoleSimulator
{
    enum Command
    {
        GET,
        MOV,
        STP,
        RST,
    }

    struct Packet
    {
        public long command;
        public double[] data;
    }

    class Program
    {
        const short port = 17997;

        private static void Echo(Socket client)
        {
            var buf = new byte[256];

            while (client.Receive(buf) > 0) {
                var packet = new Packet() {
                    command = BitConverter.ToInt64(buf, 0),
                    data = new double[4]
                };
                for (int i = 0; i < packet.data.Length; i++)
                    packet.data[i] = BitConverter.ToDouble(buf, 8 * (i + 1));
                switch ((Command)packet.command) {
                    case Command.GET: {
                            Error.WriteLine("Command : GET");
                            packet.data[0] = packet.data[1] = packet.data[2] = packet.data[3] = 1.0;
                            Buffer.BlockCopy(BitConverter.GetBytes(packet.command), 0, buf, 0, 8);
                            for (int i = 0; i < packet.data.Length; i++)
                                Buffer.BlockCopy(BitConverter.GetBytes(packet.data[i]), 0, buf, 8 * (i + 1), 8);
                            if (client.Send(buf, 40, SocketFlags.None) <= 0)
                                Error.WriteLine("Sending Error.");
                        }
                        break;
                    case Command.MOV:
                        Error.WriteLine("Command : MOV");
                        break;
                    case Command.STP:
                        Error.WriteLine("Command : STP");
                        break;
                    case Command.RST:
                        Error.WriteLine("Command : RST");
                        break;
                    default:
                        return;
                }
            }
        }

        static bool turnover, repeat;
        static CartPoleFric cp;
        static Vector x;
        const double power = 5.0;

        private class AsyncStateObject
        {
            public Socket Socket;
            public byte[] ReceiveBuffer;

            public AsyncStateObject(Socket soc)
            {
                Socket = soc;
                ReceiveBuffer = new byte[256];
            }
        }

        private static AsyncStateObject StartReceive(Socket soc)
        {
            var so = new AsyncStateObject(soc);
            soc.BeginReceive(so.ReceiveBuffer,
                0,
                so.ReceiveBuffer.Length,
                SocketFlags.None,
                new AsyncCallback(ReceiveDataCallback),
                so);
            return so;
        }

        private static void ReceiveDataCallback(IAsyncResult ar)
        {
            var so = (AsyncStateObject)ar.AsyncState;

            var len = 0;
            try {
                len = so.Socket.EndReceive(ar);
            }
            catch (ObjectDisposedException) {
                Error.WriteLine("Closed.");
                return;
            }

            if (len <= 0) {
                Error.WriteLine("Disconnected.");
                so.Socket.Close();
                return;
            }

            if (so.Socket.Available == 0) {
                var p = new Packet() {
                    command = BitConverter.ToInt64(so.ReceiveBuffer, 0),
                    data = new double[4]
                };

                switch ((Command)p.command) {
                    case Command.GET: {
                            Error.WriteLine("Command : GET");
                            if (!turnover) {
                                Buffer.BlockCopy(BitConverter.GetBytes(p.command), 0, so.ReceiveBuffer, 0, 8);
                                for (int i = 0; i < p.data.Length; i++)
                                    Buffer.BlockCopy(BitConverter.GetBytes(x[i]), 0, so.ReceiveBuffer, 8 * (i + 1), 8);
                                if (so.Socket.Send(so.ReceiveBuffer, 40, SocketFlags.None) <= 0)
                                    Error.WriteLine("Sending Error.");
                            } else {
                                Buffer.BlockCopy(BitConverter.GetBytes((long)Command.RST), 0, so.ReceiveBuffer, 0, 8);
                                for (int i = 0; i < p.data.Length; i++)
                                    Buffer.BlockCopy(BitConverter.GetBytes(x[i]), 0, so.ReceiveBuffer, 8 * (i + 1), 8);
                                if (so.Socket.Send(so.ReceiveBuffer, 40, SocketFlags.None) <= 0)
                                    Error.WriteLine("Sending Error.");
                            }
                        }
                        break;
                    case Command.MOV: {
                            if (!turnover) {
                                Error.WriteLine("Command : MOV");
                                cp.F = (BitConverter.ToDouble(so.ReceiveBuffer, 8) < 0) ? power : -power;
                            } else {
                                repeat = false;
                                turnover = false;
                            }
                        }
                        break;
                    case Command.STP: {
                            if (!turnover) {
                                Error.WriteLine("Command : STP");
                                cp.F = 0.0;
                            } else {
                                repeat = false;
                                turnover = false;
                            }
                        }
                        break;
                    case Command.RST: {
                            repeat = true;
                            turnover = false;
                        }
                        break;
                    default:
                        return;
                }
            }

            so.Socket.BeginReceive(so.ReceiveBuffer,
                0,
                so.ReceiveBuffer.Length,
                SocketFlags.None,
                new AsyncCallback(ReceiveDataCallback),
                so);
        }

        public static void Main()
        {
            var gp = new Gnuplot(@"C:\Program Files\gnuplot\bin\gnuplot.exe");
            cp = new CartPoleFric();

            gp.Start();
            //gp.StandardInput.WriteLine("set size square");
            gp.SetXRange(-0.5, 0.5);
            gp.SetXLabel();
            gp.SetYRange(0.0, 0.1);

            var sock = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            var point = new IPEndPoint(IPAddress.Any, port);
            sock.Bind(point);
            sock.Listen(1);
            Error.WriteLine("Listening");
            var client = sock.Accept();
            //Echo(client);
            StartReceive(client);
        
            ODESolver.dt = 1e-3;
            double deg_80 = 4.0 * PI / 9.0;
            var t = 0;
            repeat = true;
            while (repeat) {

                t += 1;
                cp.Init();
                x = cp.x0;
                //var step = 0;
                while (-deg_80 < x[2] && x[2] < deg_80) {
                    //cp.F *= (++step % 1000 == 0) ? -1 : 1;
                    x = ODESolver.rk4Step(cp, 0.0, x);
                    var pos = new Vector2(x[0], 0.0);
                    var p_g = new Vector2(0.0, CartPole.l);
                    p_g *= Matrix.RotationMatrix2D(x[2]);
                    p_g += pos;
                    gp.SetXLabelName("trial = " + t);
                    gp.PlotLines(pos, p_g);
                }

                turnover = true;
                while (turnover) ;

                WriteLine((repeat) ? "Retry." : "Finiish.");
            }

            client.Close();
            sock.Close();
            gp.Close();
        }
    }
}
