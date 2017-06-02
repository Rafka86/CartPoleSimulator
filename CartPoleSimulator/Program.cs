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
        const int port = 50000;

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
        const double power = 5000.0;

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
            cerror = false;
            soc.BeginReceive(so.ReceiveBuffer,
                    0,
                    so.ReceiveBuffer.Length,
                    SocketFlags.None,
                    new AsyncCallback(ReceiveDataCallback),
                    so);
            return so;
        }

        static bool cerror;
        private static void ReceiveDataCallback(IAsyncResult ar)
        {
            var so = (AsyncStateObject)ar.AsyncState;

            var len = 0;
            try {
                len = so.Socket.EndReceive(ar);
            }
            catch (ObjectDisposedException) {
                Error.WriteLine("Closed.");
                cerror = true;
                repeat = false;
                return;
            }

            if (len <= 0) {
                Error.WriteLine("Disconnected.");
                so.Socket.Close();
                cerror = true;
                repeat = false;
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
                                    Buffer.BlockCopy(BitConverter.GetBytes(-x[i]), 0, so.ReceiveBuffer, 8 * (i + 1), 8);
                                if (so.Socket.Send(so.ReceiveBuffer, 40, SocketFlags.None) <= 0)
                                    Error.WriteLine("Sending Error.");
                            } else {
                                Buffer.BlockCopy(BitConverter.GetBytes((long)Command.RST), 0, so.ReceiveBuffer, 0, 8);
                                for (int i = 0; i < p.data.Length; i++)
                                    Buffer.BlockCopy(BitConverter.GetBytes(-x[i]), 0, so.ReceiveBuffer, 8 * (i + 1), 8);
                                if (so.Socket.Send(so.ReceiveBuffer, 40, SocketFlags.None) <= 0)
                                    Error.WriteLine("Sending Error.");
                            }
                        }
                        break;
                    case Command.MOV: {
                            Error.WriteLine("Command : MOV");
                            var pow = BitConverter.ToDouble(so.ReceiveBuffer, 8);
                            cp.F = -pow;
                            Error.WriteLine("Move power : " + cp.F);
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
                    default: {
                            repeat = true;
                            turnover = false;
                        }
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
            gp.StandardInput.WriteLine("set size square");
            gp.SetXRange(-0.1, 0.1);
            gp.SetXLabel();
            gp.SetYRange(0.0, 0.2);

            var sock = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            var point = new IPEndPoint(IPAddress.Any, port);
            sock.Bind(point);
            sock.Listen(1);
            Error.WriteLine("Listening");
            var client = sock.Accept();
            Error.WriteLine("Accepted.");
            StartReceive(client);

            ODESolver.dt = 1e-4;
            double deg_45 = PI / 4.0;
            var t = 0;
            repeat = true;
            while (repeat) {

                t += 1;
                cp.Init();
                x = cp.x0;
                var count = 0;
                while (-deg_45 < x[2] && x[2] < deg_45 && -1.0 < x[0] && x[0] < 1.0) {
                    x = ODESolver.rk4Step(cp, 0.0, x);
                    cp.UpdateFriction(x);
                    Error.WriteLine(x);
                    var pos = new Vector2(x[0], 0.0);
                    var p_g = new Vector2(0.0, CartPoleFric.l);
                    p_g *= Matrix.RotationMatrix2D(x[2]);
                    p_g += pos;
                    gp.SetXLabelName("trial = " + t);
                    if (count++ % 10 == 0) {
                        cp.F = 0.0;
                        gp.PlotLines(pos, p_g);
                    }
                }

                turnover = true;
                while (turnover && !cerror) ;

                WriteLine((repeat) ? "Retry." : "Finish.");
            }

            client.Close();
            sock.Close();
            gp.Close();
        }
    }
}