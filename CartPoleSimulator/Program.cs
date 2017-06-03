using System;
using System.Net;
using System.Net.Sockets;
using Rafka.MathLib.Utils;
using Rafka.MathLib.Real;
using Rafka.MathLib.Real.Numerics;
using static System.Console;
using static System.Math;

namespace CartPoleSimulator {
	enum Command {
		GET,
		MOV,
		STP,
		RST,
	}

	struct Packet {
		public long command;
		public double[] data;
	}

	class Program {
		const int port = 50000;

		private static void Echo(Socket client) {
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

		static bool turnover, repeat, cerror;
		static CartPole cp;
		static Vector x;
		static int count_f;

		private class AsyncStateObject {
			public Socket Socket;
			public byte[] ReceiveBuffer;

			public AsyncStateObject(Socket soc) {
				Socket = soc;
				ReceiveBuffer = new byte[256];
			}
		}

		private static AsyncStateObject StartReceive(Socket soc) {
			var so = new AsyncStateObject(soc);
			soc.BeginReceive(so.ReceiveBuffer,
							0,
							so.ReceiveBuffer.Length,
							SocketFlags.None,
							new AsyncCallback(ReceiveDataCallback),
							so);
			return so;
		}

		private static void ReceiveDataCallback(IAsyncResult ar) {
			var so = (AsyncStateObject)ar.AsyncState;

			cerror = false;
			var len = 0;
			try {
				len = so.Socket.EndReceive(ar);
			}
			catch (ObjectDisposedException) {
				Error.WriteLine("Closed.");
				cerror = true;
				return;
			}

			if (len <= 0) {
				Error.WriteLine("Disconnected.");
				so.Socket.Close();
				cerror = true;
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
							Error.WriteLine("Command : MOV");
							var pow = BitConverter.ToDouble(so.ReceiveBuffer, 8);
							cp.F = -pow;
							count_f = 0;
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

		static double deg_45 = PI / 4.0;
		private static bool CheckState(Vector x) {
			return -deg_45 < x[2] && x[2] < deg_45 && -1.0 < x[0] && x[0] < 1.0;
		}

		public static void Main() {
			var gp = new Gnuplot(@"C:\Program Files\gnuplot\bin\gnuplot.exe");
			cp = new CartPole();

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
			var t = 0;
			repeat = true;
			while (repeat) {

				t += 1;
				gp.SetXLabelName("trial = " + t);

				cp.Init();
				x = cp.x0;

				var count = 0;
				count_f = 0;
				while (CheckState(x) && !cerror) {
					x = ODESolver.rk4Step(cp, 0.0, x);

					var pos = new Vector2(x[0], 0.0);
					var p_g = new Vector2(0.0, CartPoleFric.l);
					p_g *= Matrix.RotationMatrix2D(x[2]);
					p_g += pos;

					if (count_f++ == 1000) cp.F = 0.0;

					if (count++ % 10 == 0) {
						gp.PlotLines(pos, p_g);
					}
				}

				if (!cerror) turnover = true;
				while (turnover) ;

				WriteLine((repeat) ? "Retry." : "Finish.");
			}

			client.Close();
			sock.Close();
			gp.Close();
		}
	}
}