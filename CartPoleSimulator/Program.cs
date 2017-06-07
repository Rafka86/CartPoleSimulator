using System;
using System.Net.Sockets;
using Rafka.MathLib.Utils;
using Rafka.MathLib.Real;
using Rafka.MathLib.Real.Numerics;
using static System.Console;

namespace CartPoleSimulator {
	class Program {
		private static void Echo(Socket client) {
			var buf = new byte[256];

			while (client.Receive(buf) > 0) {
				var packet = new Packet() {
					command = (Command)BitConverter.ToInt64(buf, 0),
					data = new double[4]
				};
				for (int i = 0; i < packet.data.Length; i++)
					packet.data[i] = BitConverter.ToDouble(buf, 8 * (i + 1));
				switch ((Command)packet.command) {
					case Command.GET: {
							Error.WriteLine("Command : GET");
							packet.data[0] = packet.data[1] = packet.data[2] = packet.data[3] = 1.0;
							Buffer.BlockCopy(BitConverter.GetBytes((long)packet.command), 0, buf, 0, 8);
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

		public static void Main() {
			var gp = new Gnuplot(@"G:\Applications\gnuplot\bin\gnuplot.exe");
			var cp = new CartPole();
			var cs = new ControlServer(cp);

			gp.Start();
			gp.StandardInput.WriteLine("set size square");
			gp.SetXRange(-0.1, 0.1);
			gp.SetXLabel();
			gp.SetYRange(0.0, 0.2);

			cs.WaitClient();

			ODESolver.dt = 1e-5;
			var t = 0;
			while (cs.Repeat) {
				t += 1;
				gp.SetYLabelName("trial = " + t);
				cs.Reseted();

				cp.Init();
				cs.UpdateCartInfo(cp.x0);
				var center = 0.0;

				var count = 0;
				cs.Start();
				while (!cs.TurnOver && !cs.ResetRequest) {
					cs.UpdateCartInfo(ODESolver.rk4Step(cp, 0.0, cs.x));

					var pos = new Vector2(cs.x[0], 0.0);
					var p_g = new Vector2(0.0, CartPoleFric.l);
					p_g *= Matrix.RotationMatrix2D(cs.x[2]);
					p_g += pos;
					if (count++ % 10 == 0) {
						gp.SetXLabelName("time = " + (count * ODESolver.dt) + " s");
						if (count % 1000 == 0) {
							center += 0.5 * (cs.x[0] - center);
							gp.SetXRange(center - 0.1, center + 0.1);
						}
						gp.PlotLines(pos, p_g);
					}
				}

				while (cs.TurnOver) ;

				WriteLine((cs.Repeat) ? "Retry." : "Finish.");
			}

			cs.Close();
			gp.Close();
		}
	}
}