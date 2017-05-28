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
    }

    struct Packet {
        Command command;
        double[] data;
    }

	class Program {
        const short port = 17997;

		public static void Main() {
            var gp = new Gnuplot(@"G:\Applications\gnuplot\bin\gnuplot.exe");
            var cp = new CartPole();

            gp.Start();
            gp.StandardInput.WriteLine("set size square");
            gp.SetXRange(-0.1, 0.1);
            gp.SetYRange(0.0, 0.1);

            var listener = new TcpListener(IPAddress.Any, port);
            listener.Start();
            

            ODESolver.dt = 1e-3;
            double deg_80 = 4.0 * PI / 9.0;
            var repeat = true;
            while(repeat) {
                cp.Init();
                cp.F = -1.0;
                Vector x = cp.x0;
                double t = cp.ts;
                while(-deg_80 < x[2] && x[2] < deg_80 && t < 1.0) {
                    x = ODESolver.rk4Step(cp, t, x);
                    var pos = new Vector2(x[0], 0.0);
                    var p_g = new Vector2(0.0, CartPole.l);
                    p_g *= Matrix.RotationMatrix2D(x[2]);
                    p_g += pos;
                    gp.PlotLines(pos, p_g);
                    t += ODESolver.dt;
                }
                repeat = false;
                WriteLine("Repeating.");
            }

            listener.Stop();
            gp.Close();
		}
	}
}
