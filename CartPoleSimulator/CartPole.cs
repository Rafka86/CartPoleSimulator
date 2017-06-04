using System;
using Rafka.MathLib.Real;
using Rafka.MathLib.Real.Numerics;

namespace CartPoleSimulator {
	public class CartPole : ODE {
		private const double mc = 0.002;
		private const double mp = 0.150 + 0.054;
		private const double g = 9.80665;
		public const double l = 0.0745;

		private const double div43 = 4.0 / 3.0;
		private const double mc_mp = mc + mp;
		private const double mpl = mp * l;

		public double F { get; set; } = 0.0;

		public CartPole() {
			Init();
		}

		public void Init() {
			x0 = new Vector(0.0, 0.0, 0.0, 0.0);
			ts = 0.0;
			te = 1.0;
			F = 0.0;
		}

		private double theta_2nd(double v, double theta, double theta_v) {
			double sin = Math.Sin(theta);
			double cos = Math.Cos(theta);

			double term1 = g * sin;
			double term2 = cos * (-F - mpl * theta_v * theta_v * sin) / mc_mp;
			double numer = term1 + term2;

			double term4 = div43 - mp * cos * cos / mc_mp;
			double denom = l * term4;

			return numer / denom;
		}

		private double x_2nd(double v, double theta, double theta_v, double theta_2) {
			double sin = Math.Sin(theta);
			double cos = Math.Cos(theta);

			double numer = F + mpl * (theta_v * theta_v * sin - theta_2 * cos);

			return numer / mc_mp;
		}

		public override Vector Feval(double t, Vector x) {
			double p = x[0];
			double v = x[1];
			double th = x[2];
			double th_v = x[3];

			Vector res = new Vector(x.Length);
			res[3] = theta_2nd(v, th, th_v);
			res[2] = th_v;
			res[1] = x_2nd(v, th, th_v, res[3]);
			res[0] = v;

			return res;
		}
	}
}
