using System;
using Rafka.MathLib.Real;
using Rafka.MathLib.Real.Numerics;

namespace CartPoleSimulator {
    public class CartPoleFric : ODE {
        private const double mc = 2.0;
        private const double mp = 150.0 + 54.0;
        private const double g = 9.80665;
        public const double l = 0.0745;
		private const double muc = 0.53;
		private const double mup = 0.4;

        private const double div43 = 4.0 / 3.0;
        private const double mc_mp = mc + mp;
        private const double mpl = mp * l;

		public double F { get; set; } = 0.0;

		public CartPoleFric() {
			Init();
		}

		public void Init() {
			x0 = new Vector(0.0, 0.0, 0.0, 0.0, (mc + mp) * g);
            ts = 0.0;
            te = 1.0;
            F = 0.0;
		}

		private double theta_2nd(double nc, double v, double theta, double theta_v) {
			double sin = Math.Sin(theta);
			double cos = Math.Cos(theta);

			double term1 = g * sin;
			double term2_1 = (-F - mpl * theta_v * theta_v * (sin + muc * Math.Sign(nc * v) * cos)) / mc_mp;
			double term2_2 = muc * g * Math.Sign(nc * v);
			double term2 = cos * (term2_1 + term2_1);
			double term3 = (mup * theta_v) / mpl;
			double numer = term1 + term2 - term3;

			double term4 = div43 - ((mp * cos) * (cos - muc * Math.Sign(nc * v))) / mc_mp;
			double denom = l * term4;

			return numer / denom;
		}

		private double x_2nd(double nc, double v, double theta, double theta_v, double theta_2) {
			double sin = Math.Sin(theta);
			double cos = Math.Cos(theta);

            double numer = F + mpl * (theta_v * theta_v * sin - theta_2 * cos) - muc * nc * Math.Sin(nc * v);

			return numer / mc_mp;
		}

		public override Vector Feval(double t, Vector x) {
			double nc   = x[4];
			double p    = x[0];
			double v    = x[1];
			double th   = x[2];
			double th_v = x[3];

			Vector res = new Vector(x.Length);
			res[3] = theta_2nd(nc, v, th, th_v);
            res[2] = th_v;
            res[1] = x_2nd(nc, v, th, th_v, res[3]);
            res[0] = v;
            res[4] = mc_mp * g - mpl * (res[3] * Math.Sin(th) + th_v * th_v * Math.Cos(th));

			return res;
		}
	}
}