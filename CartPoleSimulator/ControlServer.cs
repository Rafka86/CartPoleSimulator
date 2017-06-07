using System;
using System.Net;
using System.Net.Sockets;
using Rafka.MathLib.Real;
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
		public Command command;
		public double[] data;
	}

	public class ControlServer {
		private bool cerror, reset;
		public bool Repeat { get; private set; }
		public bool TurnOver { get; private set; }

		private const ushort port = 50000;
		Socket server, client;

		CartPole cp;
		public Vector x { get; private set; }
		int count_F;
		const int size = 1024;
		byte[] recvBuf, sendBuf;

		public ControlServer(CartPole cp) {
			this.cp = cp;
			x = cp.x0;
			count_F = 0;
			Repeat = true;
			cerror = false;
			reset = false;

			server = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
			server.Bind(new IPEndPoint(IPAddress.Any, port));
			server.Listen(10);
			Error.WriteLine("Listening");
			client = null;

			recvBuf = new byte[size];
			sendBuf = new byte[size];
		}

		public void WaitClient() {
			if (client != null) throw new ArgumentException("Already connected client.");
			client = server.Accept();
			Error.WriteLine("Accepted.");
		}

		public void Disconnect() {
			if (client == null) throw new ArgumentException("Connection has not been established.");
			client.Close();
			Error.WriteLine("Disconnected.");
			client = null;
		}

		public void Close() {
			if (client != null) Disconnect();
			server.Close();
		}

		private void BufferClear() {
			while (client.Available != 0) client.Receive(recvBuf, size, SocketFlags.None);
		}

		public void Start() {
			if (client == null) throw new ArgumentException("Connection has not been established.");
			BufferClear();
			client.BeginReceive(recvBuf, 0, size, SocketFlags.None, new AsyncCallback(ReceiveCallBack), recvBuf);
			Error.WriteLine("Start Receiving.");
		}

		private void ReceiveCallBack(IAsyncResult ar) {
			cerror = false;

			int len;
			try {
				len = client.EndReceive(ar);
			}
			catch (Exception e) {
				Error.WriteLine(e.ToString());
				Repeat = false;
				cerror = true;
				return;
			}

			if (len <= 0) {
				Disconnect();
				Repeat = false;
				return;
			}

			Procedure();
			client.BeginReceive(recvBuf, 0, size, SocketFlags.None, new AsyncCallback(ReceiveCallBack), recvBuf);
			Error.WriteLine("Start Receiving.");
		}

		private void Procedure() {
			if (client.Available == 0) {
				var p = new Packet() {
					command = (Command)BitConverter.ToInt64(recvBuf, 0),
					data = new double[4]
				};

				switch (p.command) {
					case Command.GET: {
							Error.WriteLine("Command : GET");

							if (!TurnOver) {
								Buffer.BlockCopy(BitConverter.GetBytes((long)p.command), 0, sendBuf, 0, 8);
								for (int i = 0; i < p.data.Length; i++)
									Buffer.BlockCopy(BitConverter.GetBytes(-x[i]), 0, sendBuf, 8 * (i + 1), 8);
								client.BeginSend(sendBuf, 0, 40, SocketFlags.None, new AsyncCallback(SendCallBack), sendBuf);
							} else {
								Buffer.BlockCopy(BitConverter.GetBytes((long)Command.RST), 0, sendBuf, 0, 8);
								for (int i = 0; i < p.data.Length; i++)
									Buffer.BlockCopy(BitConverter.GetBytes(-x[i]), 0, sendBuf, 8 * (i + 1), 8);
								client.BeginSend(sendBuf, 0, 40, SocketFlags.None, new AsyncCallback(SendCallBack), sendBuf);
							}
						}
						break;
					case Command.MOV: {
							Error.WriteLine("Command : MOV");

							var pow = BitConverter.ToDouble(recvBuf, 8);
							cp.F = -pow;
							count_F = 0;
						}
						break;
					case Command.RST: {
							Error.WriteLine("Command : RST");

							if (!TurnOver) reset = true;
							else {
								Repeat = true;
								TurnOver = false;
								return;
							}
						}
						break;
					case Command.STP: {
							Error.WriteLine("Command : STP");

							if (!TurnOver) cp.F = 0.0;
							else {
								Repeat = false;
								TurnOver = false;
								return;
							}
						}
						break;
					default: {
							Error.WriteLine("Undefined Command.");

							cerror = true;
						}
						return;
				}
			} else Error.WriteLine("Data left.");
		}

		private void SendCallBack(IAsyncResult ar) {
			var len = 0;
			try {
				len = client.EndSend(ar);
			}
			catch (Exception e) {
				Error.WriteLine(e.ToString());
				Repeat = false;
				cerror = true;
				return;
			}
		}

		public bool ResetRequest {
			get { return cerror | reset; }
		}

		public void Reseted() {
			reset = false;
		}

		const double deg_45 = PI / 4.0;
		private bool CheckState() {
			return -deg_45 < x[2] && x[2] < deg_45 && -1.0 < x[0] && x[0] < 1.0;
		}

		public void UpdateCartInfo(Vector x) {
			this.x = x;
			if (count_F++ == 10000) cp.F = 0.0;
			if (!ResetRequest) TurnOver = !CheckState();
		}
	}
}