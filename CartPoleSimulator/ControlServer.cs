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
		public int time_stamp;
		public double[] data;
	}

	public class ControlServer {
		private bool cerror, reset;
		public bool Repeat { get; private set; }
		public bool TurnOver { get; private set; }

		private const ushort port = 50000;
		Socket server, client;

		CartPole cp;
		private Vector x;
		int count_F, time_stamp;
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
			cerror = true;
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

		public void SyncStart() {
			client.Receive(recvBuf, size, SocketFlags.None);
			SyncProcedure();
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
				if (client != null) Disconnect();
				Repeat = false;
				return;
			}

			Procedure();
			client.BeginReceive(recvBuf, 0, size, SocketFlags.None, new AsyncCallback(ReceiveCallBack), recvBuf);
			Error.WriteLine("Start Receiving.");
		}

		private void SyncProcedure() {
			var p = new Packet() {
				command = (Command)BitConverter.ToInt32(recvBuf, 0),
				time_stamp = BitConverter.ToInt32(recvBuf, 4),
				data = new double[4]
			};

			switch (p.command) {
				case Command.GET: {
						Error.WriteLine("Command : GET");

						if (!TurnOver) {
							Buffer.BlockCopy(BitConverter.GetBytes((int)p.command), 0, sendBuf, 0, 4);
							Buffer.BlockCopy(BitConverter.GetBytes(time_stamp), 0, sendBuf, 4, 4);
							for (int i = 0; i < p.data.Length; i++)
								Buffer.BlockCopy(BitConverter.GetBytes(-x[i]), 0, sendBuf, 8 * (i + 1), 8);
							client.Send(sendBuf, 40, SocketFlags.None);
						} else {
							Buffer.BlockCopy(BitConverter.GetBytes((int)Command.RST), 0, sendBuf, 0, 4);
							Buffer.BlockCopy(BitConverter.GetBytes(time_stamp), 0, sendBuf, 4, 4);
							for (int i = 0; i < p.data.Length; i++)
								Buffer.BlockCopy(BitConverter.GetBytes(-x[i]), 0, sendBuf, 8 * (i + 1), 8);
							client.Send(sendBuf, 40, SocketFlags.None);
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
							TurnOver = true;
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
		}

		private void Procedure() {
			if (client.Available == 0) {
				var p = new Packet() {
					command = (Command)BitConverter.ToInt32(recvBuf, 0),
					time_stamp = BitConverter.ToInt32(recvBuf, 4),
					data = new double[4]
				};

				switch (p.command) {
					case Command.GET: {
							Error.WriteLine("Command : GET");

							if (!TurnOver) {
								Buffer.BlockCopy(BitConverter.GetBytes((int)p.command), 0, sendBuf, 0, 4);
								Buffer.BlockCopy(BitConverter.GetBytes(time_stamp), 0, sendBuf, 4, 4);
								for (int i = 0; i < p.data.Length; i++)
									Buffer.BlockCopy(BitConverter.GetBytes(-x[i]), 0, sendBuf, 8 * (i + 1), 8);
								client.BeginSend(sendBuf, 0, 40, SocketFlags.None, new AsyncCallback(SendCallBack), sendBuf);
							} else {
								Buffer.BlockCopy(BitConverter.GetBytes((int)Command.RST), 0, sendBuf, 0, 4);
								Buffer.BlockCopy(BitConverter.GetBytes(time_stamp), 0, sendBuf, 4, 4);
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

		const double lim_deg = 45.0 * PI / 180.0;
		const double lim_dst = 1000.0;						//mm
		private bool CheckState() {
			return -lim_deg < x[2] && x[2] < lim_deg && -lim_dst < x[0] && x[0] < lim_dst;
		}

		public void CartPoleUpdateF() {
			if (count_F++ == 100) cp.F = 0.0;
		}

		public void UpdateCartInfo(int time, Vector x) {
			time_stamp = time;
			this.x = x.Clone;
			this.x[0] *= 1000;	//m->mm
			this.x[1] *= 1000;	//m/s->mm/s
			if (!ResetRequest) TurnOver = !CheckState();
		}
	}
}