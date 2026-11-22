namespace PrintGuard;

// ============================================================
//  参考过程模型:由原始切片指令推导出的"每层应有的物理过程"
// ============================================================

public static class Rng
{
    public static double Gaussian(Random r, double mu = 0.0, double sigma = 1.0)
    {
        double u1 = 1.0 - r.NextDouble();
        double u2 = r.NextDouble();
        return mu + sigma * Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2);
    }
}

/// <summary>声学模型:喷头振动幅度与挤出速率近似成正比;空驶时只有基底噪声。</summary>
public static class Acoustic
{
    public static double Amp(Move m) =>
        m.Kind == MoveKind.Extrude && m.Duration > 1e-9
            ? 0.25 + 0.9 * (m.E / m.Duration)
            : 0.15;
}

/// <summary>单层的期望物理特征(由参考 G-code 离线计算)。</summary>
public readonly struct LayerFeatures
{
    public readonly double ExtrudeLen;  // 该层应挤出的丝料总长 mm
    public readonly double PrintTime;   // 该层应耗时 s
    public readonly double DirCos2;     // 振幅加权的运动方向 cos^2(填充方向指纹)
    public readonly double Temp;        // 该层喷嘴目标温度
    public readonly double Rms;         // 该层声学幅度均值
    public LayerFeatures(double e, double t, double d, double temp, double rms)
        => (ExtrudeLen, PrintTime, DirCos2, Temp, Rms) = (e, t, d, temp, rms);
}

public sealed class ReferenceModel
{
    public required LayerFeatures[] Layers;

    public static ReferenceModel FromProgram(PrintProgram p)
    {
        var arr = new LayerFeatures[p.Layers.Count];
        for (int i = 0; i < p.Layers.Count; i++)
        {
            var l = p.Layers[i];
            double e = 0, time = 0, wCos = 0, wSum = 0, ampSum = 0;
            foreach (var m in l.Moves)
            {
                double dur = m.Duration;
                double amp = Acoustic.Amp(m);
                e += m.E;
                time += dur;
                wCos += amp * dur * m.Cos2;
                wSum += amp * dur;
                ampSum += amp * dur;
            }
            arr[i] = new LayerFeatures(
                e, time,
                wSum > 1e-9 ? wCos / wSum : 0.5,
                l.NozzleTemp,
                time > 1e-9 ? ampSum / time : 0.15);
        }
        return new ReferenceModel { Layers = arr };
    }
}

// ============================================================
//  传感器仿真:把(可能被篡改的)G-code 回放成三路传感信号
//    - 步进脉冲(E 轴 / XY 轴)→ 实际挤出量、实际运动
//    - 喷嘴温度(一阶热惯性 + 测量噪声 + 环境慢漂)
//    - 打印声音(幅度 ∝ 挤出速率,方向谱 ∝ 运动方向)
//  每路信号带各自的传输延迟,模拟真实采集链。
// ============================================================

public readonly struct SensorSample
{
    public readonly double Dt, DE, DXY, Amp, Cos2, Temp;
    public readonly bool LayerMark;
    public SensorSample(double dt, double dE, double dXY, double amp, double cos2, double temp, bool mark)
        => (Dt, DE, DXY, Amp, Cos2, Temp, LayerMark) = (dt, dE, dXY, amp, cos2, temp, mark);
}

public sealed class SimConfig
{
    public double Dt = 0.01;            // 采样周期 100 Hz
    public double TempTau = 6.0;        // 喷嘴热惯性时间常数 s
    public double TempNoise = 0.25;     // 温度测量噪声 °C
    public double TempWander = 0.01;    // 环境慢漂强度 °C/√s(OU 均值回归, 相关时间 ~5 min)
    public double TempWanderTau = 300.0; // 慢漂相关时间 s(有界漂移, 模拟车间空调环境)
    public double FlowJitter = 0.02;    // 单段挤出流量抖动(乘性)
    public double LayerFlowDrift = 0.015; // 层间流量慢漂(丝径不均等)
    public double FeedJitter = 0.03;    // 单段速度抖动(加减速等,影响层耗时)
    public double LayerFeedDrift = 0.01;  // 层间速度慢漂
    public double AmpNoise = 0.08;      // 声学测量噪声(乘性)
    public double TempDelay = 0.8;      // 温度通道延迟 s
    public double AcousticDelay = 0.05; // 声学通道延迟 s
    public int Seed = 1;
}

public sealed class SensorSimulator
{
    private readonly SimConfig _cfg;
    public SensorSimulator(SimConfig cfg) => _cfg = cfg;

    /// <summary>回放整个程序,把样本流推给 sink(监测器或通道)。</summary>
    public void Run(PrintProgram prog, Action<SensorSample> sink)
    {
        var rnd = new Random(_cfg.Seed);
        double clock = 0;
        double temp = prog.Layers.Count > 0 ? prog.Layers[0].NozzleTemp : 200;
        double wander = 0;

        // 传输延迟线(先进先出)
        var tempQ = new Queue<(double t, double v)>();
        var acQ = new Queue<(double t, double amp, double cos2)>();
        double tempOut = temp, ampOut = 0.15, cos2Out = 0.5;

        foreach (var layer in prog.Layers)
        {
            sink(new SensorSample(0, 0, 0, 0, 0, 0, true)); // Z 轴层标记(来自 Z 步进脉冲)
            double layerFlow = 1.0 + Rng.Gaussian(rnd, 0, _cfg.LayerFlowDrift);
            double layerFeed = 1.0 + Rng.Gaussian(rnd, 0, _cfg.LayerFeedDrift);

            foreach (var move in layer.Moves)
            {
                double dur = move.Duration * layerFeed * (1.0 + Rng.Gaussian(rnd, 0, _cfg.FeedJitter));
                if (dur <= 1e-9) continue;
                int n = Math.Max(1, (int)Math.Round(dur / _cfg.Dt));
                double dt = dur / n;
                double flow = layerFlow * (1.0 + Rng.Gaussian(rnd, 0, _cfg.FlowJitter));
                double amp0 = Acoustic.Amp(move);
                bool extrude = move.Kind == MoveKind.Extrude;

                for (int i = 0; i < n; i++)
                {
                    clock += dt;
                    // --- 喷嘴温度物理过程 ---
                    temp += (layer.NozzleTemp - temp) * dt / _cfg.TempTau;
                    // Ornstein-Uhlenbeck 慢漂:有界,符合车间恒温环境
                    wander += -wander * dt / _cfg.TempWanderTau + Rng.Gaussian(rnd, 0, _cfg.TempWander * Math.Sqrt(dt));
                    tempQ.Enqueue((clock, temp + wander + Rng.Gaussian(rnd, 0, _cfg.TempNoise)));
                    // --- 声学 ---
                    double amp = amp0 * (extrude ? flow : 1.0) * (1.0 + Rng.Gaussian(rnd, 0, _cfg.AmpNoise));
                    acQ.Enqueue((clock, amp, move.Cos2));
                    // --- 按各自延迟取队首作为"此刻读到"的传感值 ---
                    while (tempQ.Count > 0 && tempQ.Peek().t <= clock - _cfg.TempDelay) tempOut = tempQ.Dequeue().v;
                    while (acQ.Count > 0 && acQ.Peek().t <= clock - _cfg.AcousticDelay)
                    { var a = acQ.Dequeue(); ampOut = a.amp; cos2Out = a.cos2; }

                    sink(new SensorSample(
                        dt,
                        extrude ? move.E / n * flow : 0.0,
                        move.Len / n,
                        ampOut, cos2Out, tempOut, false));
                }
            }
        }
    }
}

// ============================================================
//  在线监测器:逐层特征提取 → 标准化残差(z) → 每特征 CUSUM
//  任一特征的累积偏移量超过校准阈值即触发停机。
//  全部计算 O(1)/样本,可在打印同时实时运行。
// ============================================================

public enum Feat { ExtrudeLen = 0, PrintTime = 1, Direction = 2, Temp = 3, AcousticRms = 4 }

public sealed class Calib
{
    public double[] Std = new double[5];   // 各特征正常波动 σ(由干净打印标定)
    public double[] K = { 0.5, 0.5, 1.0, 0.5, 1.0 }; // CUSUM 允许量
    public double[] H = new double[5];     // 各特征停机阈值(由干净打印标定)
    // 篡改方向先验:挤出量/耗时/温度只会被改小,方向与声学可能双向
    public static readonly bool[] OneSidedDown = { true, true, false, true, false };
}

public sealed class Monitor
{
    private readonly ReferenceModel _ref;
    private readonly Calib _cal;

    public bool Stopped { get; private set; }
    public int AlarmLayer { get; private set; } = -1;
    public Feat AlarmFeature { get; private set; }
    public readonly double[] MaxCusum = new double[5]; // 本次打印中各特征 CUSUM 峰值(标定用)
    public List<double[]>? RawLog;                     // 标定阶段:记录 [obs,exp] 原始特征

    public event Action<int, Feat, double>? Alarm;      // (层号, 触发特征, 统计量)
    public event Action<int, double[]>? LayerDone;      // (层号, 各特征 z 分数)

    private double _e, _time, _bA, _bB, _tSum, _aSum;
    private int _n, _done;
    private readonly double[] _s = new double[5];

    public Monitor(ReferenceModel refModel, Calib cal) { _ref = refModel; _cal = cal; }

    public void Push(SensorSample s) => Push(s.Dt, s.DE, s.DXY, s.Amp, s.Cos2, s.Temp, s.LayerMark);

    public void Push(double dt, double dE, double dXY, double amp, double cos2, double temp, bool layerMark)
    {
        if (Stopped) return;
        if (layerMark)
        {
            if (_n > 0) FinalizeLayer();
            return;
        }
        _time += dt; _e += dE;
        _bA += amp * cos2; _bB += amp * (1.0 - cos2);
        _tSum += temp; _aSum += amp; _n++;
    }

    public void EndOfPrint()
    {
        if (!Stopped && _n > 0) FinalizeLayer();
    }

    private void FinalizeLayer()
    {
        int li = _done++;
        if (li < _ref.Layers.Length)
        {
            var exp = _ref.Layers[li];
            double band = _bA + _bB;
            Span<double> obs = stackalloc double[]
            {
                _e, _time,
                band > 1e-9 ? _bA / band : exp.DirCos2,
                _tSum / _n, _aSum / _n,
            };
            Span<double> expA = stackalloc double[]
                { exp.ExtrudeLen, exp.PrintTime, exp.DirCos2, exp.Temp, exp.Rms };

            if (RawLog != null)
            {
                var row = new double[10];
                for (int f = 0; f < 5; f++) { row[f] = obs[f]; row[5 + f] = expA[f]; }
                RawLog.Add(row);
            }
            else
            {
                var z = new double[5];
                for (int f = 0; f < 5; f++)
                {
                    z[f] = (obs[f] - expA[f]) / _cal.Std[f];
                    double inc = Calib.OneSidedDown[f] ? -z[f] : Math.Abs(z[f]);
                    _s[f] = Math.Max(0.0, _s[f] + inc - _cal.K[f]);
                    if (_s[f] > MaxCusum[f]) MaxCusum[f] = _s[f];
                }
                LayerDone?.Invoke(li, z);
                for (int f = 0; f < 5; f++)
                {
                    if (_s[f] > _cal.H[f])
                    {
                        Stopped = true;
                        AlarmLayer = li;
                        AlarmFeature = (Feat)f;
                        Alarm?.Invoke(li, (Feat)f, _s[f]);
                        break;
                    }
                }
            }
        }
        _e = _time = _bA = _bB = _tSum = _aSum = 0;
        _n = 0;
    }
}
