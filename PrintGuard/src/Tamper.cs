namespace PrintGuard;

// ============================================================
//  篡改指令生成器(红队工具):在保持零件外观与外形尺寸完全
//  不变的前提下,对内部结构做多种隐蔽篡改,用于验证监测器。
//  所有篡改都只作用于内部填充/工艺参数,不动外壳,因此
//  从外部测量(尺寸、重量差异 < 数克)无法发现。
// ============================================================

public static class Tamper
{
    /// <summary>1. 隐藏空洞:在连续若干层内,填充线穿过圆柱形空洞区域的部分
    /// 被剪掉(挤出在空洞处中断),形成内部空腔;外壳不动,外观完全正常。</summary>
    public static PrintProgram VoidPocket(PrintProgram src, int startLayer, int layerCount,
        double cx = 30, double cy = 30, double radius = 8)
    {
        var p = src.Clone();
        for (int li = startLayer; li < Math.Min(startLayer + layerCount, p.Layers.Count); li++)
        {
            var moves = p.Layers[li].Moves;
            for (int i = 0; i < moves.Count; i++)
            {
                var m = moves[i];
                if (m.Kind != MoveKind.Extrude || !m.IsInfill || m.Len < 1e-9) continue;
                double x0 = i > 0 ? moves[i - 1].X : m.X;
                double y0 = i > 0 ? moves[i - 1].Y : m.Y;
                var outside = ClipSegmentOutsideCircle(x0, y0, m.X, m.Y, cx, cy, radius);
                if (outside.Count == 1 && outside[0].t0 < 1e-9 && outside[0].t1 > 1 - 1e-9)
                    continue; // 不经过空洞
                // 用空洞外的子段替换原填充段
                var replacement = new List<Move>();
                foreach (var (t0, t1) in outside)
                {
                    if (t1 - t0 < 1e-6) continue;
                    double frac = t1 - t0;
                    replacement.Add(new Move
                    {
                        Kind = MoveKind.Extrude, Feed = m.Feed, IsInfill = true, Z = m.Z,
                        X = x0 + (m.X - x0) * t1, Y = y0 + (m.Y - y0) * t1,
                        Len = m.Len * frac, E = m.E * frac, Cos2 = m.Cos2,
                    });
                }
                moves.RemoveAt(i);
                moves.InsertRange(i, replacement);
                i += replacement.Count - 1;
            }
        }
        return p;
    }

    /// <summary>线段 P0→P1 落在圆外的参数区间(t∈[0,1])。</summary>
    private static List<(double t0, double t1)> ClipSegmentOutsideCircle(
        double x0, double y0, double x1, double y1, double cx, double cy, double r)
    {
        double dx = x1 - x0, dy = y1 - y0;
        double fx = x0 - cx, fy = y0 - cy;
        double a = dx * dx + dy * dy;
        if (a < 1e-18) return new() { (0, 1) };
        double b = 2 * (fx * dx + fy * dy);
        double c = fx * fx + fy * fy - r * r;
        double disc = b * b - 4 * a * c;
        if (disc <= 0) return new() { (0, 1) }; // 不相交
        double sq = Math.Sqrt(disc);
        double tIn = (-b - sq) / (2 * a), tOut = (-b + sq) / (2 * a);
        var res = new List<(double, double)>();
        if (tIn > 0) res.Add((0, Math.Min(1, tIn)));
        if (tOut < 1) res.Add((Math.Max(0, tOut), 1));
        return res;
    }

    /// <summary>2. 填充方向旋转:把一段层区间的内部填充整体旋转 90°
    /// (挤出总量几乎不变,但承力方向改变,层间剪切强度下降)。</summary>
    public static PrintProgram InfillRotate(PrintProgram src, int startLayer, int endLayer,
        double cx = 30, double cy = 30)
    {
        var p = src.Clone();
        for (int li = startLayer; li <= Math.Min(endLayer, p.Layers.Count - 1); li++)
            foreach (var m in p.Layers[li].Moves)
            {
                if (!m.IsInfill) continue;
                // 绕零件中心旋转 90°:(x,y) -> (cx-(y-cy), cy+(x-cx))
                (m.X, m.Y) = (cx - (m.Y - cy), cy + (m.X - cx));
                m.Cos2 = 1.0 - m.Cos2;
            }
        return p;
    }

    /// <summary>3. 填充密度稀释:从某层起随机删除少量填充线,
    /// 分散到余下所有层,单层挤出量变化被噪声掩盖。</summary>
    public static PrintProgram DensityThin(PrintProgram src, int startLayer, double dropProb, int seed)
    {
        var p = src.Clone();
        var rnd = new Random(seed);
        for (int li = startLayer; li < p.Layers.Count; li++)
            p.Layers[li].Moves.RemoveAll(m =>
                m.Kind == MoveKind.Extrude && m.IsInfill && rnd.NextDouble() < dropProb);
        return p;
    }

    /// <summary>4. 微跳过:每层随机丢掉极少量填充段(比密度稀释更细微),
    /// 模拟"积少成多"型内部疏松。</summary>
    public static PrintProgram MicroSkip(PrintProgram src, int startLayer, double skipProb, int seed)
        => DensityThin(src, startLayer, skipProb, seed);

    /// <summary>5. 温度漂移:从某层起缓慢降低喷嘴温度设定,
    /// 层间结合强度下降,但外观无变化。</summary>
    public static PrintProgram TempDrift(PrintProgram src, int startLayer, int rampLayers, double deltaC)
    {
        var p = src.Clone();
        for (int li = startLayer; li < p.Layers.Count; li++)
        {
            double k = Math.Min(1.0, (double)(li - startLayer + 1) / rampLayers);
            p.Layers[li].NozzleTemp += deltaC * k;
        }
        return p;
    }

    /// <summary>6. 速度覆写:加快填充挤出速度,造成压力性欠挤与结合不良,
    /// 外形尺寸不变。</summary>
    public static PrintProgram FeedOverride(PrintProgram src, int startLayer, double factor)
    {
        var p = src.Clone();
        for (int li = startLayer; li < p.Layers.Count; li++)
            foreach (var m in p.Layers[li].Moves)
                if (m.Kind == MoveKind.Extrude && m.IsInfill)
                    m.Feed *= factor;
        return p;
    }

    public static readonly (string Name, string Desc)[] Kinds =
    {
        ("void",        "隐藏空洞:连续 10 层挖除 Φ16mm 区域填充"),
        ("rotate",      "填充方向旋转 90°:第 60-100 层承力方向改变"),
        ("density",     "填充密度稀释:每层随机删除 5% 填充线"),
        ("microskip",   "微跳过:每层随机删除 2.5% 填充段"),
        ("tempdrift",   "温度漂移:20 层内渐降 4°C"),
        ("feed",        "速度覆写:填充速度 x1.35 造成欠挤"),
    };

    public static PrintProgram Apply(string kind, PrintProgram src, int startLayer, int seed) =>
        kind switch
        {
            "void" => VoidPocket(src, startLayer, 10),
            "rotate" => InfillRotate(src, startLayer, startLayer + 40),
            "density" => DensityThin(src, startLayer, 0.05, seed),
            "microskip" => MicroSkip(src, startLayer, 0.025, seed),
            "tempdrift" => TempDrift(src, startLayer, 20, -4.0),
            "feed" => FeedOverride(src, startLayer, 1.35),
            _ => throw new ArgumentException($"未知篡改类型: {kind}。可选: {string.Join(", ", Kinds.Select(k => k.Name))}"),
        };
}
