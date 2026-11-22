using System.Globalization;
using System.Text;

namespace PrintGuard;

// ============================================================
//  G-code 程序模型:移动指令、层、完整打印程序
// ============================================================

public enum MoveKind { Travel, Extrude }

/// <summary>一条空间移动指令(已解析为物理量)。</summary>
public sealed class Move
{
    public MoveKind Kind;
    public double X, Y, Z;     // 目标坐标 mm
    public double Feed;        // mm/min
    public double Len;         // XY 平面路径长度 mm
    public double Cos2;        // 运动方向 cos^2(theta),用于声学方向谱
    public double E;           // 本次挤出的丝料长度 mm
    public bool IsInfill;      // 语义标记:属于内部填充(篡改的主要目标)
    public double Duration => Len <= 1e-12 ? 0.0 : Len / (Feed / 60.0); // 秒
    public Move Clone() => (Move)MemberwiseClone();
}

public sealed class Layer
{
    public int Index;
    public double Z;
    public double NozzleTemp = 200.0;
    public readonly List<Move> Moves = new();
    public Layer Clone()
    {
        var l = new Layer { Index = Index, Z = Z, NozzleTemp = NozzleTemp };
        foreach (var m in Moves) l.Moves.Add(m.Clone());
        return l;
    }
}

public sealed class PrintProgram
{
    public readonly List<Layer> Layers = new();
    public double NozzleTemp = 200.0;
    public PrintProgram Clone()
    {
        var p = new PrintProgram { NozzleTemp = NozzleTemp };
        foreach (var l in Layers) p.Layers.Add(l.Clone());
        return p;
    }
}

// ============================================================
//  G-code 解析 / 输出(支持常见切片器子集)
// ============================================================

public static class GCode
{
    public static PrintProgram Parse(IEnumerable<string> lines)
    {
        var prog = new PrintProgram();
        double x = 0, y = 0, z = 0, e = 0, f = 1200, temp = 200;
        bool relE = false;
        Layer? cur = null;

        foreach (var raw in lines)
        {
            var line = raw;
            int ci = line.IndexOf(';');
            if (ci >= 0) line = line[..ci];
            line = line.Trim();
            if (line.Length == 0) continue;

            var tok = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
            var cmd = tok[0].ToUpperInvariant();

            double? Param(char c)
            {
                for (int i = 1; i < tok.Length; i++)
                    if (tok[i].Length > 1 && char.ToUpperInvariant(tok[i][0]) == c &&
                        double.TryParse(tok[i][1..], NumberStyles.Float, CultureInfo.InvariantCulture, out var v))
                        return v;
                return null;
            }

            switch (cmd)
            {
                case "G0":
                case "G1":
                {
                    var pz = Param('Z');
                    if (pz.HasValue && pz.Value > z + 1e-9)
                    {
                        z = pz.Value;
                        cur = new Layer { Index = prog.Layers.Count, Z = z, NozzleTemp = temp };
                        prog.Layers.Add(cur);
                    }
                    cur ??= new Layer { Index = 0, Z = z, NozzleTemp = temp };
                    if (prog.Layers.Count == 0) prog.Layers.Add(cur);

                    var px = Param('X'); var py = Param('Y'); var pe = Param('E'); var pf = Param('F');
                    if (pf.HasValue) f = pf.Value;
                    double nx = px ?? x, ny = py ?? y;
                    double de = 0;
                    if (pe.HasValue)
                    {
                        de = relE ? pe.Value : pe.Value - e;
                        e = relE ? e + pe.Value : pe.Value;
                    }
                    double dx = nx - x, dy = ny - y;
                    double len = Math.Sqrt(dx * dx + dy * dy);
                    if (len > 1e-9 || de > 1e-9)
                    {
                        bool infill = raw.Contains(";INFILL", StringComparison.OrdinalIgnoreCase);
                        cur.Moves.Add(new Move
                        {
                            Kind = de > 1e-9 ? MoveKind.Extrude : MoveKind.Travel,
                            X = nx, Y = ny, Z = z, Feed = f, Len = len,
                            Cos2 = len > 1e-9 ? dx * dx / (len * len) : 0.0,
                            E = Math.Max(0, de),
                            IsInfill = infill,
                        });
                    }
                    x = nx; y = ny;
                    break;
                }
                case "G92":
                {
                    var pe = Param('E');
                    if (pe.HasValue) e = pe.Value;
                    break;
                }
                case "M82": relE = false; break;
                case "M83": relE = true; break;
                case "M104":
                case "M109":
                {
                    var ps = Param('S');
                    if (ps.HasValue) { temp = ps.Value; prog.NozzleTemp = temp; if (cur != null) cur.NozzleTemp = temp; }
                    break;
                }
            }
        }
        return prog;
    }

    public static string ToGCode(PrintProgram p)
    {
        var sb = new StringBuilder();
        sb.AppendLine("; PrintGuard reference/tampered program");
        sb.AppendLine("G90"); sb.AppendLine("M83");
        sb.AppendLine(FormattableString.Invariant($"M104 S{p.NozzleTemp:F0}"));
        foreach (var layer in p.Layers)
        {
            sb.AppendLine(FormattableString.Invariant($"M104 S{layer.NozzleTemp:F1}"));
            sb.AppendLine(FormattableString.Invariant($"G1 Z{layer.Z:F3} F9000"));
            sb.AppendLine(FormattableString.Invariant($";LAYER:{layer.Index}"));
            foreach (var m in layer.Moves)
            {
                if (m.Kind == MoveKind.Extrude)
                    sb.AppendLine(FormattableString.Invariant(
                        $"G1 X{m.X:F3} Y{m.Y:F3} E{m.E:F5} F{m.Feed:F0} {(m.IsInfill ? ";INFILL" : ";PERIM")}"));
                else
                    sb.AppendLine(FormattableString.Invariant(
                        $"G1 X{m.X:F3} Y{m.Y:F3} F{m.Feed:F0} {(m.IsInfill ? ";INFILL" : ";PERIM")}"));
            }
        }
        sb.AppendLine("M104 S0");
        return sb.ToString();
    }
}

// ============================================================
//  参考零件生成器:40x40x30mm 承重支架(2 层外壳 + 正交填充)
//  用于在没有真实切片文件时做闭环验证;实际部署时替换为
//  客户切片文件经 GCode.Parse 得到的程序即可。
// ============================================================

public static class PartGenerator
{
    public static PrintProgram GenerateBracket(
        double size = 40.0, double layerH = 0.2, int layers = 150,
        double infillSpacing = 2.0, int perimeters = 2,
        double lineW = 0.4, double nozzleTemp = 200.0)
    {
        const double ox = 10.0, oy = 10.0;         // 零件原点
        const double filDia = 1.75;
        double filArea = Math.PI * filDia * filDia / 4.0;
        double ePerMm = lineW * layerH / filArea;  // 每 mm 路径对应的丝料 mm

        var prog = new PrintProgram { NozzleTemp = nozzleTemp };
        var rnd = new Random(7);

        for (int li = 0; li < layers; li++)
        {
            var layer = new Layer { Index = li, Z = (li + 1) * layerH, NozzleTemp = nozzleTemp };

            // ---- 外壳(矩形环) ----
            for (int p = 0; p < perimeters; p++)
            {
                double inset = lineW / 2 + p * lineW;
                double x0 = ox + inset, y0 = oy + inset;
                double x1 = ox + size - inset, y1 = oy + size - inset;
                AddTravel(layer, x0, y0);
                AddExtrude(layer, x1, y0, ePerMm, 1800, false);
                AddExtrude(layer, x1, y1, ePerMm, 1800, false);
                AddExtrude(layer, x0, y1, ePerMm, 1800, false);
                AddExtrude(layer, x0, y0, ePerMm, 1800, false);
            }

            // ---- 内部填充:0°/90° 逐层交替的直线填充 ----
            double inner = perimeters * lineW + 0.15;
            double lo = ox + inner, hi = ox + size - inner;
            bool horizontal = li % 2 == 0;
            bool flip = false;
            for (double c = lo + infillSpacing / 2; c <= hi; c += infillSpacing)
            {
                double jx = (rnd.NextDouble() - 0.5) * 0.02; // 模拟切片器的微小数值差异
                if (horizontal)
                {
                    double y = c + jx;
                    if (flip) { AddTravel(layer, hi, y, infill: true); AddExtrude(layer, lo, y, ePerMm, 3000, true); }
                    else      { AddTravel(layer, lo, y, infill: true); AddExtrude(layer, hi, y, ePerMm, 3000, true); }
                }
                else
                {
                    double x = c + jx;
                    if (flip) { AddTravel(layer, x, hi, infill: true); AddExtrude(layer, x, lo, ePerMm, 3000, true); }
                    else      { AddTravel(layer, x, lo, infill: true); AddExtrude(layer, x, hi, ePerMm, 3000, true); }
                }
                flip = !flip;
            }

            prog.Layers.Add(layer);
        }
        return prog;

        static void AddTravel(Layer l, double x, double y, bool infill = false)
        {
            var last = l.Moves.Count > 0 ? l.Moves[^1] : null;
            double px = last?.X ?? x, py = last?.Y ?? y;
            double dx = x - px, dy = y - py;
            double len = Math.Sqrt(dx * dx + dy * dy);
            // 零长度空驶也要保留:它携带了喷头当前位置,后续挤出段依赖它计算几何
            l.Moves.Add(new Move
            {
                Kind = MoveKind.Travel, X = x, Y = y, Z = l.Z, Feed = 9000,
                Len = len, Cos2 = len > 1e-9 ? dx * dx / (len * len) : 0.0, E = 0, IsInfill = infill,
            });
        }

        static void AddExtrude(Layer l, double x, double y, double ePerMm, double feed, bool infill)
        {
            var last = l.Moves[^1];
            double dx = x - last.X, dy = y - last.Y;
            double len = Math.Sqrt(dx * dx + dy * dy);
            if (len < 1e-9) return;
            l.Moves.Add(new Move
            {
                Kind = MoveKind.Extrude, X = x, Y = y, Z = l.Z, Feed = feed,
                Len = len, Cos2 = dx * dx / (len * len), E = len * ePerMm, IsInfill = infill,
            });
        }
    }
}
