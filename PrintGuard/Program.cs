using System.Threading.Channels;
using PrintGuard;
using Monitor = PrintGuard.Monitor;

// ============================================================
//  printguard — 打印过程完整性监测系统
//    gen      生成参考零件 G-code
//    tamper   生成隐蔽篡改版 G-code(红队验证工具)
//    monitor  实时监测演示(异步流水线,模拟边打印边监测)
//    bench    完整基准实验:标定 + 六类篡改检出层数 + 误停机率
// ============================================================

string Opt(string[] a, string name, string def)
{
    for (int i = 0; i < a.Length - 1; i++)
        if (a[i] == name) return a[i + 1];
    return def;
}

if (args.Length == 0)
{
    Console.WriteLine("""
        用法:
          printguard gen      [-o part.gcode]
          printguard tamper   <void|rotate|density|microskip|tempdrift|feed> [-i part.gcode] [-o out.gcode] [--start 60] [--seed 1]
          printguard monitor  [void|rotate|...]   实时监测演示(默认 density)
          printguard bench    [-o report.md]      完整基准实验
        """);
    return;
}

switch (args[0])
{
    case "gen":
    {
        var prog = PartGenerator.GenerateBracket();
        var path = Opt(args, "-o", "part.gcode");
        File.WriteAllText(path, GCode.ToGCode(prog));
        Console.WriteLine($"已生成参考零件 G-code: {path} ({prog.Layers.Count} 层)");
        break;
    }

    case "tamper":
    {
        if (args.Length < 2) { Console.WriteLine("缺少篡改类型"); return; }
        var kind = args[1];
        var inPath = Opt(args, "-i", "");
        var prog = inPath.Length > 0
            ? GCode.Parse(File.ReadLines(inPath))
            : PartGenerator.GenerateBracket();
        int start = int.Parse(Opt(args, "--start", "60"));
        int seed = int.Parse(Opt(args, "--seed", "1"));
        var tampered = Tamper.Apply(kind, prog, start, seed);
        var outPath = Opt(args, "-o", $"tampered-{kind}.gcode");
        File.WriteAllText(outPath, GCode.ToGCode(tampered));
        Console.WriteLine($"已生成篡改程序: {outPath} (类型 {kind}, 起始层 {start})");
        Console.WriteLine("说明: 外壳与外形尺寸完全不变, 仅内部填充/工艺参数被修改。");
        break;
    }

    case "monitor":
    {
        var kind = args.Length > 1 ? args[1] : "density";
        var refProg = PartGenerator.GenerateBracket();
        var refModel = ReferenceModel.FromProgram(refProg);

        // 快速标定(演示用)
        var raw = new List<double[]>();
        for (int i = 0; i < 12; i++)
        {
            var m = new Monitor(refModel, new Calib()) { RawLog = raw };
            new SensorSimulator(new SimConfig { Seed = i + 1 }).Run(refProg, m.Push);
            m.EndOfPrint();
        }
        var cal = new Calib();
        for (int f = 0; f < 5; f++)
        {
            double mean = raw.Average(r => r[f] - r[5 + f]);
            cal.Std[f] = Math.Max(1e-9, Math.Sqrt(raw.Average(r => Math.Pow(r[f] - r[5 + f] - mean, 2))));
        }
        for (int i = 0; i < 15; i++)
        {
            var c = new Calib { Std = cal.Std, H = new double[] { 1e9, 1e9, 1e9, 1e9, 1e9 } };
            var m = new Monitor(refModel, c);
            new SensorSimulator(new SimConfig { Seed = 100 + i }).Run(refProg, m.Push);
            m.EndOfPrint();
            for (int f = 0; f < 5; f++) cal.H[f] = Math.Max(cal.H[f], m.MaxCusum[f]);
        }
        for (int f = 0; f < 5; f++) cal.H[f] = cal.H[f] * 1.35 + 1.0;
        Console.WriteLine($"标定完成: H = [{string.Join(", ", cal.H.Select(x => x.ToString("F1")))}]");

        // 实时流水线:生产者(打印机+采集卡) → Channel → 消费者(监测器)
        var tampered = Tamper.Apply(kind, refProg, Bench.TamperStartLayer, 42);
        var ch = Channel.CreateBounded<SensorSample>(new BoundedChannelOptions(8192));
        var mon = new Monitor(refModel, cal);
        int maxBacklog = 0;
        string[] featNames = { "挤出量", "耗时", "方向", "温度", "声强" };

        mon.LayerDone += (li, z) =>
        {
            if (li % 10 == 0 || li >= Bench.TamperStartLayer - 1)
                Console.WriteLine($"  层 {li,3}: z = [{string.Join(", ", z.Select(v => v.ToString("F2")))}]");
        };
        mon.Alarm += (li, feat, stat) =>
        {
            var fname = featNames[(int)feat];
            Console.WriteLine($"*** 层 {li}: 检测到内部结构异常({fname}通道, 统计量 {stat:F1})→ 立即停机 ***");
        };

        var consumer = Task.Run(() =>
        {
            while (ch.Reader.WaitToReadAsync().AsTask().Result)
                while (ch.Reader.TryRead(out var s))
                    mon.Push(s);
            mon.EndOfPrint();
        });
        var producer = Task.Run(() =>
        {
            new SensorSimulator(new SimConfig { Seed = 42 }).Run(tampered, s =>
            {
                ch.Writer.TryWrite(s);
                if (ch.Reader.Count > maxBacklog) maxBacklog = ch.Reader.Count;
            });
            ch.Writer.Complete();
        });
        Task.WaitAll(producer, consumer);

        Console.WriteLine(mon.Stopped
            ? $"结果: 第 {mon.AlarmLayer} 层停机(篡改始于第 {Bench.TamperStartLayer} 层, 第 {mon.AlarmLayer - Bench.TamperStartLayer + 1} 个受篡改层被检出)。通道峰值积压 {maxBacklog} 样本 — 监测远快于打印, 不拖慢生产。"
            : "结果: 未检出(不应发生, 请检查标定)。");
        break;
    }

    case "bench":
    {
        var report = Bench.Run(Console.Out);
        var path = Opt(args, "-o", "report.md");
        File.WriteAllText(path, report);
        Console.WriteLine($"报告已写入 {path}");
        break;
    }

    default:
        Console.WriteLine($"未知命令: {args[0]}");
        break;
}
