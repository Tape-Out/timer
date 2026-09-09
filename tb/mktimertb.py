"""timer 的行为测试台：数得对不对、比中了会不会置位、写一清不清得掉、捕获锁不锁得住。

认矩阵：`channels` 决定例化几个通道、比较寄存器有几个；`capture` 两个方向都跑——
开着时给通道一个上升沿，捕获寄存器必须锁住计数值；关掉时同一段动作之后必须读回零。
只测开着的那一半，门控写漏了也看不出来。

原来这一份写死 `capture: True`、通道写死 2，而 `capture` 的默认值是 false——
矩阵每一点跑的都不是那一点。
"""
import json
import pathlib
import sys

out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".")
out.mkdir(parents=True, exist_ok=True)
cfg = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
label = cfg.get("label", "")
k = cfg.get("knobs", {})
ch = int(k.get("channels", 2))
capture = bool(k.get("capture", False))

NL = chr(10)
# 通道多于一个时才能验「别的通道没被误触发」
cross = "" if ch < 2 else NL.join([
    "      if (x.rdata[1] == 1) begin",
    '        $display("FAIL channel 1 fired without a compare");',
    "        wrong = True;",
    "      end",
])
capt_chk = (
    NL.join([
        "    if (x.rdata == 0) begin",
        '      $display("FAIL capture register still zero after an edge");',
        "      bad <= True;",
        "    end",
    ]) if capture else NL.join([
        "    // 特性关着：同一段动作走完，捕获寄存器必须还是零",
        "    if (x.rdata != 0) begin",
        '      $display("FAIL capture is off yet the register moved: %08h", x.rdata);',
        "      bad <= True;",
        "    end",
    ]))

txt = f'''package Timer{label}Tb;

import RegIf::*;
import Timer::*;

// 由 tb/mktimertb.py 生成，勿手改。
// 这一点：channels={ch} capture={capture}

Bit#(8) rCTRL  = 8'h00;
Bit#(8) rPRESC = 8'h04;
Bit#(8) rCNT   = 8'h08;
Bit#(8) rCMP0  = 8'h10;
Bit#(8) rCAPT0 = 8'h30;
Bit#(8) rISTA  = 8'h40;
Bit#(8) rIEN   = 8'h44;

typedef enum {{ Setup, Count, Match, Clear, Capture, CheckCapt, Done }}
  Phase deriving (Bits, Eq);

(* synthesize *)
module mkTimer{label}Tb(Empty);
  TimerIfc#(8, 32, {ch}) t <- mkTimer(TimerCfg {{ capture: {str(capture).title()} }});

  Reg#(Phase)    ph  <- mkReg(Setup);
  Reg#(Bit#(8))  s   <- mkReg(0);
  Reg#(Bit#(32)) cyc <- mkReg(0);
  Reg#(Bool)     bad <- mkReg(False);
  Reg#(Bit#({ch})) cap <- mkReg(0);
  Reg#(Bit#(32)) seenCnt <- mkReg(0);
  // `irq` 走 ista 的端口 0、总线访问走端口 1，**读它的规则不能再碰总线**——
  // 否则 CReg 的两个端口撞在一条规则里，bsc 把那条规则整个丢掉，
  // 表现出来是测试超时而不是报错。所以中断只在这里记一笔，别处只看这一笔。
  Reg#(Bool) sawIrq <- mkReg(False);

  rule pins;
    t.pins.capt(cap);
    if (t.irq) sawIrq <= True;
  endrule

  rule timeout;
    cyc <= cyc + 1;
    if (cyc > 40000) begin
      $display("TIMEOUT in phase %0d", pack(ph));
      $finish(1);
    end
  endrule

  function Action wr(Bit#(8) a, Bit#(32) d) = action
    let _ <- t.regs.access(RegReq {{ addr: a, write: True,
                                    wdata: d, wstrb: 4'hF }});
  endaction;

  rule setup (ph == Setup);
    case (s)
      0: wr(rPRESC, 0);          // 不分频，一拍加一
      1: wr(rCMP0, 20);
      2: wr(rIEN, 32'h1);        // 只开 0 通道
      3: wr(rCTRL, 32'h1);       // en
      default: ph <= Count;
    endcase
    s <= s + 1;
  endrule

  // 数到 10 就看一眼计数器读回来是不是真的在走
  rule counting (ph == Count);
    let x <- t.regs.access(RegReq {{ addr: rCNT, write: False,
                                    wdata: 0, wstrb: 4'hF }});
    if (x.rdata >= 10) begin
      seenCnt <= x.rdata;
      ph <= Match;
    end
  endrule

  // 比中之后中断该拉起来，状态位该置上
  rule matching (ph == Match);
    let x <- t.regs.access(RegReq {{ addr: rISTA, write: False,
                                    wdata: 0, wstrb: 4'hF }});
    Bool wrong = False;
    if (x.rdata[0] == 1) begin
{cross}
      ph <= Clear;
    end
    // 一条规则里只写一次 bad，分支里各写各的会被判成并行冲突
    if (wrong) bad <= True;
  endrule

  rule clearing (ph == Clear);
    wr(rISTA, 32'h1);            // 写一清零
    ph <= Capture;
    s  <= 0;
  endrule

  // 通道输入给一个上升沿，计数器该被锁进捕获寄存器
  rule capturing (ph == Capture);
    case (s)
      0: begin
        let x <- t.regs.access(RegReq {{ addr: rISTA, write: False,
                                        wdata: 0, wstrb: 4'hF }});
        if (x.rdata[0] != 0) begin
          $display("FAIL ista not cleared by write one: %08h", x.rdata);
          bad <= True;
        end
      end
      1: cap <= 1;               // 0 通道拉高
      default: ph <= CheckCapt;
    endcase
    s <= s + 1;
  endrule

  rule checkCapt (ph == CheckCapt);
    let x <- t.regs.access(RegReq {{ addr: rCAPT0, write: False,
                                    wdata: 0, wstrb: 4'hF }});
{capt_chk}
    ph <= Done;
  endrule

  rule fin (ph == Done);
    if (!sawIrq) begin
      $display("FAIL irq never went high");
      bad <= True;
    end
    if (bad || !sawIrq) $display("FAILED");
    else $display("PASS timer: counts, compares, clears, captures");
    $finish(bad ? 1 : 0);
  endrule
endmodule

endpackage
'''
(out / f"Timer{label}Tb.bsv").write_text(txt, encoding="utf-8")
print(f"  timer 行为测试台就位：channels={ch} capture={capture}")
