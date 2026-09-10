package Timer;

import ConfigReg::*;
import Vector::*;
import RegIf::*;
import TimerRegs::*;

// 本包不认识任何总线：对外只给中立的 RegIf，接哪种总线由 wrap 或装配决定。
typedef struct {
  Bool capture;
} TimerCfg;

interface TimerPins#(numeric type channels);
  (* always_ready, always_enabled, prefix = "" *)
  method Action capt((* port = "capt_in" *) Bit#(channels) v);
endinterface

interface TimerIfc#(numeric type aw, numeric type dw, numeric type channels);
  interface RegIf#(aw, dw) regs;
  interface TimerPins#(channels) pins;
  (* always_ready *) method Bool irq;
endinterface

module mkTimer#(TimerCfg cfg)(TimerIfc#(aw, dw, channels))
    provisos (Mul#(TDiv#(dw, 8), 8, dw), Add#(_a, 8, aw), Add#(_b, 32, dw),
              Add#(_c, 16, dw), Add#(_d, 8, dw), Add#(_e, 1, dw),
              Add#(_f, channels, 8), Log#(TAdd#(channels, 1), _g),
              Add#(_h, TLog#(TAdd#(channels, 1)), 8));

  TimerRegsIfc#(aw, dw, channels) r <- mkTimerRegs(
      TimerRegsCfg { capture: cfg.capture });

  // 捕获要读计数器、计数器规则要读配置寄存器、总线又要写它们，
  // 三者首尾相接成环。ConfigReg 让读恒取旧值，环就断了（D39）。
  Reg#(Bit#(32)) cnt  <- mkConfigReg(0);
  Reg#(Bit#(16)) div  <- mkReg(0);

  // 比较取「跨过」而不是「正等于」。ACLINT 规范的原话是 MTIME 大于等于 MTIMECMP
  // 就挂起，我们自己的 aclint 照抄了那一句，而这里写的是正等于——**同一颗芯片里
  // 两个定时器对「比较」的定义不一样**。代价藏得深：设在过去的比较值要等计数器
  // 绕满一圈才响，32 位在 100MHz 下是四十三秒，而「读计数、加个差值、写回去」
  // 正是最常见的用法，差值算小了或者写晚一格就整整丢掉一圈。
  //
  // 电平比较不能直接用：ista 是写一清零的粘滞位，跨过之后每拍重新置上，
  // 软件刚清掉就被硬件顶起来。所以取边沿——这一拍过了、上一拍还没过。
  // 上一拍的状态复位成「已经过了」，使能那一刻就不会所有通道全响。
  Reg#(Bit#(8)) prevGe <- mkConfigReg(8'hFF);
  // 软件写了新的比较值就强制记成「还没过」，于是设在过去的下一拍当场响。
  // 没有这一条，大于等于与正等于在这件事上表现一样。
  Reg#(Bool) wrPend <- mkReg(False);
  Reg#(Bit#(TLog#(TAdd#(channels, 1)))) wrIdx <- mkReg(0);

  rule tick;
    r.cnt_in(cnt);
    if (r.ctrl_rst == 1) begin
      cnt <= 0;
      div <= 0;
    end else if (r.ctrl_en == 1) begin
      if (div >= r.presc) begin
        div <= 0;
        cnt <= cnt + 1;
      end else
        div <= div + 1;
    end
  endrule

  // 总线方法发的脉冲要先落一拍再用。直接在 cmpHit 里读它，这条规则就被要求
  // 排在总线方法之后，而它写的 ista 又要求排在之前，bsc 会把整条规则丢掉——
  // 症状是「某个阶段挂住」，不是报错。
  rule mark;
    wrPend <= r.cmp_wr;
    wrIdx  <= r.cmp_wr_i;
  endrule

  // 比较命中置位，写 1 清除由寄存器组代管
  rule cmpHit (r.ctrl_en == 1);
    Bit#(8) hit = 0;
    Bit#(8) ge  = 0;
    for (Integer i = 0; i < valueOf(channels); i = i + 1) begin
      Bool now  = cnt >= r.cmp[i];
      Bool mine = wrPend && wrIdx == fromInteger(i);
      ge[i] = pack(now && !mine);
      if (now && !mine && prevGe[i] == 0) hit[i] = 1;
    end
    r.ista_set(hit);
    prevGe <= ge;
  endrule

  Wire#(Bit#(channels)) capt     <- mkBypassWire;
  Reg#(Bit#(channels))  captPrev <- mkReg(0);

  if (cfg.capture) begin
    // 每一路各有写口（regmap 的 hweach），同拍来几个边沿就锁几个。
    // 原来按下标写，一拍只锁得了编号最小的那一路，**其余的连个记号都不留**——
    // 而捕获单元的全部职责就是给外部边沿打时间戳。两个传感器同拍翻转并不稀奇。
    rule doCapture;
      captPrev <= capt;
      Bit#(channels) rise = capt & ~captPrev;
      Vector#(channels, Maybe#(Bit#(32))) v = newVector;
      for (Integer i = 0; i < valueOf(channels); i = i + 1)
        v[i] = rise[i] == 1 ? tagged Valid cnt : tagged Invalid;
      r.capt_in(v);
    endrule
  end

  interface regs = r.regs;
  interface TimerPins pins;
    method Action capt(Bit#(channels) v); capt._write(v); endmethod
  endinterface
  method Bool irq = (r.ista & r.ien) != 0;
endmodule

endpackage
