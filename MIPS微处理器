//利用verilog HDL语言，基于Xilinx FPGA nexys4实验平台，设计一个能够执行以下MIPS指令集的单周期类MIPS处理器，要求完成所有支持指令的功能仿真，验证指令执行的正确性
//1.支持基本的算术逻辑运算如add，sub，and，or，slt指令
//2.支持基本的内存操作如lw，sw指令
//3.支持基本的程序控制如beq，j指令

//1、寄存器组RegFile
//regfile实现代码
module RegFile (RsAddr, RtAddr, WrAddr, DataIn, RegWr,clk, RsData, RtData,reset);
          input [4:0] RsAddr,RtAddr, WrAddr; //写寄存器编号 WrAddr  即Rd
          input [31:0] DataIn;
          input RegWr,clk,reset;  // RegWr: 写控制信号（高电平有效）reset :高电平有效
          output [31:0] RsData, RtData;
          reg [31:0] regs[31:0];    //32个寄存器regs[i]
//R型指令执行时，首先根据指令字段Instr[25...21],Instr[20...16]获取$Rs,$Rt的值
          assign RsData=RsAddr ?regs[RsAddr] : 0;
          assign RtData=RtAddr ?regs[RtAddr] : 0;

          integer i;
          always @(posedge clk or negedge reset)
         if (reset)
                 for(i=0;i<32;i=i+1)  regs[i] = 0;
          else if(!reset && RegWr)
                 regs[ WrAddr]=DataIn; 
endmodule	



//regfile测试代码
`timescale 1ns / 1ps
module RegFileSim();
          reg [4:0] RsAddr,RtAddr, WrAddr; //输入
          reg [31:0] DataIn;    //输入
          reg RegWr,clk,reset;
          wire [31:0] RsData, RtData; //输出
          RegFile U0(RsAddr, RtAddr, WrAddr, DataIn, RegWr,clk, RsData, RtData,reset);
parameter PERIOD = 10;
   initial  clk=1;
   always   #(PERIOD/2) clk = ~clk;
 
 integer i;  //integer类型的变量为有符号数，而reg类型的变量则为无符号数
  initial  begin 
          reset=1;
       #1   reset =0 ; RegWr=1; 
          for(i=0;i<32;i=i+1)
           begin   WrAddr=i;   DataIn=i;   #10;      end
     //  RegWr=0;
     for(i=0;i<32;i=i+1)
          begin  RsAddr=i;  RtAddr=31-i;  #10;  end      
     $stop;
  end
endmodule




//2、ALU模块
//ALU模块实现
module ALU(//组合逻辑
    input signed [31:0] In1,In2,   //32位有符号输入，注意大小比较
                                                //In1:RsData          In2:RtData/Imm32
    input [3:0] ALUCtr, 
    output reg[31:0] ALURes,  //  无符号运算结果 ALURes
    output  Zero    //如果 ALURes 不为零 Zero 为 0，否则 Zero 为 1, 用于大小比较
    );
          assign Zero=(ALURes==0) ?1:0;   
     always @(In1 or In2 or ALUCtr)
          begin
              case(ALUCtr)
                     4'b0110://sub
                              ALURes = In1-In2;
                     4'b0010://add
                              ALURes = In1+In2;
                     4'b0000://and
                              ALURes = In1 & In2;
                    4'b0001://or
                              ALURes = In1 | In2;
                    4'b0111://slt
                              ALURes = (In1<In2) ?1:0;
                    default:    ALURes=0;
              endcase
          end
endmodule

//ALU测试代码
`timescale 1ns / 1ps
module ALUSim();
    reg signed [31:0] In1;
    reg signed [31:0] In2;
    reg [3:0] ALUCtr;
    wire [31:0] ALURes;
    wire  Zero ; 
    ALU U0_ALU(In1,In2,ALUCtr, ALURes,Zero);
    initial
    begin
    In1 = 4;   In2 = 8;
               ALUCtr = 0;//按位与
    #10;   ALUCtr = 1;//按位或
    #10;ALUCtr = 2;   //加
    #10; ALUCtr = 6;  //减
    #10; ALUCtr = 7; //小于设置
    #10;
    In1 = 0 ;
    In2 = 32'hfe;
    ALUCtr = 0;
    #10;ALUCtr = 1;
    #10; ALUCtr = 2;
    #10; ALUCtr = 6;
    #10; ALUCtr = 7;
    #10; 
    In1 = 32'h99999999;
    In2 = 32'h88888888;
    ALUCtr = 0;
    #10;ALUCtr = 1;
    #10;ALUCtr = 2;//溢出
    #10;ALUCtr = 6;
    #10;ALUCtr = 7;
    #10;  $stop;
    end
endmodule


//3、数据存储器dram
dram实现代码：
module dram(
          input [4:0] addr, //数据容量32，5位地址        
          input [31:0] writedata,
          input MemWR,clk, 
          output [31:0] readdata
          );
          
          reg [31:0] regs [31:0];
          assign readdata = regs[addr];
          always @(posedge clk)
          begin
                    if(MemWR)
                    regs[addr] <= writedata;
          end
          integer i;
          initial 
                    for(i=0;i<32;i=i+1) regs[i]=i;
endmodule

//dram测试代码
`timescale 1ns / 1ps
module dram_tb( );
         reg[4:0] addr;
         wire [31:0] readdata;  //输出
         reg [31:0] writedata;
         reg MemWR;
 dram U1(addr,readdata,writedata,MemWR);
 integer i;
 initial begin
     MemWR=1; 
     for(i=0;i<32;i=i+1)
          begin addr=i; writedata=0; #0; end   // #0：瞬间将数据初始化
     for(i=0;i<32;i=i+1)
          begin  addr=i; writedata=4*i; #5; end
     #5  $stop;
end
endmodule



4、指令存储器iROM
iROM实现代码
module iROM( input [4:0] addr,     // 输入程序地址PC[6:2]
                       input clk,
                       output reg [31:0] Instr //输出32位指令机器码
                      ); 
          reg [31:0] regs[12:0];  //指令池
         always @ (posedge clk)    
                Instr <= regs[addr];     //时钟上升沿输出相应地址的指令
endmodule

iROM测试代码
`timescale 1ns / 1ps
module iROM_tb;
          reg [4:0] addr;
          reg clk;
          wire [31:0] Instr;
    iROM  U0(addr,clk,Instr);
    
          initial  clk=0;
          always    #5  clk = ~clk;
       //initial
      //   $readmemh ("C:\\Users\\DWY\\Desktop\\MIPS\\data\\Mcode.txt", U0.regs,0,10);       
    integer i;  
   initial  begin 
           //$readmemh  ("C:/Users/DWY/Desktop/MIPS/data/Mcode.txt", U0.regs,0,10);   
              U0.regs[0]=32'h00432020;
              U0.regs[1]=32'h8c440004;
              U0.regs[2]=32'h00831022;
              U0.regs[3]=32'h00831025;
              U0.regs[4]=32'h00831024;
              U0.regs[5]=32'h0083102a;
              U0.regs[6]=32'h10830001;
              U0.regs[7]=32'h08000000;
              U0.regs[8]=32'h8c620000;
              U0.regs[9]=32'h08000000;
                 
            for(i=0;i<10;i=i+1)
                   begin  addr=i;   #10;    end
       $stop;
  end
endmodule


5、主控制器mainctr
Mainctr实现代码
module mainctr(
          input [5:0] opCode, //Instr[31:26]
          output [1:0] ALUop, //
          output RtDst, regwr, Imm, memwr,B, J, M2R
          );
          reg [8:0] outputtemp;
          assign RtDst   =   outputtemp[8];
          assign Imm    =   outputtemp[7]; //ALUSrc
          assign M2R    =   outputtemp[6];
          assign regwr  =  outputtemp[5]; 
          assign memwr=  outputtemp[4];
          assign  B   =  outputtemp[3];
          assign  J    =  outputtemp[2]; 
          assign ALUop=  outputtemp[1:0];
     always@(opCode)
          case (opCode)
                    6'b000010 : outputtemp = 9'bxxx0_001_xx ;  //jump
                    6'b000000 : outputtemp = 9'b1001_000_10;  //R型指令
                    6'b100011 : outputtemp = 9'b0111_000_00;  //lw 
                    6'b101011 : outputtemp = 9'bx1x0_100_00;  //sw 
                    6'b000100 : outputtemp = 9'bx0x0_010_01;  //beq 
                    6'b001100 : outputtemp = 9'b0101_000_11;  //addi     
                    default   : outputtemp = 9'b0000000000;
          endcase 
endmodule


//6、逻辑运算控制器aluctr
//aluctr实现代码
//R型指令ALUop为10，
//I型指令ALUop lw,sw:00  beq:01 addi:11
module aluctr(
          input [1:0]  ALUop,
          input [5:0]  func,
          output reg [3:0] ALUctr
          );
          always @(ALUop  or  func)
          casex( {ALUop, func} )
                    8'b00xxxxxx :       ALUctr=4'b0010; //lw,sw 
                    8'b01xxxxxx :       ALUctr=4'b0110; //beq
                    8'b11xxxxxx :       ALUctr=4'b0000; //andi 
                    8'b10xx0000:       ALUctr=4'b0010; //add 
                    8'b10xx0010:       ALUctr=4'b0110; //sub 
                    8'b10xx0100:       ALUctr=4'b0000; //and 
                    8'b10xx0101:       ALUctr=4'b0001; //or 
                    8'b10xx1010:       ALUctr=4'b0111; //slt 
                    default :              ALUctr=4'b0000;
          endcase
endmodule


//7、顶层综合模块
//综合模块实现代码
module MIPSCPU_2 ( 
           input clk,
           input reset 
            );
            wire [31:0] TempPC, MuxPC, JumpPC, BranchPC, SquencePC, Imm32, ImmL2, RegWD, RsData, RtData, ALUIn2, ALURes, MemRD, Instr;
            wire [4:0] RegWA;
            wire [27:0] PsudeoPC;
            wire BranchZ, J, B, Zero, RegDst, RegWr, ALUSrc, MemWR, Mem2Reg;
            wire [1:0] ALUOp;
            wire [3:0] ALUCtr;
            reg [31:0] PC;
            assign PsudeoPC = { Instr[25:0] , 2'b00 };
            assign JumpPC = { SquencePC[31:28], PsudeoPC};
            assign SquencePC = PC + 4;  //顺序执行的PC
            assign BranchPC = ImmL2 + SquencePC;
            assign MuxPC = BranchZ ? BranchPC : SquencePC;
            assign TempPC = J ? JumpPC : MuxPC; //PC的次态
            assign BranchZ = B&Zero;
            assign ImmL2 = { Imm32[29:0] , 2'b00 };
            assign Imm32 = { Instr[15] ? 16'hffff : 16'h0 , Instr[15:0] };
            assign ALUIn2 = ALUSrc  ?    Imm32  :  RtData;
            assign RegWA  = RegDst  ? Instr[15:11] : Instr[20:16];
            assign RegWD = Mem2Reg ? MemRD : ALURes;
            ALU UnitALU(RsData, ALUIn2, ALUCtr, ALURes,Zero);
            dram UnitDram (ALURes[6:2], RtData, MemWR,clk, MemRD);
            iROM Unitirom(PC[6:2],~clk, Instr);
            RegFile UnitRegFile(Instr[25:21], Instr[20:16], RegWA, RegWD,RegWr, clk, RsData, RtData, reset); 
            mainctr Unitmainctr(Instr[31:26],ALUOp,RegDst,RegWr,ALUSrc, MemWR,B,J, Mem2Reg);
            aluctr Unitaluctr(ALUOp, Instr[5:0], ALUCtr);
       always @ (posedge clk)
       begin 
           if(reset)   PC <= 0;
           else     PC <= TempPC;
       end
endmodule




//顶层模块测试代码
`timescale 1ns / 1ps
module MIPSCPU_tb();
          reg clk,reset;
          MIPSCPU_2 u1(clk,reset);
          
          integer i;
          parameter PERIOD = 10;
             initial  clk=0;
             always   #(PERIOD/2) clk = ~clk;
        
           initial begin
             $readmemh ("C:\\Users\\DWY\\Desktop\\MIPS\\data\\Mcode.coe", u1.Unitirom.regs,0,12); 
             reset=1; #20;
            for(i=0;i<32;i=i+1)
            begin      //RegFile 以及 Dram 前 32 个存储单元存储的初始值为单元地址*4
              u1.UnitRegFile.regs[i]=i*4;
              u1.UnitDram.regs[i]=i*4;
            end      
              reset=0;
              #200  $stop;
         end         
endmodule
