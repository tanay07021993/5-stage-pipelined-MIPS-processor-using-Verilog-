module MuxRegDst (z,x,b,sel);
output reg [4:0] z;
input [4:0] x;
input [4:0] b;
input sel;
always @ (x or b or sel)
begin

if (sel==0)
begin
z=x;
end
else if (sel==1)
begin
z=b;
end
else
begin
z=1'bx;
end
end
endmodule



module Mux2to1 (z, x, b,sel);
output reg [31:0] z;
input [31:0] x;
input [31:0] b;
input sel;
always @ (x or b or sel)
begin

if (sel==0)
begin
z=x;
end
else if (sel==1)
begin
z=b;
end
else
begin
z=1'bx;
end
end
endmodule

module ALU (A,B,op,Result,shiftamt,zero,clk);

output reg [31:0] Result;
output wire zero;
input wire [31:0] A,B;
input wire [3:0] op;
input wire clk;
input wire [4:0] shiftamt;
wire [31:0] B_Neg;
assign B_Neg= -B;

always @(posedge clk)
begin
assign Result= (op==0)? (A+B) : (op==1)? (A-B) :(op==2) ? A&B :
(op==3) ? A|B : (op==4) ? B<<shiftamt:
(op==5) ? A>>shiftamt : (op==6) ? $signed (A)>>>shiftamt :
(op==7 && A>B) ? 1: (op==8 && A<B) ? 1: 0;
end
//assign overflow = (mode==1'b1 && op==0 && A[31]==B[31] && Result[31]==~A[31]) |
//(mode==1'b1 && op==1 && A[31]==B_Neg[31] && Result[31]==~A[31])? 1'b1:1'b0;
assign zero=(A==B)?1:0;

endmodule

module RegFile (clk, rr1, rr2, wr, wdata, wen, data1, data2);
input [4:0] rr1, rr2, wr;
input [31:0] wdata;
input clk, wen;
output reg [31:0] data1,data2;
integer wreg,read1,read2;

reg [31:0] A [0:31];
integer i;
initial
begin
for(i=0;i<32;i=i+1)
begin
A[i]=i;
end
end
always @ (posedge clk)
begin
if (wen==1)
begin
wreg=wr;
A[wreg]<=wdata;
read1=rr1;
read2=rr2;
data1<=A[read1];
data2<=A[read2];
end

else if (wen==0)
begin
read1=rr1;
read2=rr2;
data1<=A[read1];
data2<=A[read2];
end

else
data1=1'bz;
data2=1'bz;

end
always @*
begin
read1=rr1;
read2=rr2;
data1<=A[read1];
data2<=A[read2];
end

endmodule


module Data_Memory(addr,wrdata,memwrite, memread, redata, clk);
input [31:0]addr;
input [31:0] wrdata;
input wire memwrite, memread,clk;
output reg [31:0] redata;
reg [31:0] MEMORY [0:255];
integer i;

initial begin
for(i=0; i<256; i=i+1)
begin
MEMORY[i]=i;
end
end

always @(posedge clk)
begin
if (memwrite==1'b1) begin
MEMORY[addr]<=wrdata;
redata <=0;
end
if(memread==1'b1) begin
redata<=MEMORY[addr];
end
end
endmodule

/*
module tbDM;
reg [31:0]addr;
reg [31:0] wrdata;
reg memwrite, memread,clk;
wire [31:0] redata;
Data_Memory DataMem(addr, wrdata, memwrite, memread, redata, clk);
initial
begin
$monitor ("clk= %b addr=%b memwrite=%b memread=%b redata=%b ", clk,addr, memwrite,memread,redata );
clk=0;
memwrite<=0;
memread<=0;
#3
addr<=0;
memwrite<=1;
wrdata<=5;
#8
addr<=1;
wrdata<=10;
#13
addr<=0;
memwrite<=0;
memread<=1;
#18
addr<=1;
end
always
begin
#5
clk=~clk;
end
endmodule
*/


//// INSTRUCTION MEMORY

module InstructionMemory (ReadAdd, Instruction,clk);
input wire [31:0] ReadAdd;
input wire clk;
output reg [31:0] Instruction;
reg [31:0] InsMem [255:0];

always @(posedge clk)
begin
Instruction <= InsMem[ReadAdd>>2];
end




initial
begin
$readmemb("Processor Project.txt",InsMem);
end


endmodule 


/*
module testInsMem();
reg [31:0]Add;
wire [31:0]Ins;
reg clk;
always 
begin
#5
clk=~clk;
end
initial
begin
clk=0;
$monitor ("%h" ,Ins);
#4 
Add<=0;
#8
Add<=1;
end
InstructionMemory MyTest(Add,Ins,clk);
endmodule 
*/






///// CONTROL UNIT MODULE


//IDEXctrlin[0]<=memwrite; IDEXctrlin[1]<=memread; IDEXctrlin[2]<=wen; IDEXctrlin[3]<= regdst; IDEXctrlin[4]<=memtoreg; IDEXctrlin[5]<=alusrc; IDEXctrlin[6]<=pc_control; IDEXctrlin[6:10]<=aluop;

module control(instruction,memwrite,memread,wen,regdst,memtoreg,aluop,alusrc,pc_control,clk);

output reg memwrite,memread,wen,regdst,memtoreg,alusrc,pc_control ;
output reg [3:0]aluop;

input 	[31:0]	instruction;
input clk;
wire [5:0] op,funct;

assign op = instruction[31:26];
assign funct = instruction[5:0];

always @(negedge clk)
begin


//  SW
if (op == 6'h2B) begin 
memwrite=1'b1;
memread=1'b0;
wen=1'b0;
regdst=1'b0;
memtoreg=1'b0;
alusrc=1'b1;
pc_control=1'b0;
aluop=4'b0000;
end 



// LW

else if(op == 6'h23) begin
memwrite<=1'b0;
memread<=1'b1;
wen<=1'b1;
regdst<=1'b0;
memtoreg<=1'b1;
alusrc<=1'b1;
pc_control<=1'b0;
aluop<=4'b0000;
end



/// R-FORMAT

else if (op ==6'h0 )
begin
memwrite=1'b0;
memread=1'b0;
wen=1'b1;
regdst=1'b1;
memtoreg=1'b0;
alusrc=1'b0;
pc_control=1'b0;

// ADD
if(funct == 6'h20)begin
aluop=4'b0000;end
//SUB
else if (funct == 6'h22)begin
aluop=4'b0001;end
//AND
else if (funct == 6'h24)begin
aluop=4'b0010;end
//OR
else if (funct == 6'h25)begin
aluop=4'b0011;end
//SLL
else if (funct == 6'h0)begin
aluop=4'b0100;end
//SRL
else if (funct == 6'h02)begin
aluop=4'b0101;end

end



// BRANCH
else //(op==6'h04)
begin
memwrite<=1'b0;
memread<=1'b0;
wen<=1'b0;
regdst<=1'b0;
memtoreg<=1'b0;
alusrc<=1'b0;
pc_control<=1'b1;
aluop<=4'b0001;
end


end


endmodule






/*
module TB_CONTROL();
reg clk;
reg [31:0] instruction;
wire memwrite,memread,wen,regdst,memtoreg,alusrc,pc_control ;
wire [2:0]aluop;
always
begin
#5
clk=~clk;
end
initial
begin
clk<=0;
$monitor("clk %b     memwrite %b  memread  %b   wen  %b  regdst  %b   memtoreg   %b   aluop   %b  alusrc   %b    pc_control   %b",
clk,memwrite,memread,wen,regdst,memtoreg,aluop,alusrc,pc_control);
#4
//instruction<=32'h00123120   ;
instruction<= 32'h10123120  ;
#14
instruction<= 32'h00123120  ;
#24
instruction<= 32'h10123120  ;
end
control a(instruction,memwrite,memread,wen,regdst,memtoreg,aluop,alusrc,pc_control,clk);
endmodule
*/





////  SIGN EXTEND


module signextend(in ,out,clk);
input [15:0] in ;
output reg [31:0] out ;
input clk;
//parameter input_width =16 ,output_width=32;
localparam sign_bit_location=15;
localparam sign_bit_replication_count=16;
always@ (posedge clk)
begin
out ={{sign_bit_replication_count{in[sign_bit_location]}},in[15:0]};
end
endmodule



/*
module TB_SI;
reg [15:0] in ;
wire [31:0] out;
initial 
begin
$monitor("%b",out);
#5
in<=16'h1111;
#5
in<=16'hf111;
end
signextend a(in ,out);
endmodule
*/





//// PROGRAM COUNTER




module program_counter(clk,rst,pc,pc_control,branch_offset);
input clk,rst,pc_control;
input [15:0] branch_offset;
output reg[31:0]pc;
//wire [31:0] pc_plus_4;
//assign pc_plus_4=pc+4;
always @(posedge clk or posedge rst)
	begin
		if (rst)
		begin
			pc <= 32'd0;
		end
		else
		begin
			case(pc_control)
					//1'b0 : pc<= pc_plus_4;
					//1'b1 : pc<= pc_plus_4 + { {14{branch_offset[15]}} , branch_offset[15:0] , 2'b00 };
					1'b1 : pc<= pc + branch_offset*4;
					default : pc<= pc+4;
			endcase

		end
	end

endmodule


/*
module TB_PC;
reg clk,rst,pc_control;
reg [15:0] branch_offset;
wire [31:0] pc;
always
begin
#5 clk=~clk;
end
initial
begin
clk<=0;
rst<=1;
#2
rst<=0;
//pc<=32'h00000000;
$monitor($time," PC_control = %d   offset = %d    pc = %h  ",pc_control,branch_offset,pc);
#2
pc_control<=0;
branch_offset<=16'h009a;
#10
pc_control<=1;
branch_offset<=16'h009a;
#10
pc_control<=0;
branch_offset<=16'h0001;
end
program_counter a(clk,rst,pc,pc_control,branch_offset);
endmodule
*/

/*
//////  CPU MODULE 
module cpu();
reg clk,rst;
wire [31:0]instruction;
wire [31:0]  data1_to_A;
wire [31:0] Reg_data2_to_AluSrcMux;
wire [31:0] AluSrcMux_to_B;
wire [31:0] Sextend_out;
wire [4:0] RegDest_mux_to_wrreg;
wire [31:0] ALUout_to_Mem_Addess;
wire [31:0] DataMem_redata_to_Mux3;
wire [31:0] MemtoRegMuxout_to_RegFilewdata;
wire [31:0] pc_to_InstMem;
wire memwrite,memread,wen,regdst,memtoreg,alusrc,pc_control;
wire [3:0]aluop;
wire zero;
wire pc_control_to_pcmodule;
assign pc_control_to_pcmodule= zero & pc_control;
program_counter f(clk,rst,pc_to_InstMem,pc_control_to_pcmodule,instruction[15:0]);
InstructionMemory c(pc_to_InstMem,instruction,clk);
RegFile b(clk, instruction[25:21],instruction [20:16], RegDest_mux_to_wrreg, MemtoRegMuxout_to_RegFilewdata, wen, data1_to_A,  Reg_data2_to_AluSrcMux);
signextend e(instruction[15:0] ,Sextend_out);
Mux2to1 ALUSRC(AluSrcMux_to_B, Reg_data2_to_AluSrcMux, Sextend_out,alusrc);
control d(instruction,memwrite,memread,wen,regdst,memtoreg,aluop,alusrc,pc_control,clk);
ALU a(data1_to_A,AluSrcMux_to_B,aluop,ALUout_to_Mem_Addess,instruction[10:6],zero);
MuxRegDst RegDstMux (RegDest_mux_to_wrreg, instruction[20:16], instruction[15:11],regdst);
Data_Memory t(ALUout_to_Mem_Addess, Reg_data2_to_AluSrcMux, memwrite, memread, DataMem_redata_to_Mux3, clk);
Mux2to1 Mem_to_reg (MemtoRegMuxout_to_RegFilewdata, ALUout_to_Mem_Addess,DataMem_redata_to_Mux3 ,memtoreg);
//RegisterFiles m(instruction,data1_to_A,  Reg_data2_to_AluSrcMux,Sextend_out,ALUout_to_Mem_Addess,zero,RegDest_mux_to_wrreg,DataMem_redata_to_Mux3,ALUout_to_Mem_Addess,MemtoRegMuxout_to_RegFilewdata,clk,instruction,instruction[25:21],instruction [20:16],Sextend_out,data1_to_A,Reg_data2_to_AluSrcMux,zero,ALUout_to_Mem_Addess,Reg_data2_to_AluSrcMux,MemtoRegMuxout_to_RegFilewdata,DataMem_redata_to_Mux3,ALUout_to_Mem_Addess,RegDest_mux_to_wrreg);
always
begin	
#10 clk =~clk;
end
initial
begin
$monitor($time,"     %h     %b     %d     %d    %d    %d",pc_to_InstMem,instruction,data1_to_A,AluSrcMux_to_B,RegDest_mux_to_wrreg,MemtoRegMuxout_to_RegFilewdata);
clk<=0;
rst<=1;
#4
rst<=0;
end
endmodule
*/


//

module RegisterFiles (memwrite, memread,wen,regdst,memtoreg,alusrc,pc_control,aluop,IDEXctrl,EXMEMctrl,MEMWBctrl,Instruction,ReadData1,ReadData2,SignEx_out,ALUresult,zero,MuxRegDest,DataMem_Redata,MuxMem_to_reg_out1,MuxMem_to_reg_out2,clk,IFIDinst,IDEXinst,EXMEMinst,IDEXredata1,IDEXredata2,IDEXSextend_out,EXMEMzero,EXMEMALUout,EXMEMredata2,EXMEMregdst_Mux_Out,MEMWBwrite_register,MEMWBdatamem_redata,MEMWBdatamem_address);
input [31:0]Instruction;
input [31:0]ReadData1;
input [31:0]ReadData2;
input [31:0]SignEx_out;
input [31:0]ALUresult;
input zero;
input [4:0]MuxRegDest;
input [31:0]DataMem_Redata;

input clk; 
input memwrite, memread,wen,regdst,memtoreg,alusrc,pc_control;
input [3:0]aluop;


///Outputs
output [31:0]MuxMem_to_reg_out1;
output [31:0]MuxMem_to_reg_out2;


output reg[31:0] IFIDinst;


output reg[31:0] IDEXinst;
output reg [10:0] IDEXctrl;
output reg[31:0] IDEXredata1;
output reg[31:0] IDEXredata2;
output reg[31:0] IDEXSextend_out;


output reg[31:0] EXMEMinst;
output reg [10:0] EXMEMctrl;
output reg EXMEMzero;
output reg[31:0] EXMEMALUout;
output reg[31:0] EXMEMredata2;
output reg[4:0] EXMEMregdst_Mux_Out;


output reg [10:0] MEMWBctrl;
output reg[4:0] MEMWBwrite_register;
output reg[31:0] MEMWBdatamem_redata;
output reg[31:0] MEMWBdatamem_address;

always @(posedge clk)
begin
IDEXctrl[0]<=memwrite; 
IDEXctrl[1]<=memread; 
IDEXctrl[2]<=wen; 
IDEXctrl[3]<= regdst; 
IDEXctrl[4]<=memtoreg; 
IDEXctrl[5]<=alusrc;
IDEXctrl[6]<=pc_control;
IDEXctrl[10:7]<=aluop;
end


//always @(EXMEMregdst_Mux_Out) begin

//end
/*always @(posedge clk) begin
if(EXMEMctrl[2]&& EXMEMregdst_Mux_Out==IFIDinst[20:16]&&(!(MuxRegDest==IFIDinst[20:16] &&IDEXctrl[2]))) begin
IDEXredata2<=EXMEMALUout;
end
end
*/
//////////////////////////////////////////////////
always @(posedge clk)begin
if (MuxRegDest==IFIDinst[25:21] &&IDEXctrl[2])
begin
IDEXredata1=ALUresult;
end
else if(EXMEMregdst_Mux_Out==IFIDinst[25:21]&&EXMEMctrl[2]&&(!(MuxRegDest==IFIDinst[25:21] &&IDEXctrl[2])))
begin
IDEXredata1<=EXMEMALUout;
end
else begin
IDEXredata1<=ReadData1;
end
end
always @(posedge clk)begin

if (IDEXctrl[2] &&MuxRegDest==IFIDinst[20:16]) 
begin
IDEXredata2<=ALUresult;
end
else if(EXMEMctrl[2]&& EXMEMregdst_Mux_Out==IFIDinst[20:16]&&(!(MuxRegDest==IFIDinst[20:16] &&IDEXctrl[2]))) begin
IDEXredata2<=EXMEMALUout;
end
else begin
IDEXredata2<=ReadData2;
end
end
always @(posedge clk)
begin


///IFID Register
IFIDinst<=Instruction;


//MEMWBinst<=EXMEMinst;
//IDEXrt<=ReadData2;
//IDEXrd<=ReadData1;

///IDEX Register
IDEXinst<=IFIDinst;
//IDEXredata1<=ReadData1;
//IDEXredata2<=ReadData2;
IDEXSextend_out<=SignEx_out;

///EXMEM Register
EXMEMctrl<=IDEXctrl;
EXMEMinst<=IDEXinst;
EXMEMzero<=zero;
EXMEMALUout<=ALUresult;
EXMEMredata2<=IDEXredata2;
EXMEMregdst_Mux_Out<=MuxRegDest;

///MEMWB Register
MEMWBctrl<=EXMEMctrl;
MEMWBwrite_register<=EXMEMregdst_Mux_Out;
MEMWBdatamem_redata<=EXMEMredata2;
MEMWBdatamem_address<=EXMEMALUout;

end

endmodule


module cpu2();


///Clock Registers
reg clk,rst;


///Single Cycle Wires
wire [31:0]instruction;
wire [31:0]  data1_to_A;
wire [31:0] Reg_data2_to_AluSrcMux;
wire [31:0] AluSrcMux_to_B;
wire [31:0] Sextend_out;
wire [4:0] MuxRegDest;
wire [31:0] ALUout_to_Mem_Addess;
wire [31:0] DataMem_redata_to_Mux3;
wire [31:0] MemtoRegMuxout_to_RegFilewdata;
wire [31:0] pc_to_InstMem;
wire memwrite,memread,wen,regdst,memtoreg,alusrc,pc_control;
wire [3:0]aluop;
wire zero;
wire pc_control_to_pcmodule;

///IFID Wires
wire [31:0]IFIDinst;


//wire [31:0]MEMWBinst;

///IDEX Wires
wire [31:0]IDEXinst;
wire [10:0]IDEXctrl;
wire [31:0]IDEXredata1;
wire [31:0]IDEXredata2;
wire [31:0]IDEXSextend_out;


///EXMEM Wires
wire [10:0] EXMEMctrl;
wire [31:0]EXMEMinst;
wire EXMEMzero;
wire [31:0]EXMEMALUout;
wire [31:0]EXMEMredata2;
wire [4:0]EXMEMregdst_Mux_Out;

///MEMWB Wires
wire [10:0]MEMWBctrl; 
wire [31:0]MEMWBdatamem_redata;
wire [31:0]MEMWBdatamem_address;
wire [31:0]MEMWBregdst_Mux_Out;
wire [4:0]MEMWBwrite_register;
wire [31:0]MEMWBDataMem_Redata;



assign pc_control_to_pcmodule= EXMEMzero & EXMEMctrl[6];

program_counter f(clk,rst,pc_to_InstMem,pc_control_to_pcmodule,EXMEMinst[15:0]);

InstructionMemory c(pc_to_InstMem,instruction,clk);
RegFile b(clk, IFIDinst[25:21],IFIDinst [20:16], MEMWBwrite_register, MemtoRegMuxout_to_RegFilewdata, MEMWBctrl[2],data1_to_A,  Reg_data2_to_AluSrcMux);
signextend e(IFIDinst[15:0] ,IDEXSextend_out,clk);



Mux2to1 ALUSRC(AluSrcMux_to_B, IDEXredata2, IDEXSextend_out,IDEXctrl[5]);
control d(IFIDinst,memwrite,memread,wen,regdst,memtoreg,aluop,alusrc,pc_control,clk);
ALU a(IDEXredata1,AluSrcMux_to_B,IDEXctrl[10:7],ALUout_to_Mem_Addess,IDEXinst[10:6],zero,clk);
MuxRegDst RegDstMux (MuxRegDest, IDEXinst[20:16], IDEXinst[15:11],IDEXctrl[3]);
Data_Memory t(EXMEMALUout, EXMEMredata2 , EXMEMctrl[0], EXMEMctrl[1], MEMWBDataMem_Redata, clk);
Mux2to1 Mem_to_reg(MemtoRegMuxout_to_RegFilewdata, MEMWBdatamem_address,MEMWBDataMem_Redata ,MEMWBctrl[4]);


RegisterFiles m(memwrite, memread,wen,regdst,memtoreg,alusrc,pc_control,aluop,IDEXctrl,EXMEMctrl,MEMWBctrl,instruction,data1_to_A,
Reg_data2_to_AluSrcMux,Sextend_out,ALUout_to_Mem_Addess,zero,
MuxRegDest,DataMem_redata_to_Mux3,ALUout_to_Mem_Addess,MemtoRegMuxout_to_RegFilewdata,clk,
IFIDinst,IDEXinst,EXMEMinst,IDEXredata1,IDEXredata2,IDEXSextend_out,EXMEMzero,EXMEMALUout,EXMEMredata2,EXMEMregdst_Mux_Out,
MEMWBwrite_register,MEMWBdatamem_redata,MEMWBdatamem_address);


always
begin	
#10 clk =~clk;
end

initial
begin
$monitor($time,"     %h     %b     %d    %d   %d   ",
pc_to_InstMem,IFIDinst,data1_to_A,  AluSrcMux_to_B,MemtoRegMuxout_to_RegFilewdata,);
clk<=0;
rst<=1;
#5
rst<=0;
end
endmodule
