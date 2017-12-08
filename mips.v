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

module ALU (A,B,op,mode,Result,shiftamt,zero);

output wire [31:0] Result;
output wire zero;
input wire [31:0] A,B;
input wire [3:0] op;
input wire [31:0] shiftamt;
input mode;
wire [31:0] B_Neg;

assign B_Neg= -B;

assign Result = (op==0)? (A+B) : (op==1)? (A-B) :(op==2) ? A&B :
(op==3) ? A|B : (op==4) ? A<<shiftamt:
(op==5) ? A>>shiftamt : (op==6) ? $signed (A)>>>shiftamt :
(op==7 && A>B) ? 1: (op==8 && A<B) ? 1: 0;

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


module Data_Memory(addr, wrdata, memwrite, memread, redata, clk);
input wire [31:0]addr;
input wire [31:0] wrdata;
input wire memwrite, memread,clk;
output reg [31:0] redata;
reg [31:0] MEMORY [0:255];
integer i;

initial begin
redata <=0;
for(i=0; i<256; i=i+1)
begin
MEMORY[i]=i;
end
end

always @(posedge clk)
begin
if (memwrite==1'b1) begin
MEMORY[addr]<=wrdata;
end
if(memread==1'b1) begin
redata<=MEMORY[addr];
end
end
endmodule


/*module tbDM;
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
Instruction <= InsMem[ReadAdd];
end

integer i;



initial
begin
$readmemh("Processor Project.txt",InsMem);
end


endmodule 




/*module testInsMem();
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




module control(instruction,memwrite,memread,wen,regdst,memtoreg,aluop,alusrc,pc_control,clk);

output reg memwrite,memread,wen,regdst,memtoreg,alusrc,pc_control ;
output reg [2:0]aluop;

input 	[31:0]	instruction;
input clk;
wire [5:0] op,funct;

assign op = instruction[31:26];
assign funct = instruction[5:0];

always @(posedge clk)
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
aluop=3'b000;
end 



// LW

if(op == 6'h23) begin
memwrite<=1'b0;
memread<=1'b1;
wen<=1'b1;
regdst<=1'b0;
memtoreg<=1'b1;
alusrc<=1'b1;
pc_control<=1'b0;
aluop<=3'b000;
end



/// R-FORMAT

if (op ==6'h0 )
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
aluop=3'b000;end
//SUB
else if (funct == 6'h22)begin
aluop=3'b001;end
//AND
else if (funct == 6'h24)begin
aluop=3'b010;end
//OR
else if (funct == 6'h25)begin
aluop=3'b011;end
//SLL
else if (funct == 6'h0)begin
aluop=3'b100;end
//SRL
else if (funct == 6'h02)begin
aluop=3'b101;end

end



// BRANCH
if (op==6'h04)
begin
memwrite<=1'b0;
memread<=1'b0;
wen<=1'b0;
regdst<=1'b0;
memtoreg<=1'b0;
alusrc<=1'b0;
pc_control<=1'b1;
aluop<=3'b001;
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


module signextend(in ,out);
input [15:0] in ;
output [31:0] out ;
//parameter input_width =16 ,output_width=32;
localparam sign_bit_location=15;
localparam sign_bit_replication_count=16;
assign out ={{sign_bit_replication_count{in[sign_bit_location]}},in[15:0]};
endmodule




/*module TB_SI;

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
output reg [31:0]pc;
wire [31:0] pc_plus_4;
assign pc_plus_4=pc+4;
always @(posedge clk or posedge rst)
	begin
		if (rst)
		begin
			pc <= 32'd0;
		end
		else
		begin
			case(pc_control)
					1'b0 : pc<= pc_plus_4;
					1'b1 : pc<= pc_plus_4 + { {14{branch_offset[15]}} , branch_offset[15:0] , 2'b00 };
					default : pc<= pc_plus_4;
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
$monitor($time," control = %b   offset = %h    pc = %h  ",pc_control,branch_offset,pc);
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
Mux2to1 ALUSRC(AluSrcMux_to_B, Reg_data2_to_AluSrcMux, Sextend_out,alusrc);
MuxRegDst RegDstMux (RegDest_mux_to_wrreg, instruction[20:16], instruction[15:11],regdst);
Mux2to1 Mem_to_reg (MemtoRegMuxout_to_RegFilewdata, ALUout_to_Mem_Addess,DataMem_redata_to_Mux3 ,memtoreg);
ALU a(data1_to_A,AluSrcMux_to_B,aluop,ALUout_to_Mem_Addess,instruction[10:6],zero);
RegFile b(clk, instruction[25:21],instruction [20:16], RegDest_mux_to_wrreg, MemtoRegMuxout_to_RegFilewdata, wen, data1_to_A,  Reg_data2_to_AluSrcMux);
Data_Memory t(ALUout_to_Mem_Addess, Reg_data2_to_AluSrcMux, memwrite, memread, DataMem_redata_to_Mux3, clk);
InstructionMemory c(pc_to_InstMem, instruction,clk);
control d(instruction,memwrite,memread,wen,regdst,memtoreg,aluop,alusrc,pc_control,clk);
signextend e(instruction[15:0] ,Sextend_out);
program_counter f(clk,rst,pc_to_InstMem,pc_control_to_pcmodule,instruction[15:0]);

always
begin
#10 clk =~clk;
end

initial
begin
$monitor($time,"   %h     %h     %h     %h    %h",pc_to_InstMem,ALUout_to_Mem_Addess,data1_to_A,Reg_data2_to_AluSrcMux,aluop);
clk<=0;
rst<=1;
#4
rst<=0;


end



endmodule

//////// Register Files Module

module PipelineRegisters(Instruction,pc,ReadData1,ReadData2,SignEx_out,ALUresult,zero,MuxRegDest,DataMem_Redata,MuxMem_to_reg_out1,MuxMem_to_reg_out2,clk);
input [31:0]Instruction;
input [31:0]pc;
input [31:0]ReadData1;
input [31:0]ReadData2;
input [31:0]SignEx_out;
input [31:0]ALUresult;
input zero;
input [4:0]MuxRegDest;
input [31:0]DataMem_Redata;
output [31:0]MuxMem_to_reg_out1;
output [31:0]MuxMem_to_reg_out2;
input clk;

output reg[31:0] IFIDinst;
output reg[31:0] IDEXredata1;
output reg[31:0] IDEXredata2;
output reg[31:0] IDEXSextend_out;
output reg[31:0] IDEXrd;
output reg[31:0] IDEXrt;
output reg EXMEMzero;
output reg[31:0] EXMEMALUout;
output reg[31:0] EXMEMredata2;
output reg[31:0] EXMEMregdst_Mux_Out;
output reg[31:0] MEMWBdatamem_redata;
output reg[31:0] MEMWBdatamem_address;
output reg[31:0] MEMWBregdst_Mux_Out;

always @(negedge clk)
begin 
IFIDinst=Instruction;
IDEXrt=Instruction[20:16];
IDEXrd=Instruction[15:11];
IDEXredata1=ReadData1;
IDEXredata2<=ReadData2;
IDEXSextend_out=SignEx_out;
EXMEMzero=zero;
EXMEMALUout<=ALUresult;
EXMEMredata2<=IDEXredata2;
EXMEMregdst_Mux_Out=MuxRegDest;
MEMWBdatamem_redata=DataMem_Redata;
MEMWBdatamem_address<=EXMEMALUout;
end

endmodule


