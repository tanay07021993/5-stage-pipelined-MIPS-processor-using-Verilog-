//MuxRegDst RegDstMux (MuxRegDest, IDEX_rt, IDEX_rd,regdst);
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



//Mux2to1 Mem_to_reg(MemtoregOut_writedata, MEMWBdatamem_address,MEMWBdatamem_redata ,memtoreg);
//Mux2to1 ALUSRC(AluSrcMux_to_B, IDEXredata2, IDEXSextend_out,alusrc);

module Mux2to1 (z, x, b,sel);
output reg signed[31:0] z;
input signed[31:0] x;
input signed[31:0] b;

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




///////////////////////////////////////////////////////   ALU   /////////////////////////////////////////////////////////////////////////



/*ALU a(IDEXredata1,AluSrcMux_to_B,aluop,ALUout_to_Mem_Addess,IFIDinst[10:6],zero);*/

module ALU (A,B,op,Result,shiftamt,clk);
input clk;
output reg signed[31:0] Result;
//output wire zero;
input wire signed [31:0] A,B;
input wire [3:0] op;
input wire [4:0] shiftamt;
wire signed [31:0] B_Neg;

assign B_Neg= -B;

always @(posedge clk)
begin
assign Result = (op==0)? (A+B) : (op==1)? (A-B) :(op==2) ? A&B :
(op==3) ? A|B : (op==4) ? B<<shiftamt:
(op==5) ? B>>shiftamt : (op==6) ? $signed (B)>>>shiftamt :
(op==7 && A>B) ? 1: (op==8 && A<B) ? 1: 0;
end
//assign overflow = (mode==1'b1 && op==0 && A[31]==B[31] && Result[31]==~A[31]) |
//(mode==1'b1 && op==1 && A[31]==B_Neg[31] && Result[31]==~A[31])? 1'b1:1'b0;

//assign zero=(A==B)?1:0;


endmodule


/////////////////////////////////////////////////   REGISTER   FILE   //////////////////////////////////////////////////////////////


//RegFile b(clk, IFIDinst[25:21],IFIDinst [20:16], MEMWBwrite_register, MemtoregOut_writedata, wen,data1_to_A,  Reg_data2_to_AluSrcMux);

module RegFile (clk, rr1, rr2, wr, wdata, wen, data1, data2);
input [4:0] rr1, rr2, wr;
input signed[31:0] wdata;
input clk, wen;
output reg signed [31:0] data1,data2;
integer wreg,read1,read2;

reg signed[31:0] A [0:31];
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



/////////////////////////////////////////////////////////    DATA  MEMORY   //////////////////////////////////////////////////////////////     


//Data_Memory t(EXMEMALUout, EXMEMredata2 , memwrite, memread, MEMWB_DataMem_Redata, clk);



module Data_Memory(addr, wrdata, memwrite, memread, redata, clk);
input wire [31:0]addr;
input wire signed[31:0] wrdata;
input wire memwrite, memread,clk;
output reg signed[31:0] redata;
reg signed[31:0] MEMORY [0:255];
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




//////////////////////////////////////////////////// INSTRUCTION   MEMORY   //////////////////////////////////////////////////////////////


//InstructionMemory c(pc_to_InstMem,instruction,clk);

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
$readmemh("Processor Project.txt",InsMem);
end


endmodule 








/////////////////////////////////////////////////// CONTROL UNIT MODULE   ////////////////////////////////////////////////////////////////


//IDEXctrlin[0]<=memwrite; IDEXctrlin[1]<=memread; IDEXctrlin[2]<=wen; IDEXctrlin[3]<= regdst; IDEXctrlin[4]<=memtoreg; IDEXctrlin[5]<=alusrc; IDEXctrlin[6]<=pc_control; IDEXctrlin[6:10]<=aluop;


//module control(instruction,memwrite,memread,wen,regdst,memtoreg,aluop,alusrc,pc_control,clk);



//control d(IFIDinst,memwrite,memread,wen,regdst,memtoreg,aluop,alusrc,pc_control,clk);


module control(instruction,Control_Signals,clk);

//output reg memwrite,memread,wen,regdst,memtoreg,alusrc,pc_control ;
//output reg [3:0]aluop;
output reg [10:0] Control_Signals;
input 	[31:0]	instruction;
input clk;
wire [5:0] op,funct;

assign op = instruction[31:26];
assign funct = instruction[5:0];

always @(negedge clk)
begin


//  SW
if (op == 6'h2B) begin 
Control_Signals[0]<=1'b1;	// memwrite
Control_Signals[1]<=1'b0;	// memread
Control_Signals[2]<=1'b0;	//wen
Control_Signals[3]<=1'b0;	//regdst
Control_Signals[4]<=1'b0;	//memtoreg
Control_Signals[5]<=1'b1;	//alusrc	
Control_Signals[6]<=1'b0;	//pc_control
Control_Signals[10:7]<=4'b0000;  //aluop
end 



// LW

else if(op == 6'h23) begin
Control_Signals[0]<=1'b0;	// memwrite
Control_Signals[1]<=1'b1;	// memread
Control_Signals[2]<=1'b1;	//wen
Control_Signals[3]<=1'b0;	//regdst
Control_Signals[4]<=1'b1;	//memtoreg
Control_Signals[5]<=1'b1;	//alusrc	
Control_Signals[6]<=1'b0;	//pc_control
Control_Signals[10:7]<=4'b0000;  //aluop
end



/// R-FORMAT

else if (op ==6'h0 )
begin
Control_Signals[0]<=1'b0;	// memwrite
Control_Signals[1]<=1'b0;	// memread
Control_Signals[2]<=1'b1;	//wen
Control_Signals[3]<=1'b1;	//regdst
Control_Signals[4]<=1'b0;	//memtoreg
Control_Signals[5]<=1'b0;	//alusrc	
Control_Signals[6]<=1'b0;	//pc_control


// ADD
if(funct == 6'h20)begin
Control_Signals[10:7]<=4'b0000;  //aluop
end
//SUB
else if (funct == 6'h22)begin
Control_Signals[10:7]<=4'b0001;  //aluop
end
//AND
else if (funct == 6'h24)begin
Control_Signals[10:7]<=4'b0010;  //aluop
end
//OR
else if (funct == 6'h25)begin
Control_Signals[10:7]<=4'b0011;  //aluop
end
//SLL
else if (funct == 6'h0)begin
Control_Signals[10:7]<=4'b0100;  //aluop
end
//SRL
else if (funct == 6'h02)begin
Control_Signals[10:7]<=4'b0101;  //aluop

end
end


// BRANCH
else if (op==6'h04)
begin
Control_Signals[0]<=1'b0;	// memwrite
Control_Signals[1]<=1'b0;	// memread
Control_Signals[2]<=1'b1;	//wen
Control_Signals[3]<=1'b1;	//regdst
Control_Signals[4]<=1'b0;	//memtoreg
Control_Signals[5]<=1'b0;	//alusrc	
Control_Signals[6]<=1'b1;	//pc_control
Control_Signals[10:7]<=4'b0001;  //aluop

end


end


endmodule











/////////////////////////////////////////////////////    SIGN EXTEND     /////////////////////////////////////////////////////////////////////

//signextend e(IFIDinst[15:0] ,Sextend_out);
module signextend(in ,out,clk);
input [15:0] in ;
input clk;
output reg [31:0] out ;
//parameter input_width =16 ,output_width=32;
localparam sign_bit_location=15;
localparam sign_bit_replication_count=16;
always @(negedge clk)
begin
out ={{sign_bit_replication_count{in[sign_bit_location]}},in[15:0]};
end
endmodule







/////////////////////////////////////////////////////// PROGRAM COUNTER   /////////////////////////////////////////////////////////////////



//program_counter f(clk,rst,pc_to_InstMem,pc_control_to_pcmodule,instruction[15:0]);
/*module program_counter(clk,rst,pc,pc_control,IDEXSextend_out,IDEX_PC);
input clk,rst,pc_control;
input [31:0] IDEXSextend_out;
input [31:0] IDEX_PC;
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
					1'b1 : pc<= IDEX_PC + {IDEXSextend_out[29:0] , 2'b00 };
					default : pc<= pc_plus_4;
			endcase

		end
	end
endmodule

*/





module RegisterFiles (pc_to_InstMem,IDEX_PC,clk,Control_Signals,IDEXctrl,EXMEMctrl,MEMWBctrl,Instruction,IDEXinst,IFIDinst,IDEX_ReadData1,IDEX_ReadData2,IDEXredata1,IDEXredata2,IDEX_SignEx_out,IDEXSextend_out,IDEX_rd,IDEX_rt,

EXMEM_ALUresult,EXMEMALUout,EXMEMredata2,EXMEM_MuxRegDest,

MEMWB_DataMem_Redata,MEMWB_DataMem_redata,MEMWBwrite_register,MEMWBdatamem_address);

input clk; 
input [31:0] pc_to_InstMem;
 //reg [31:0] IFID_PC;
output reg [31:0] IDEX_PC; 
//// CONTROL SIGNAL REGISTER

input [10:0]Control_Signals;
output reg [10:0] IDEXctrl;
output reg [10:0] EXMEMctrl;
output reg [10:0] MEMWBctrl;





/////// INSTRUCTION REGISTER

input [31:0]Instruction;

output reg[31:0] IFIDinst;
output reg[31:0] IDEXinst;
reg[31:0] EXMEMinst;
reg[31:0] MEMWBinst;



///IDEX REGISTER

input signed[31:0]IDEX_ReadData1;/////
input signed[31:0]IDEX_ReadData2;////
input [31:0]IDEX_SignEx_out;///
output reg [4:0]IDEX_rt;/////
output reg [4:0]IDEX_rd;////
output reg signed[31:0] IDEXredata1;
output reg signed[31:0] IDEXredata2;
output reg[31:0] IDEXSextend_out;


/// EXMEM REGISTER

//input zero;
input signed[31:0]EXMEM_ALUresult;////
input [4:0]EXMEM_MuxRegDest;//
//output reg EXMEMzero;
output reg signed[31:0] EXMEMALUout;
output reg signed[31:0] EXMEMredata2;
reg[4:0] EXMEMregdst_Mux_Out;


/// MEMWB REGISTER

input signed[31:0]MEMWB_DataMem_Redata;//
output reg[4:0] MEMWBwrite_register;
output reg signed[31:0] MEMWB_DataMem_redata;
output reg signed[31:0] MEMWBdatamem_address;



always @(negedge clk)
 begin
 IDEXctrl<=Control_Signals;
 EXMEMctrl<=IDEXctrl;
 MEMWBctrl<=EXMEMctrl;
end




always @(posedge clk)
begin


//IFID_PC<=pc_to_InstMem;
IDEX_PC<=pc_to_InstMem;


////// CONTROL






/// INSTRUCTION REGISTER

IFIDinst<=Instruction;
IDEXinst<=IFIDinst;
EXMEMinst<=IDEXinst;
MEMWBinst<=EXMEMinst;


/// IDEX REGISTER

IDEX_rt<=IFIDinst[20:16];
IDEX_rd<=IFIDinst[15:11];
IDEXredata1<=IDEX_ReadData1;
IDEXredata2<=IDEX_ReadData2;
IDEXSextend_out<=IDEX_SignEx_out;



///EXMEM REGISTER


//EXMEMzero<=zero;
EXMEMALUout<=EXMEM_ALUresult;
EXMEMredata2<=IDEXredata2;
EXMEMregdst_Mux_Out<=EXMEM_MuxRegDest;



/// MEMWB REGISTER 


MEMWBwrite_register<=EXMEMregdst_Mux_Out;
MEMWB_DataMem_redata<=MEMWB_DataMem_Redata;
MEMWBdatamem_address<=EXMEMALUout;
end

endmodule










module cpu2();
reg clk,rst;

wire [10:0] Control_Signals;
reg [10:0] Control_Signalsout;
wire [10:0] IDEXctrl;
wire [10:0] EXMEMctrl;
wire [10:0] MEMWBctrl; 
wire [31:0]instruction;


wire [31:0]IFIDinst;
wire [31:0] IDEXinst;
wire signed[31:0]  data1_to_A;
wire signed[31:0] Reg_data2_to_AluSrcMux;
wire signed[31:0]IDEXredata1;
wire signed[31:0]IDEXredata2;
wire signed[31:0] AluSrcMux_to_B;             /// selka ben el alusrc mux wel fr3 eltany lel alu
wire [31:0] Sextend_out;
wire [31:0]IDEXSextend_out;
wire [4:0] IDEX_rt;
wire [4:0] IDEX_rd;





wire signed[31:0] ALUout_to_Mem_Addess;
wire signed[31:0]EXMEMALUout;
wire signed[31:0]EXMEMredata2;
wire [4:0] MuxRegDest;



wire signed[31:0]MEMWB_DataMem_Redata;
wire signed[31:0]MEMWBdatamem_redata;
wire [4:0]MEMWBwrite_register;
wire signed[31:0]MEMWBdatamem_address;
wire signed[31:0]MemtoregOut_writedata;           ////wire ben memtoreg l write data fel register file


reg [31:0] pc_to_InstMem;
wire [31:0] IDEX_PC;
wire pc_control_to_pcmodule;
wire zero;
wire [31:0] pc_plus_4;
assign pc_plus_4=pc_to_InstMem+4;

assign zero= (data1_to_A==Reg_data2_to_AluSrcMux)?1:0;
assign pc_control_to_pcmodule= zero & Control_Signals[6];
always @(posedge clk or posedge rst)
	begin
		if (rst)
		begin
			pc_to_InstMem <= 32'd0;
		end
		else
		begin
		case(pc_control_to_pcmodule)
					1'b0 : pc_to_InstMem<= pc_plus_4;
					1'b1 : pc_to_InstMem<= IDEX_PC + {Sextend_out[29:0] , 2'b00 };
					default : pc_to_InstMem<= pc_plus_4;
			endcase
end
end


always@(IDEXctrl[6] or Control_Signals)           /// stall of branch
begin
if(IDEXctrl[6]==0)
begin
Control_Signalsout=Control_Signals;
end
else
begin
Control_Signalsout=0;
end
end



//program_counter f(clk,rst,pc_to_InstMem,pc_control_to_pcmodule,Sextend_out,IDEX_PC);

InstructionMemory c(pc_to_InstMem,instruction,clk);
RegFile b(clk, IFIDinst[25:21],IFIDinst [20:16], MEMWBwrite_register, MemtoregOut_writedata, MEMWBctrl[2],data1_to_A,  Reg_data2_to_AluSrcMux);
signextend e(IFIDinst[15:0] ,Sextend_out,clk);


//control d(IFIDinst,memwrite,memread,wen,regdst,memtoreg,aluop,alusrc,pc_control,clk);
control d(IFIDinst,Control_Signals,clk);
ALU a(IDEXredata1,AluSrcMux_to_B,IDEXctrl[10:7],ALUout_to_Mem_Addess,IDEXinst[10:6],clk);

Data_Memory t(EXMEMALUout, EXMEMredata2 , EXMEMctrl[0], EXMEMctrl[1], MEMWB_DataMem_Redata, clk);


Mux2to1 Mem_to_reg(MemtoregOut_writedata, MEMWBdatamem_address,MEMWBdatamem_redata ,MEMWBctrl[4]);
Mux2to1 ALUSRC(AluSrcMux_to_B, IDEXredata2, IDEXSextend_out,IDEXctrl[5]);
MuxRegDst RegDstMux (MuxRegDest, IDEX_rt, IDEX_rd,IDEXctrl[3]);



/*module RegisterFiles (clk,Instruction,IFIDinst,IDEX_ReadData1,IDEX_ReadData2,IDEXredata1,IDEXredata2,IDEX_SignEx_out,IDEXSextend_out,IDEX_rd,IDEX_rt,

EXMEM_ALUresult,EXMEMALUout,EXMEMredata2,EXMEM_MuxRegDest,

MEMWB_DataMem_Redata,MEMWB_DataMem_redata,MEMWBwrite_register,MEMWBdatamem_address);*/




RegisterFiles m(pc_to_InstMem,IDEX_PC,clk,Control_Signalsout,IDEXctrl,EXMEMctrl,MEMWBctrl,instruction,IDEXinst,IFIDinst,data1_to_A,  Reg_data2_to_AluSrcMux,IDEXredata1,IDEXredata2,Sextend_out,IDEXSextend_out,IDEX_rd,IDEX_rt,

ALUout_to_Mem_Addess,EXMEMALUout,EXMEMredata2,MuxRegDest,

MEMWB_DataMem_Redata,MEMWBdatamem_redata,MEMWBwrite_register,MEMWBdatamem_address);

always
begin	
#10 clk =~clk;
end

initial
begin
$monitor($time,"     %h     %b     %d     %d    ",pc_to_InstMem,IFIDinst,ALUout_to_Mem_Addess,MemtoregOut_writedata);
//$monitor($time," %b     %d   ",IFIDinst,ALUout_to_Mem_Addess);
clk<=0;
rst<=1;

#4
rst<=0;
end
endmodule
