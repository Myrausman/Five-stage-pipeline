package datapath
import chisel3._
import chisel3.util._
import chisel3.core

class Top extends Module {
  val io = IO (new Bundle {
	val Reg_Out = Output(SInt(32.W))
	val addr= Output(UInt(10.W))
  })
  
//   class objects 
	val Alu = Module(new Alu())
	val Control = Module(new control_decode())
	val ImmediateGeneration = Module(new ImmdValGen())
	val AluControl = Module(new AluCntrl())
	val reg = Module(new  RegFile())
	val Memory = Module(new Memory())
	val Pc = Module(new PC())
	val Jalr = Module(new Jalr())
	val DataMemory = Module(new  data_Mem())
	// val Branch= Module(new BranchControl())
	// pipeline
	val IF_ID =Module(new IF_ID())
	val ID_EXE =Module(new ID_EXE())
	val EX_MEM =Module (new EXE_MEM())
	val MEM_WR =Module (new MEM_WR())

	//  pipeline fetch Module inputs
	IF_ID.io.pc_in :=Pc.io.pc
	IF_ID.io.pc4_in :=Pc.io.pc4
	IF_ID.io.ins_in := Memory.io.ReadData
	//  Decode execute  pipeline  inputs
	// control signals 
	ID_EXE.io.memWrite_in := Control.io.MemWrite
	ID_EXE.io.memRead_in := Control.io.MemRead
	ID_EXE.io.regWrite_in := Control.io.RegWrite
	ID_EXE.io.memToReg_in :=Control.io.MemtoReg
	ID_EXE.io.aluOp_in := Control.io.AluOp
	ID_EXE.io.operandAsel_in:= Control.io.OpA
	ID_EXE.io.operandBsel_in:= Control.io.OpB
	ID_EXE.io.br_en_in := Control.io.Branch
	ID_EXE.io.NextPc := Control.io.NextPc

	ID_EXE.io.pc_in := IF_ID.io.pc_out
	ID_EXE.io.pc4_in := IF_ID.io.pc4_out
	ID_EXE.io.func3_in := IF_ID.io.ins_out(14,12)
	ID_EXE.io.func7_in := IF_ID.io.ins_out(31)
	ID_EXE.io.rs1Ins_in:= IF_ID.io.ins_out(19,15)
	ID_EXE.io.rs2Ins_in := IF_ID.io.ins_out(24,20)
	ID_EXE.io.rd_in := IF_ID.io.ins_out(11,7)
	ID_EXE.io.operandA_in := reg.io.rs1
	ID_EXE.io.operandB_in:= reg.io.rs2

	when(Control.io.ExtSel === "b00".U) {
    // I-Type instruction
    	ID_EXE.io.imm := ImmediateGeneration.io.i_imm,
	}.elsewhen(Control.io.ExtSel === "b01".U) {
    // S-Type instruction
    	ID_EXE.io.imm := ImmediateGeneration.io.s_imm,
	}.elsewhen(Control.io.ExtSel === "b10".U) {
    // U-Type instruction
    	ID_EXE.io.imm := ImmediateGeneration.io.u_imm,
	}.otherwise {
    	ID_EXE.io.imm := 0.S(32.W)
	}

	// EXecute Memory stage
	EX_MEM.io.memWrite_in := ID_EXE.io.memWrite_out
	EX_MEM.io.memRead_in := ID_EXE.io.memRead_out
	EX_MEM.io.memToReg_in := ID_EXE.io.memToReg_out
	EX_MEM.io.regWrite_in := ID_EXE.io.regWrite_out
	EX_MEM.io.rd_in := ID_EXE.io.rd_out
	EX_MEM.io.aluOutput_in := Alu.io.out
	EX_MEM.io.rs2Sel_in:=ID_EXE.io.operandBsel_out
	EX_MEM.io.rs2:=ID_EXE.io.operandB_out
	// val strData_in = Input(SInt(32.W))
	// MEmwriteback stage
	MEM_WR.io.memToReg_in:= EX_MEM.io.memToReg_out
	MEM_WR.io.regWrite_in:=EX_MEM.io.regWrite_out
	MEM_WR.io.MemRead_in:=EX_MEM.io.memRead_out
	MEM_WR.io.rd_in :=EX_MEM.io.rd_out
	MEM_WR.io.dataOut_in :=DataMemory.io.instRead
	MEM_WR.io.aluOutput_in :=EX_MEM.io.aluOutput_out
	
	MEM_WR.io.memWrite_in:=EX_MEM.io.memWrite_out

	// wiring
	// memory and pc
	
	Pc.io.input:=Pc.io.pc4
	Memory.io.WriteAddr := Pc.io.pc(11,2)
	io.addr:=Memory.io.WriteAddr
	Control.io.Op_code:= IF_ID.io.ins_out(6,0) // opcode 7 bits , 
	// reg 
	reg.io.RegWrite := MEM_WR.io.regWrite_out
	reg.io.reg1:= IF_ID.io.ins_out(19,15)
	reg.io.reg2:= IF_ID.io.ins_out(24,20)
	reg.io.rd:=MEM_WR.io.rd_out

	// instruction
	ImmediateGeneration.io.instr:=IF_ID.io.ins_out
	ImmediateGeneration.io.PC:= IF_ID.io.pc_out
	// jalr
	Jalr.io.rs1:= reg.io.rs1
	Jalr.io.immd_se:=ImmediateGeneration.io.i_imm
	// Alucntrl
	AluControl.io.AluOp:= ID_EXE.io.aluOp_out
	AluControl.io.funct3:= ID_EXE.io.func3_out
	AluControl.io.funct7:= ID_EXE.io.func7_out
	// // Branch
	// Branch.io.alucnt:=AluControl.io.alucnt
	// Branch.io.in1:=MuxCase(0.S,Array(
	// 	(Control.io.OpA === 0.U) -> reg.io.rs1,
	// 	(Control.io.OpA === 1.U ) -> (Pc.io.pc4).asSInt,
	// 	(Control.io.OpA === 2.U )-> (Pc.io.pc).asSInt,
	// 	(Control.io.OpA === 3.U ) -> reg.io.rs1
	// ))
	// Branch.io.in2:= MuxCase(0.S,Array(
	// 	(Control.io.ExtSel === 0.U && Control.io.OpB ===1.U) -> ImmediateGeneration.io.i_imm,
	// 	(Control.io.ExtSel === 1.U &&  Control.io.OpB === 1.U ) -> ImmediateGeneration.io.s_imm,
	// 	(Control.io.ExtSel === 2.U && Control.io.OpB === 1.U )-> ImmediateGeneration.io.u_imm,
	// 	(Control.io.OpB === 0.U ) -> reg.io.rs2))
	// Branch.io.Branch:=Control.io.Branch
	

	// Alu
	// mux opA
	Alu.io.in1:= MuxCase(0.S,Array(
		(ID_EXE.io.operandAsel_in === 0.U) -> ID_EXE.io.operandA_out,
		(ID_EXE.io.operandAsel_in  === 1.U ) -> (ID_EXE.io.pc4_out).asSInt,
		(ID_EXE.io.operandAsel_in === 2.U )-> (ID_EXE.io.pc_out).asSInt,
		(Control.io.OpA === 3.U ) -> ID_EXE.io.operandA_out
	))
	// mux opb
	when(ID_EXE.io.operandBsel_out === "b1".U) {
		Alu.io.in2:= ID_EXE.io.imm_out
	} .otherwise {
		Alu.io.in2:= ID_EXE.io.operandB_out
	}
	Alu.io.alucnt:=AluControl.io.alucnt
	// datamemory
	DataMemory.io.Addr:=(EX_MEM.io.aluOutput_out(9,2)).asUInt
	DataMemory.io.Data:= EX_MEM.io.rs2_out
	DataMemory.io.MemWrite:= EX_MEM.io.memWrite_out
	DataMemory.io.MemRead:= EX_MEM.io.memRead_out
	// mem to reg
	reg.io.WriteData:= MuxCase(0.S,Array(
		(MEM_WR.io.memToReg_out === 0.U) -> MEM_WR.io.aluOutput_out,
		// testing
		(MEM_WR.io.memToReg_out  === 1.U) -> DataMemory.io.instRead
		))
	
	Pc.io.input := MuxCase(0.U,Array(
		(Control.io.NextPc === 0.U) -> Pc.io.pc4,
		// (Control.io.NextPc === 1.U) ->  Mux(Branch.io.br_taken,(ImmediateGeneration.io.sb_imm).asUInt,Pc.io.pc4),
		(Control.io.NextPc === 2.U) -> (ImmediateGeneration.io.uj_imm).asUInt,
		(Control.io.NextPc === 3.U) -> (Jalr.io.jalout).asUInt()
	)) 
	io.Reg_Out:=0.S
}
