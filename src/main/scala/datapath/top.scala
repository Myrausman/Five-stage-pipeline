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
	val forwardUnit=Module(new ForwardUnit())
	val hazardDetection = Module(new HazardDetection())
	val branchLogic =Module(new BranchLogic())
	val decodeForward =Module(new DecodeForwardUnit())
	val structuralDetector =Module(new StructuralDetector())
	// wiring
	//-----------------------------------------------IF------------------
	//  pc
	Pc.io.input:=Pc.io.pc4
	// instructions 
	Memory.io.WriteAddr := Pc.io.pc(11,2)
	val inst = Memory.io.ReadData
	io.addr:=Memory.io.WriteAddr
	

	//  if-id inputs
	IF_ID.io.pc_in :=Pc.io.pc
	IF_ID.io.pc4_in :=Pc.io.pc4
	IF_ID.io.ins_in := inst

	// opcode
	Control.io.Op_code:= IF_ID.io.ins_out(6,0) // opcode 7 bits , 
	// reg 
	
	reg.io.reg1:= IF_ID.io.ins_out(19,15)
	reg.io.reg2:= IF_ID.io.ins_out(24,20)
	// immediate generation
	ImmediateGeneration.io.instr:=IF_ID.io.ins_out
	ImmediateGeneration.io.PC:= IF_ID.io.pc_out
	// immediate generation
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

		// Alucntrl
	AluControl.io.AluOp:= ID_EXE.io.aluOp_out
	AluControl.io.funct3:= ID_EXE.io.func3_out
	AluControl.io.funct7:= ID_EXE.io.func7_out
	
	Alu.io.alucnt:=AluControl.io.alucnt

		// Initializing Branch Forward Unit
	decodeForward.io.ID_EX_REGRD := ID_EXE.io.rd_out
	decodeForward.io.ID_EX_MEMRD := ID_EXE.io.memRead_out
	decodeForward.io.EX_MEM_REGRD := EX_MEM.io.rd_out
	decodeForward.io.MEM_WB_REGRD := MEM_WR.io.rd_out
	decodeForward.io.EX_MEM_MEMRD := EX_MEM.io.memRead_out
	decodeForward.io.MEM_WB_MEMRD := MEM_WR.io.MemRead_out
	decodeForward.io.rs1_sel := IF_ID.io.ins_out(19, 15)
	decodeForward.io.rs2_sel := IF_ID.io.ins_out(24, 20)
	decodeForward.io.ctrl_branch := Control.io.Branch
	
	// FOR REGISTER RS1 in BRANCH LOGIC UNIT
	when(decodeForward.io.forward_rs1 === "b0000".U) {
	// No hazard just use register file data
	branchLogic.io.in_rs1 := reg.io.rs1
	Jalr.io.rs1 :=  reg.io.rs1
	} .elsewhen(decodeForward.io.forward_rs1 === "b0001".U) {
	// hazard in alu stage forward data from alu output
	branchLogic.io.in_rs1 := Alu.io.out
	Jalr.io.rs1 := reg.io.rs1
	} .elsewhen(decodeForward.io.forward_rs1 === "b0010".U) {
	// hazard in EX/MEM stage forward data from EX/MEM.alu_output
	branchLogic.io.in_rs1 := EX_MEM.io.aluOutput_out
	Jalr.io.rs1 := reg.io.rs1
	} .elsewhen(decodeForward.io.forward_rs1 === "b0011".U) {
	// hazard in MEM/WB stage forward data from register file write data which will have correct data from the MEM/WB mux
	branchLogic.io.in_rs1 := reg.io.WriteData
	Jalr.io.rs1 := reg.io.rs1
	} .elsewhen(decodeForward.io.forward_rs1 === "b0100".U) {
	// hazard in EX/MEM stage and load type instruction so forwarding from data memory data output instead of EX/MEM.alu_output
	branchLogic.io.in_rs1 := DataMemory.io.instRead
	Jalr.io.rs1 := reg.io.rs1
	} .elsewhen(decodeForward.io.forward_rs1 === "b0101".U) {
	// hazard in MEM/WB stage and load type instruction so forwarding from register file write data which will have the correct output from the mux
	branchLogic.io.in_rs1 := reg.io.WriteData
	Jalr.io.rs1 := reg.io.rs1
	}
		.elsewhen(decodeForward.io.forward_rs1 === "b0110".U) {
		// hazard in alu stage forward data from alu output
		Jalr.io.rs1 := Alu.io.out
		branchLogic.io.in_rs1 := reg.io.rs1
	} .elsewhen(decodeForward.io.forward_rs1 === "b0111".U) {
		// hazard in EX/MEM stage forward data from EX/MEM.alu_output
		Jalr.io.rs1 := EX_MEM.io.aluOutput_out
		branchLogic.io.in_rs1 := reg.io.rs1
	} .elsewhen(decodeForward.io.forward_rs1 === "b1000".U) {
		// hazard in MEM/WB stage forward data from register file write data which will have correct data from the MEM/WB mux
		Jalr.io.rs1 := reg.io.WriteData
		branchLogic.io.in_rs1 := reg.io.rs1
	} .elsewhen(decodeForward.io.forward_rs1 === "b1001".U) {
		// hazard in EX/MEM stage and load type instruction so forwarding from data memory data output instead of EX/MEM.alu_output
		Jalr.io.rs1 := DataMemory.io.instRead
		branchLogic.io.in_rs1 := reg.io.rs1
	} .elsewhen(decodeForward.io.forward_rs1 === "b1010".U) {
		// hazard in MEM/WB stage and load type instruction so forwarding from register file write data which will have the correct output from the mux
		Jalr.io.rs1 := reg.io.WriteData
		branchLogic.io.in_rs1 := reg.io.rs1
	}
	.otherwise {
		branchLogic.io.in_rs1 := reg.io.rs1
		Jalr.io.rs1 := reg.io.rs1
	}

		// FOR REGISTER RS2 in BRANCH LOGIC UNIT
	when(decodeForward.io.forward_rs2 === "b0000".U) {
	// No hazard just use register file data
	branchLogic.io.in_rs2 := reg.io.rs2
	} .elsewhen(decodeForward.io.forward_rs2 === "b0001".U) {
	// hazard in alu stage forward data from alu output
	branchLogic.io.in_rs2 := Alu.io.out
	} .elsewhen(decodeForward.io.forward_rs2 === "b0010".U) {
	// hazard in EX/MEM stage forward data from EX/MEM.alu_output
	branchLogic.io.in_rs2 := EX_MEM.io.aluOutput_out
	} .elsewhen(decodeForward.io.forward_rs2 === "b0011".U) {
	// hazard in MEM/WB stage forward data from register file write data which will have correct data from the MEM/WB mux
	branchLogic.io.in_rs2 := reg.io.WriteData
	} .elsewhen(decodeForward.io.forward_rs2 === "b0100".U) {
	// hazard in EX/MEM stage and load type instruction so forwarding from data memory data output instead of EX/MEM.alu_output
	branchLogic.io.in_rs2 := DataMemory.io.instRead
	} .elsewhen(decodeForward.io.forward_rs2 === "b0101".U) {
	// hazard in MEM/WB stage and load type instruction so forwarding from register file write data which will have the correct output from the mux
	branchLogic.io.in_rs2 := reg.io.WriteData
	}
	.otherwise {
		branchLogic.io.in_rs2 := reg.io.rs2
	}

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


	



	

	when(hazardDetection.io.inst_forward === "b1".U) {
		IF_ID.io.ins_in  := hazardDetection.io.inst_out
		IF_ID.io.pc_in := hazardDetection.io.current_pc_out
	} .otherwise {
		IF_ID.io.ins_in := Memory.io.ReadData
	}
	
	// jalr
	Jalr.io.rs1:= reg.io.rs1
	Jalr.io.immd_se:=ImmediateGeneration.io.i_imm
	when(hazardDetection.io.pc_forward === "b1".U) {
  		Pc.io.input := hazardDetection.io.pc_out
	}.otherwise {
    when(Control.io.NextPc === "b01".U) {
      	when(branchLogic.io.output === 1.U && Control.io.Branch === 1.U) {
        Pc.io.input := ImmediateGeneration.io.sb_imm.asUInt
        IF_ID.io.pc_in := 0.U
        IF_ID.io.pc4_in := 0.U
        IF_ID.io.ins_in := 0.U
    	}.otherwise {
        Pc.io.input := Pc.io.pc4
		}
	}.elsewhen(Control.io.NextPc === "b10".U) {
		Pc.io.input := ImmediateGeneration.io.uj_imm.asUInt
		IF_ID.io.pc_in := 0.U
		IF_ID.io.pc4_in := 0.U
		IF_ID.io.ins_in := 0.U
	}.elsewhen(Control.io.NextPc === "b11".U) {
		Pc.io.input := Jalr.io.jalout.asUInt
		IF_ID.io.pc_in := 0.U
		IF_ID.io.pc4_in := 0.U
		IF_ID.io.ins_in := 0.U
    
    }.otherwise {
      Pc.io.input := Pc.io.pc4
    }
	}
	when(hazardDetection.io.ctrl_forward === "b1".U) {
    ID_EXE.io.memWrite_in := 0.U
	ID_EXE.io.memRead_in := 0.U
    ID_EXE.io.br_en_in := 0.U
    ID_EXE.io.regWrite_in := 0.U
    ID_EXE.io.memToReg_in := 0.U
    ID_EXE.io.aluOp_in := 0.U
    ID_EXE.io.operandAsel_in := 0.U
    ID_EXE.io.operandBsel_in := 0.U
    ID_EXE.io.NextPc := 0.U

	}.otherwise {
    ID_EXE.io.memWrite_in := Control.io.MemWrite
    ID_EXE.io.memRead_in := Control.io.MemRead
    ID_EXE.io.br_en_in := Control.io.Branch
    ID_EXE.io.regWrite_in := Control.io.RegWrite 
    ID_EXE.io.memToReg_in := Control.io.MemtoReg
    ID_EXE.io.aluOp_in := Control.io.AluOp
    ID_EXE.io.operandAsel_in := Control.io.OpA
    ID_EXE.io.operandBsel_in := Control.io.OpB 
    ID_EXE.io.NextPc := Control.io.NextPc 

	}
	
	
	forwardUnit.io.EX_MEM_REGRD := EX_MEM.io.rd_out 
	forwardUnit.io.MEM_WB_REGRD := MEM_WR.io.rd_out
	forwardUnit.io.ID_EX_REGRS1 := ID_EXE.io.rs1Ins_out
	forwardUnit.io.ID_EX_REGRS2 := ID_EXE.io.rs2Ins_out
	forwardUnit.io.EX_MEM_REGWR := EX_MEM.io.regWrite_out 
	forwardUnit.io.MEM_WB_REGWR := MEM_WR.io.regWrite_out

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
	


	
	
	
	Pc.io.input := MuxCase(0.U,Array(
		(Control.io.NextPc === 0.U) -> Pc.io.pc4,
		// (Control.io.NextPc === 1.U) ->  Mux(Branch.io.br_taken,(ImmediateGeneration.io.sb_imm).asUInt,Pc.io.pc4),
		(Control.io.NextPc === 2.U) -> (ImmediateGeneration.io.uj_imm).asUInt,
		(Control.io.NextPc === 3.U) -> (Jalr.io.jalout).asUInt()
	)) 
	io.Reg_Out:=0.S
	branchLogic.io.in_rs1 := reg.io.rs1
	branchLogic.io.in_rs2 := reg.io.rs2
	branchLogic.io.in_func3 := IF_ID.io.ins_out(14,12)

	
	structuralDetector.io.rs1_sel := IF_ID.io.ins_out(19, 15)
	structuralDetector.io.rs2_sel := IF_ID.io.ins_out(24, 20)
	structuralDetector.io.MEM_WB_REGRD := MEM_WR.io.rd_out
	structuralDetector.io.MEM_WB_regWr := MEM_WR.io.regWrite_out
	// FOR RS1
	when(structuralDetector.io.fwd_rs1 === 1.U) {
	ID_EXE.io.operandA_in := reg.io.WriteData
	} .otherwise {
	ID_EXE.io.operandA_in := reg.io.rs1
	}
	// FOR RS2
	when(structuralDetector.io.fwd_rs2 === 1.U) {
		ID_EXE.io.operandB_in := reg.io.WriteData
	} .otherwise {
		ID_EXE.io.operandB_in := reg.io.rs2
	}
	// hazard detection
	hazardDetection.io.IF_ID_INST := IF_ID.io.ins_out
	hazardDetection.io.ID_EX_MEMREAD := ID_EXE.io.memRead_out
	hazardDetection.io.ID_EX_REGRD := ID_EXE.io.rd_out
	hazardDetection.io.pc_in := IF_ID.io.pc4_out
	hazardDetection.io.current_pc := IF_ID.io.pc_out

	// Alu
	// mux opA
	when (ID_EXE.io.operandAsel_out === "b10".U){
		Alu.io.in1:= ID_EXE.io.pc4_out.asSInt
  	}.otherwise {
		Alu.io.in1:= MuxCase(0.S,Array(
		(forwardUnit.io.forward_a === 0.U) -> ID_EXE.io.operandA_out,
		(forwardUnit.io.forward_a ===  1.U ) -> EX_MEM.io.aluOutput_out,
		(forwardUnit.io.forward_a === 2.U )-> reg.io.WriteData,
		(forwardUnit.io.forward_a=== 3.U ) -> ID_EXE.io.operandA_out
	))}

	// Controlling Operand B for ALU
	when(ID_EXE.io.operandBsel_out=== 1.U) {
		Alu.io.in2 := ID_EXE.io.imm_out

		when(forwardUnit.io.forward_b === 0.U) {
		EX_MEM.io.rs2 := ID_EXE.io.operandB_out
		} .elsewhen(forwardUnit.io.forward_b === 1.U) {
		EX_MEM.io.rs2 := EX_MEM.io.aluOutput_out
		} .elsewhen(forwardUnit.io.forward_b === 2.U) {
		EX_MEM.io.rs2  := reg.io.WriteData
		} .otherwise {
		EX_MEM.io.rs2 := ID_EXE.io.operandB_out
		}
	}.otherwise {
		when(forwardUnit.io.forward_b === "b00".U) {
			Alu.io.in2:= ID_EXE.io.operandB_out
			EX_MEM.io.rs2 := ID_EXE.io.operandB_out
		} .elsewhen(forwardUnit.io.forward_b === "b01".U) {
			Alu.io.in2:= EX_MEM.io.aluOutput_out
			EX_MEM.io.rs2 := EX_MEM.io.aluOutput_out
		} .elsewhen(forwardUnit.io.forward_b === "b10".U) {
			Alu.io.in2:= reg.io.WriteData
			EX_MEM.io.rs2 := reg.io.WriteData
		} .otherwise {
			Alu.io.in2:= ID_EXE.io.operandB_out
			EX_MEM.io.rs2  := ID_EXE.io.operandB_out
		}}

	// EXecute Memory stage
	EX_MEM.io.memWrite_in := ID_EXE.io.memWrite_out
	EX_MEM.io.memRead_in := ID_EXE.io.memRead_out
	EX_MEM.io.memToReg_in := ID_EXE.io.memToReg_out
	EX_MEM.io.regWrite_in := ID_EXE.io.regWrite_out
	EX_MEM.io.rd_in := ID_EXE.io.rd_out
	EX_MEM.io.aluOutput_in := Alu.io.out
	// EX_MEM.io.rs2Sel_in:=ID_EXE.io.operandBsel_out
	EX_MEM.io.rs2:=ID_EXE.io.operandB_out
	// val strData_in = Input(SInt(32.W))
	
	// datamemory
	DataMemory.io.Addr:=(EX_MEM.io.aluOutput_out(9,2)).asUInt
	DataMemory.io.Data:= EX_MEM.io.rs2_out
	DataMemory.io.MemWrite:= EX_MEM.io.memWrite_out
	DataMemory.io.MemRead:= EX_MEM.io.memRead_out

	// MEmwriteback stage
	MEM_WR.io.memToReg_in:= EX_MEM.io.memToReg_out
	MEM_WR.io.regWrite_in:=EX_MEM.io.regWrite_out
	MEM_WR.io.MemRead_in:=EX_MEM.io.memRead_out
	MEM_WR.io.rd_in :=EX_MEM.io.rd_out
	MEM_WR.io.dataOut_in :=DataMemory.io.instRead
	MEM_WR.io.aluOutput_in :=EX_MEM.io.aluOutput_out
	MEM_WR.io.memWrite_in:=EX_MEM.io.memWrite_out

	// mem to reg
	reg.io.WriteData:= MuxCase(0.S,Array(
		(MEM_WR.io.memToReg_out === 0.U) -> MEM_WR.io.aluOutput_out,
		// testing
		(MEM_WR.io.memToReg_out  === 1.U) -> DataMemory.io.instRead
		))
	reg.io.RegWrite := MEM_WR.io.regWrite_out
	reg.io.rd:=MEM_WR.io.rd_out
}
