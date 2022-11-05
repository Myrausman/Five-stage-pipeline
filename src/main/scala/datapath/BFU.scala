package datapath
import chisel3._
import chisel3.util._
class BranchForward extends Module {
  val io = IO(new Bundle {
  val ID_EX_REGRD = Input(UInt(5.W))
  val ID_EX_MEMRD = Input(UInt(1.W))
  val EX_MEM_REGRD = Input(UInt(5.W))
  val EX_MEM_MEMRD = Input(UInt(1.W))
  val MEM_WB_REGRD = Input(UInt(5.W))
  val MEM_WB_MEMRD = Input(UInt(1.W))
  val rs1_sel = Input(UInt(5.W))
  val rs2_sel = Input(UInt(5.W))
  val ctrl_branch = Input(UInt(1.W))
  val forward_rs1 = Output(UInt(3.W))
  val forward_rs2 = Output(UInt(3.W))
})

    io.forward_rs1 := "b000".U
    io.forward_rs2 := "b000".U

    // ALU Hazard

    when(io.ctrl_branch === 1.U && io.ID_EX_REGRD =/= "b00000".U && io.ID_EX_MEMRD =/= 1.U && (io.ID_EX_REGRD === io.rs1_sel) && (io.ID_EX_REGRD === io.rs2_sel)) {

    io.forward_rs1 := "b001".U
    io.forward_rs2 := "b001".U

    } .elsewhen(io.ctrl_branch === 1.U && io.ID_EX_REGRD =/= "b00000".U && io.ID_EX_MEMRD =/= 1.U && (io.ID_EX_REGRD === io.rs1_sel)) {

    io.forward_rs1 := "b001".U

    } .elsewhen(io.ctrl_branch === 1.U && io.ID_EX_REGRD =/= "b00000".U && io.ID_EX_MEMRD =/= 1.U && (io.ID_EX_REGRD === io.rs2_sel)) {

    io.forward_rs2 := "b001".U

    }

    //EX/MEM   
    when(io.ctrl_branch === 1.U && io.EX_MEM_REGRD =/= "b00000".U && io.EX_MEM_MEMRD =/= 1.U && ~((io.ctrl_branch === "b1".U) && (io.ID_EX_REGRD =/= "b00000".U) && (io.ID_EX_REGRD === io.rs1_sel) && (io.ID_EX_REGRD === io.rs2_sel)) && (io.EX_MEM_REGRD === io.rs1_sel) && (io.EX_MEM_REGRD === io.rs2_sel)) {

        io.forward_rs1 := "b010".U
        io.forward_rs2 := "b010".U

    } .elsewhen(io.ctrl_branch === 1.U && io.EX_MEM_REGRD =/= "b00000".U && io.EX_MEM_MEMRD =/= 1.U && ~((io.ctrl_branch === "b1".U) && (io.ID_EX_REGRD =/= "b00000".U) && (io.ID_EX_REGRD === io.rs2_sel)) && (io.EX_MEM_REGRD === io.rs2_sel)) {

        io.forward_rs2 := "b010".U

    } .elsewhen(io.ctrl_branch === 1.U && io.EX_MEM_REGRD =/= "b00000".U && io.EX_MEM_MEMRD =/= 1.U && ~((io.ctrl_branch === "b1".U) && (io.ID_EX_REGRD =/= "b00000".U) && (io.ID_EX_REGRD === io.rs1_sel)) && (io.EX_MEM_REGRD === io.rs1_sel)) {

        io.forward_rs1 := "b010".U

    } .elsewhen(io.ctrl_branch === 1.U && io.EX_MEM_REGRD =/= "b00000".U && io.EX_MEM_MEMRD === 1.U && ~((io.ctrl_branch === "b1".U) && (io.ID_EX_REGRD =/= "b00000".U) && (io.ID_EX_REGRD === io.rs1_sel) && (io.ID_EX_REGRD === io.rs2_sel)) && (io.EX_MEM_REGRD === io.rs1_sel) && (io.EX_MEM_REGRD === io.rs2_sel)) {

        // FOR Load instructions
        io.forward_rs1 := "b100".U
        io.forward_rs2 := "b100".U

    } 
    .elsewhen(io.ctrl_branch === 1.U && io.EX_MEM_REGRD =/= "b00000".U && io.EX_MEM_MEMRD === 1.U && ~((io.ctrl_branch === "b1".U) && (io.ID_EX_REGRD =/= "b00000".U) && (io.ID_EX_REGRD === io.rs2_sel)) && (io.EX_MEM_REGRD === io.rs2_sel)) {

        io.forward_rs2 := "b100".U

    } 

    .elsewhen(io.ctrl_branch === 1.U && io.EX_MEM_REGRD =/= "b00000".U && io.EX_MEM_MEMRD === 1.U && ~((io.ctrl_branch === "b1".U) && (io.ID_EX_REGRD =/= "b00000".U) && (io.ID_EX_REGRD === io.rs1_sel)) && (io.EX_MEM_REGRD === io.rs1_sel)) {

        io.forward_rs1 := "b100".U

    }
    // MEM/WB Hazard
    when(io.ctrl_branch === 1.U && io.MEM_WB_REGRD =/= "b00000".U && io.MEM_WB_MEMRD =/= 1.U &&
    // IF NOT ALU HAZARD
    ~((io.ctrl_branch === "b1".U) && (io.ID_EX_REGRD =/= "b00000".U) && (io.ID_EX_REGRD === io.rs1_sel) && (io.ID_EX_REGRD === io.rs2_sel)) &&
    // IF NOT EX/MEM HAZARD
    ~((io.ctrl_branch === "b1".U) && (io.EX_MEM_REGRD =/= "b00000".U) && (io.EX_MEM_REGRD === io.rs1_sel) && (io.EX_MEM_REGRD === io.rs2_sel)) &&
    (io.MEM_WB_REGRD === io.rs1_sel) && (io.MEM_WB_REGRD === io.rs2_sel)) {

    io.forward_rs1 := "b011".U
    io.forward_rs2 := "b011".U

    }
    .elsewhen(io.ctrl_branch === 1.U && io.MEM_WB_REGRD =/= "b00000".U && io.MEM_WB_MEMRD =/= 1.U &&
        // IF NOT ALU HAZARD
        ~((io.ctrl_branch === "b1".U) && (io.ID_EX_REGRD =/= "b00000".U) && (io.ID_EX_REGRD === io.rs2_sel)) &&
        // IF NOT EX/MEM HAZARD
        ~((io.ctrl_branch === "b1".U) && (io.EX_MEM_REGRD =/= "b00000".U) && (io.EX_MEM_REGRD === io.rs2_sel)) &&
        (io.MEM_WB_REGRD === io.rs2_sel)) {

    io.forward_rs2 := "b011".U

    }
    .elsewhen(io.ctrl_branch === 1.U && io.MEM_WB_REGRD =/= "b00000".U && io.MEM_WB_MEMRD =/= 1.U &&
        // IF NOT ALU HAZARD
        ~((io.ctrl_branch === "b1".U) && (io.ID_EX_REGRD =/= "b00000".U) && (io.ID_EX_REGRD === io.rs1_sel)) &&
        // IF NOT EX/MEM HAZARD
        ~((io.ctrl_branch === "b1".U) && (io.EX_MEM_REGRD =/= "b00000".U) && (io.EX_MEM_REGRD === io.rs1_sel)) &&
        (io.MEM_WB_REGRD === io.rs1_sel)) {

        io.forward_rs1 := "b011".U

    } .elsewhen(io.ctrl_branch === 1.U && io.MEM_WB_REGRD =/= "b00000".U && io.MEM_WB_MEMRD === 1.U &&
    // IF NOT ALU HAZARD
    ~((io.ctrl_branch === "b1".U) && (io.ID_EX_REGRD =/= "b00000".U) && (io.ID_EX_REGRD === io.rs1_sel) && (io.ID_EX_REGRD === io.rs2_sel)) &&
    // IF NOT EX/MEM HAZARD
    ~((io.ctrl_branch === "b1".U) && (io.EX_MEM_REGRD =/= "b00000".U) && (io.EX_MEM_REGRD === io.rs1_sel) && (io.EX_MEM_REGRD === io.rs2_sel)) &&
    (io.MEM_WB_REGRD === io.rs1_sel) && (io.MEM_WB_REGRD === io.rs2_sel)) {
    // FOR Load instructions
    io.forward_rs1 := "b101".U
    io.forward_rs2 := "b101".U

    }
    .elsewhen(io.ctrl_branch === 1.U && io.MEM_WB_REGRD =/= "b00000".U && io.MEM_WB_MEMRD === 1.U &&
        // IF NOT ALU HAZARD
        ~((io.ctrl_branch === "b1".U) && (io.ID_EX_REGRD =/= "b00000".U) && (io.ID_EX_REGRD === io.rs2_sel)) &&
        // IF NOT EX/MEM HAZARD
        ~((io.ctrl_branch === "b1".U) && (io.EX_MEM_REGRD =/= "b00000".U) && (io.EX_MEM_REGRD === io.rs2_sel)) &&
        (io.MEM_WB_REGRD === io.rs2_sel)) {

        io.forward_rs2 := "b101".U

    }
    .elsewhen(io.ctrl_branch === 1.U && io.MEM_WB_REGRD =/= "b00000".U && io.MEM_WB_MEMRD === 1.U &&
        // IF NOT ALU HAZARD
        ~((io.ctrl_branch === "b1".U) && (io.ID_EX_REGRD =/= "b00000".U) && (io.ID_EX_REGRD === io.rs1_sel)) &&
        // IF NOT EX/MEM HAZARD
        ~((io.ctrl_branch === "b1".U) && (io.EX_MEM_REGRD =/= "b00000".U) && (io.EX_MEM_REGRD === io.rs1_sel))&&
        (io.MEM_WB_REGRD === io.rs1_sel)) {

        io.forward_rs1 := "b101".U

    }
}