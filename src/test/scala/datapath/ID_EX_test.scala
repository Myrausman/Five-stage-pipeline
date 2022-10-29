// package datapath
// import chisel3._
// import org.scalatest._
// import chiseltest._
// class id_exe_test extends FreeSpec with ChiselScalatestTester{
//     "ID_EX test" in {
//         test(new ID_EXE()){ c =>
//         c.io.pc_in.poke(1.U)
// 		c.io.pc4_in.poke(2.U)
// 		c.io.ins_in.poke(3.U)
// 		c.io.pc_out.expect(1.U)
// 		c.io.ins_out.expect(2.U)
// 		c.io.pc4_out.expect(3.U)
//         }
//     }
// }