package datapath
import chisel3._
import org.scalatest._
import chiseltest._
class five_cycle_test extends FreeSpec with ChiselScalatestTester{
    "5 stage cycle risc v" in {
        test(new Top){ c =>
        // c.io.RegWrite.poke(1.U)
        c.clock.step(100)
    }
}}