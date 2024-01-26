// `timescale 1ns / 1ps
// rename for your .dat-file
`define DATFILE "memfile.dat"

// --- PipeV start ---
module PipeV (clk, reset, memwrite, writedata, dataaddr, pc, reg_5);
    input clk, reset;
    output memwrite;
    output [31:0] writedata, dataaddr, pc, reg_5;
    reg [2:0] Tstep_Q, Tstep_D;
    parameter
        T_IF = 3'b000,
        T_ID = 3'b001,
        T_EX = 3'b010,
        T_MEM = 3'b011,
        T_WB = 3'b100;

    // use pll as a clock when using Intel Quartus Prime

    // -- for the output --
    assign memwrite = EX_ctrlsout[10];
    assign writedata = EX_writedata;
    assign dataaddr = EX_aluout;
    assign pc = IF_pc;
    
    // -- reg for IF --
    reg IF_pcsrc, IF_j_op;
    reg IF_pcflag; // Disable this line when using Pipeline
    reg [31:0] IF_pcbranch;
    wire [31:0] IF_pc;
    wire [31:0] IF_pcplus4, IF_instr;

    // -- reg for ID --
    reg ID_regwrite;
    reg [4:0] ID_writereg;
    reg [31:0] ID_instr, ID_writedata, ID_pcin, ID_pcplus4in;
    wire [3:0] ID_alucontrol;
    wire [4:0] ID_dstreg;
    wire [12:0] ID_controls;
    wire [31:0] ID_rsrca, ID_rsrcb, ID_immextv, ID_pcout, ID_pcplus4out;

    // -- reg for EX --
    reg [3:0] EX_alucontrol;
    reg [4:0] EX_dstregin;
    reg [12:0] EX_controls;
    reg [31:0] EX_pcplus4in, EX_pc, EX_rsrca, EX_rsrcb, EX_immextv;
    wire EX_btaken;
    wire [4:0] EX_dstregout;
    wire [12:0] EX_ctrlsout;
    wire [31:0] EX_pcplus4out, EX_pcbranch, EX_aluout, EX_writedata;

    // -- reg for MEM --
    reg MEM_btakenin;
    reg [4:0] MEM_dstregin;
    reg [12:0] MEM_controls;
    reg [31:0] MEM_pcplus4in, MEM_pc, MEM_dataaddr, MEM_writedata;
    wire MEM_btakenout;
    wire [4:0] MEM_dstregout;
    wire [12:0] MEM_ctrlsout;
    wire [31:0] MEM_pcplus4out, MEM_aluout, MEM_readdata;

    // -- reg for WB --
    reg WB_btaken;
    reg [4:0] WB_dstreg;
    reg [12:0] WB_controls;
    reg [31:0] WB_pcplus4, WB_aluout, WB_readdata;
    wire WB_pcsrc, WB_j_op, WB_regwrite;
    wire [4:0] WB_writereg;
    wire [31:0] WB_writedata;

    // -- modules --
    // IF IF(clk, reset, IF_pcsrc, IF_j_op, IF_pcbranch, IF_pc, IF_instr);
    IF IF(clk, reset, IF_pcsrc, IF_j_op, IF_pcflag, IF_pcbranch, IF_pc, IF_pcplus4, IF_instr); // !!!
    // ID ID(clk, ID_regwrite, ID_writereg, ID_instr, ID_writedata, ID_pcin, ID_pcplus4in,
    //     ID_alucontrol, ID_dstreg, ID_controls, ID_rsrca, ID_rsrcb, ID_immextv, ID_pcout, ID_pcplus4out);
    ID ID(clk, ID_regwrite, ID_writereg, ID_instr, ID_writedata, ID_pcin, ID_pcplus4in,
        ID_alucontrol, ID_dstreg, ID_controls, ID_rsrca, ID_rsrcb, ID_immextv, ID_pcout, ID_pcplus4out, reg_5); // !!!
    EX EX(EX_alucontrol, EX_dstregin, EX_controls, EX_pcplus4in, EX_pc, EX_rsrca, EX_rsrcb, EX_immextv,
        EX_btaken, EX_dstregout, EX_ctrlsout, EX_pcplus4out, EX_pcbranch, EX_aluout, EX_writedata);
    MEM MEM (clk, MEM_btakenin, MEM_dstregin, MEM_controls, MEM_pcplus4in, MEM_pc, MEM_dataaddr, MEM_writedata,
            MEM_btakenout, MEM_dstregout, MEM_ctrlsout, MEM_pcplus4out, MEM_aluout, MEM_readdata);
    WB WB (WB_btaken, WB_dstreg, WB_controls, WB_aluout, WB_readdata, WB_pcplus4,
            WB_pcsrc, WB_j_op, WB_regwrite, WB_writereg, WB_writedata);
    
    // -- initialization --
    initial
        begin
                IF_pcsrc        <= 1'b0;
                IF_j_op         <= 1'b0;
                IF_pcflag       <= 1'b0; // !!!
                IF_pcbranch     <= 32'b0;
                ID_regwrite     <= 1'b0;
                ID_writereg     <= 5'b0;
                ID_instr        <= 32'b0;
                ID_writedata    <= 32'b0;
                ID_pcin         <= 32'b0;
                ID_pcplus4in    <= 32'b0;
                EX_alucontrol   <= 4'b0;
                EX_dstregin     <= 5'b0;
                EX_controls     <= 13'b0;
                EX_pcplus4in    <= 32'b0;
                EX_pc           <= 32'b0;
                EX_rsrca        <= 32'b0;
                EX_rsrcb        <= 32'b0;
                EX_immextv      <= 32'b0;
                MEM_btakenin    <= 1'b0;
                MEM_dstregin    <= 5'b0;
                MEM_controls    <= 13'b0;
                MEM_pcplus4in   <= 32'b0;
                MEM_pc          <= 32'b0;
                MEM_dataaddr    <= 32'b0;
                MEM_writedata   <= 32'b0;
                WB_btaken       <= 1'b0;
                WB_dstreg       <= 5'b0;
                WB_controls     <= 13'b0;
                WB_pcplus4      <= 32'b0;
                WB_aluout       <= 32'b0;
                WB_readdata     <= 32'b0;
        end

    // -- State Machine --
    always @ (posedge clk) 
        begin
            if (reset)
                begin
                    Tstep_Q <= T_IF;
                end
            else
                begin
                    // Control FSM state table
                    case (Tstep_Q)
                        T_IF:
                            begin
                                IF_pcsrc <= WB_pcsrc;
                                IF_j_op <= WB_j_op;
                                IF_pcbranch <= EX_pcbranch;
                                IF_pcflag <= 1'b0; // !!!
                                Tstep_D = T_ID;
                            end
                        T_ID:
                            begin
                                ID_pcin <= IF_pc;
                                ID_pcplus4in <= IF_pcplus4;
                                ID_instr <= IF_instr;
                                ID_writereg <= WB_writereg;
                                ID_writedata <= WB_writedata;
                                ID_regwrite <= WB_regwrite;
                                Tstep_D = T_EX;
                            end
                        T_EX:
                            begin
                                EX_pcplus4in <= ID_pcplus4out;
                                EX_pc <= ID_pcout;
                                EX_rsrca <= ID_rsrca;
                                EX_rsrcb <= ID_rsrcb;
                                EX_immextv <= ID_immextv;
                                EX_alucontrol <= ID_alucontrol;
                                EX_controls <= ID_controls;
                                EX_dstregin <= ID_dstreg;
                                Tstep_D = T_MEM;
                            end
                        T_MEM:
                            begin
                                MEM_pcplus4in <= EX_pcplus4out;
                                MEM_pc <= EX_pc;
                                MEM_btakenin <= EX_btaken;
                                MEM_dataaddr <= EX_aluout;
                                MEM_writedata <= EX_writedata;
                                MEM_controls <= EX_ctrlsout;
                                MEM_dstregin <= EX_dstregout;
                                Tstep_D = T_WB;
                            end
                        T_WB:
                            begin
                                WB_pcplus4 <= MEM_pcplus4out;
                                WB_btaken <= MEM_btakenout;
                                WB_aluout <= MEM_aluout;
                                WB_readdata <= MEM_readdata;
                                WB_controls <= MEM_ctrlsout;
                                WB_dstreg <= MEM_dstregout;
                                IF_pcflag <= 1'b1; // !!!
                                Tstep_D = T_IF;
                            end
                    endcase
                    Tstep_Q <= Tstep_D;
                end
        end
endmodule
// --- PipeV end ---

// --- IF(Instruction Fetch) start ---
module IF (clk, reset, pcsrc, j_op, pcflag, pcbranch, pc, pcplus4, instr);
    input clk, reset, pcsrc, j_op;
    input pcflag; // Disable this line when using Pipeline
    input [31:0] pcbranch;
    output reg [31:0] pc;
    output [31:0] pcplus4, instr;

    wire [31:0] pcnext;

    // assign pcplus4 = pc + 32'b100; // Enable this line when using Pipeline
    assign pcplus4 = pcflag ? (pc + 32'b100) : pc; // Disable this line when using Pipeline
    assign pcnext = (pcsrc|j_op) ? pcbranch : pcplus4;
    always @ (negedge clk)
        if (reset) pc <= 32'b0;
        else pc <= pcnext;
    // imem imem (pc[7:2], instr);
    assign instr = 32'h00178793; // addi a5,a5,1
endmodule

/*
module imem (pc, instr);
    input [5:0] pc;
    output [31:0] instr;

    reg [31:0] RAM [0:63];
    
    initial
        begin
	        $readmemh (`DATFILE, RAM);
        end
    assign instr = RAM[pc];
endmodule
*/
// --- IF(Instruction Fetch) end ---

// --- ID(Instruction Decode) start ---
module ID (clk, regwrite, writereg, instr, writedata, pcin, pcplus4in,
        alucontrol, dstreg, controls, rsrca, rsrcb, immextv, pcout, pcplus4out, reg_5);
    input clk, regwrite;
    input [4:0] writereg;
    input [31:0] instr, writedata, pcin, pcplus4in;
    output [3:0] alucontrol;
    output [4:0] dstreg;
    output [12:0] controls;
    output [31:0] rsrca, rsrcb, immextv, pcout, pcplus4out;
    output [31:0] reg_5; // for checking the output

    assign reg_5 = regfile[15];
    
    // 32x32 registers
    reg [31:0] regfile [31:0];
    integer i;
    initial
        begin
            for(i = 0; i < 32; i = i + 1) regfile[i] = 32'b0;
        end

    // instr[19:15]=rs1, instr[24:20]=rs2, instr[11:7]=rd
    assign rsrca = (instr[19:15] != 0) ? regfile[instr[19:15]] : 0;
    assign rsrcb = (instr[24:20] != 0) ? regfile[instr[24:20]] : 0;
    always @ (negedge clk)
        if (regwrite) regfile[writereg] <= writedata;
    assign dstreg = instr[11:7];
    assign pcplus4out = pcplus4in;
    assign pcout = pcin;
    maindec maindec (instr[6:0], controls);
    immext immext (controls[7:3], instr, immextv);
    aludec aludec (controls[8], instr[14:12], controls[2:0], instr[31:25], alucontrol);
endmodule

module maindec (opcode, controls);
    input [6:0] opcode;
    output reg [12:0] controls;
        
    // controls = {regwrite, alusrc, memwrite, memtoreg, r_op, immtype[4:0], aluop[2:0]}
    // immtype[4:0] = {i_op, s_op, b_op, u_op, j_op}
    always @ (*)
        case (opcode)
            7'b0110011: controls <= 13'b1000100000100; // R-type
            7'b0010011: controls <= 13'b1100010000101; // I-type
            7'b1100011: controls <= 13'b0000000100110; // B-type
            7'b0000011: controls <= 13'b1101010000111; // LW
            7'b0100011: controls <= 13'b0110001000111; // SW
            7'b0110111: controls <= 13'b1100000010011; // LUI
            7'b1101111: controls <= 13'b1000000001000; // JAL
            // jalr, auipc, ecall, ebreak are not applied.
            default:    controls <= 13'bxxxxxxxxxxxxx;
        endcase
endmodule

module immext (immtype, instr, immextv);
    input [4:0] immtype;
    input [31:0] instr;
    output reg [31:0] immextv;
    
    always @ (*)
        case (immtype)
            5'b10000: immextv <= { {20{instr[31]}}, instr[31:20] }; // I-type
            5'b01000: immextv <= { {20{instr[31]}}, instr[31:25], instr[11:7] }; // S-type
            5'b00100: immextv <= { {19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0 }; // B-type
            5'b00010: immextv <= { instr[31:12], 12'b0 }; // U-type
            5'b00001: immextv <= { {11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0 }; // J-type
            default: immextv <= 'hx;
        endcase // case (immtpe)
endmodule

module aludec (r_op, funct3, aluop, funct7, alucontrol);
    input r_op;
    input [2:0] funct3, aluop;
    input [6:0]  funct7;
    output reg [3:0] alucontrol;

    wire exf7;
    assign exf7 = (r_op & ((funct7 == 7'b0100000 ) ? 1'b1 : ((funct7 == 7'b0000000 ) ? 1'b0 : 1'bx)));
    
    always @ (*)
        case (aluop)
            3'b000: alucontrol <= 4'b0000; // addition, JAL
            3'b011: alucontrol <= 4'b0110; // or, LUI
            3'b100: alucontrol <= {exf7, funct3}; // R-type
            3'b101: alucontrol <= {1'b0, funct3}; // I-type
            3'b110: alucontrol <= {1'b0, funct3}; // B-type
            3'b111: case (funct3) // L-type or S-type
                        3'b010: alucontrol <= 4'b0000; // LW or SW
                        default: alucontrol <= 4'bxxxx;
                    endcase
            default: alucontrol <= 4'bxxxx;
        endcase
endmodule
// --- ID(Instruction Decode) end ---

// --- EX(Execution) start ---
module EX (alucontrol, dstregin, controls, pcplus4in, pc, rsrca, rsrcb, immextv,
        btaken, dstregout, ctrlsout, pcplus4out, pcbranch, aluout, writedata);
    input [3:0] alucontrol;
    input [4:0] dstregin;
    input [12:0] controls;
    input [31:0] pcplus4in, pc, rsrca, rsrcb, immextv;
    output btaken;
    output [4:0] dstregout;
    output [12:0] ctrlsout;
    output [31:0] pcplus4out, pcbranch, aluout, writedata;

    wire u_op, alusrc;
    wire [31:0] srca, srcb;

    assign pcbranch = pc + immextv;
    assign u_op = controls[4];
    assign srca = u_op ? 32'h0 : rsrca;
    assign alusrc = controls[11];
    assign srcb = alusrc ? immextv : rsrcb;
    alu alu (alucontrol, srca, srcb, btaken, aluout);
    assign pcplus4out = pcplus4in;
    assign writedata = rsrcb;
    assign dstregout = dstregin;
    assign ctrlsout = controls;
endmodule

module alu (alucontrol, srca, srcb, btaken, aluout);
    input [3:0] alucontrol;
    input [31:0] srca;
    input [31:0] srcb;
    output reg btaken;
    output reg [31:0] aluout;

    always @ (*)
        begin
        // Latter 3bits of alucontrol is from funct3.
        case (alucontrol)
            4'b0000: aluout <= srca + srcb;
            4'b1000: aluout <= srca - srcb;
            4'b0010: aluout <= $signed(srca) < $signed(srcb);
            4'b0110: aluout <= srca | srcb;
            4'b0111: aluout <= srca & srcb;
            // -------
            4'b0100: aluout <= srca ^ srcb;
            4'b0001: aluout <= srca << srcb;
            4'b0101: aluout <= srca >> srcb;
            4'b1101: aluout <= srca >> srcb; // not familiar with msb-extends
            4'b0011: aluout <= srca < srcb;
        endcase
        case (alucontrol) // for conditonal branch
            4'b0000: btaken <= (srca == srcb) ? 1 : 0;
            // -------
            4'b0001: btaken <= (srca != srcb) ? 1 : 0;
            4'b0100: btaken <= ($signed(srca) < $signed(srcb)) ? 1 : 0;
            4'b0101: btaken <= ($signed(srca) < $signed(srcb)) ? 0 : 1;
            4'b0110: btaken <= (srca < srcb) ? 1 : 0;
            4'b0111: btaken <= (srca < srcb) ? 0 : 1;
        endcase
        end
endmodule
// --- EX(Execution) end ---

// --- MEM(Memory Access) start ---
module MEM (clk, btakenin, dstregin, controls, pcplus4in, pc, dataaddr, writedata,
            btakenout, dstregout, ctrlsout, pcplus4out, aluout, readdata);
    input clk, btakenin;
    input [4:0] dstregin;
    input [12:0] controls;
    input [31:0] pcplus4in, pc, dataaddr, writedata;
    output btakenout;
    output [4:0] dstregout;
    output [12:0] ctrlsout;
    output [31:0] pcplus4out, aluout, readdata;

    dmem dmem (clk, controls[10], dataaddr, writedata, readdata);
    assign pcplus4out = pcplus4in;
    assign btakenout = btakenin;
    assign aluout = dataaddr;
    assign ctrlsout = controls;
    assign dstregout = dstregin;
endmodule

module dmem (clk, memwrite, dataaddr, writedata, readdata);
    input  clk, memwrite;
    input  [31:0] dataaddr, writedata;
    output [31:0] readdata;

    reg [31:0] RAM [0:63];
    assign readdata = RAM[dataaddr[31:2]];
    always @ (negedge clk)
    if (memwrite)
        RAM[dataaddr[31:2]] <= writedata;

    initial
        begin
	        $readmemh (`DATFILE, RAM);
        end
endmodule
// --- MEM(Memory Access) end ---

// --- WB(Write Back) start ---
module WB (btaken, dstreg, controls, aluout, readdata, pcplus4,
            pcsrc, j_op, regwrite, writereg, writedata);
    input btaken;
    input [4:0] dstreg;
    input [12:0] controls;
    input [31:0] pcplus4, aluout, readdata;
    output pcsrc, j_op, regwrite;
    output [4:0] writereg; 
    output [31:0] writedata;

    assign pcsrc = controls[5] & btaken;
    assign j_op = controls[3];
    assign regwrite = controls[12];
    assign writereg = dstreg;
    assign writedata = controls[3] ? pcplus4 : (controls[9] ? readdata : aluout);
endmodule
// --- WB(Write Back) end ---