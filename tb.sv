
package pkg1;
    import uvm_pkg::*;
    `include "uvm_macros.svh";

    class my_seqi extends uvm_sequence_item;
      `uvm_object_utils(my_seqi)
      
      logic we;
      logic re; 
      rand logic [7:0] addr;
      rand logic [7:0] din; 
      logic [7:0] dout; 
      
      function new(string name ="my_seqi");
        super.new(name);
      endfunction


    endclass
	class my_sequence extends uvm_sequence#(my_seqi);
      `uvm_object_utils(my_sequence)
      my_seqi seqi;
          function new(string name = "my_sequence");
       		 super.new(name);
          endfunction
      task pre_body();
        seqi = my_seqi::type_id::create("seqi");
        endtask
      task body();
        for(int i =0;i<10;i++)begin
        start_item(seqi);
        	seqi.randomize();
          	seqi.we =1 ;
          	seqi.re =1 ;
          	
        finish_item(seqi);
        end
      endtask
    endclass

    
	class my_sequencer extends uvm_sequencer#(my_seqi);
      `uvm_component_utils(my_sequencer)
          function new(string name = "my_sequencer",uvm_component parent = null);
       		 super.new(name,parent);
          endfunction
      function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        $display("sequncer_build_phase");

      endfunction
      
      
      
    endclass
	
	class my_driver extends uvm_driver#(my_seqi);
      `uvm_component_utils(my_driver)
      my_seqi seqi;
      virtual mem_intf vif;
      function new(string name = "my_driver",uvm_component parent = null);
       		 super.new(name,parent);
          endfunction
      function void build_phase(uvm_phase phase);
        $display("drv_build_phase");
        super.build_phase(phase);
        uvm_config_db#(virtual mem_intf)::get(this,"","vif",vif);
        seqi = my_seqi::type_id::create("seqi");

      endfunction
      
      task run_phase(uvm_phase phase);
        super.run_phase(phase);
        forever begin
        seq_item_port.get_next_item(seqi);
          @(posedge vif.clk)begin
            vif.we <= seqi.we;
            vif.re <=seqi.re; 
            vif.din <= seqi.din; 
            vif.addr <=1;
          end
          #1step $display("driver %p",vif.din);
        seq_item_port.item_done();
        end
      endtask
      
    endclass

	class my_monitor extends uvm_monitor#(my_seqi);
      `uvm_component_utils(my_monitor)
      uvm_analysis_port#(my_seqi) my_port;
      my_seqi seqi;
      virtual mem_intf vif;
      
      function new(string name = "my_monitor",uvm_component parent = null);
        super.new(name,parent);
      endfunction
      function void build_phase(uvm_phase phase);
        $display("monitor_build_phase");
        super.build_phase(phase);
        uvm_config_db #(virtual mem_intf)::get(this,"","vif",vif);
        my_port = new("my_port",this);
        seqi = my_seqi::type_id::create("seqi");

      endfunction
      task run_phase(uvm_phase phase);
        forever begin @(posedge vif.clk)
          seqi.dout <= vif.dout ; 
          #1step my_port.write(seqi);
          $display("monitor data %p",seqi.dout);
        end
      endtask
      
      
    endclass
	class my_agent extends uvm_agent;
      
      my_driver drv;
      my_sequencer sq;
      my_monitor mon;
      
      `uvm_component_utils(my_agent)
      
      function new(string name = "my_agent",uvm_component parent = null);
       		 super.new(name,parent);
          endfunction
      function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        drv = my_driver::type_id::create("drv",this);
        sq = my_sequencer::type_id::create("sq",this);
        mon = my_monitor::type_id::create("mon",this);
        $display("agent_build_phase");
      endfunction
      function void connect_phase(uvm_phase phase);
        drv.seq_item_port.connect(sq.seq_item_export);
      endfunction
      
    endclass

	class my_scoreboard extends uvm_scoreboard;
      //need implentation
      `uvm_component_utils(my_scoreboard)
      uvm_tlm_analysis_fifo#(my_seqi) fifo_port;
      my_seqi seqi;
      function new(string name = "my_scoreboard",uvm_component parent = null);
        super.new(name,parent);
        fifo_port = new("fifo_port",this);
      endfunction
      function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        $display("score_build_phase");
        seqi = my_seqi::type_id:: create("seqi");

      endfunction
      task run_phase(uvm_phase phase);
        super.run_phase(phase);
        forever begin
          fifo_port.get_peek_export.get(seqi);
        end
        
      endtask
      
      
    endclass

	class my_subscriber extends uvm_subscriber#(my_seqi);
      `uvm_component_utils(my_subscriber)
      my_seqi seqi_cov;
      //covergroup
      covergroup group1;
        coverpoint seqi_cov.dout;
      endgroup
      function new(string name = "my_subscriber",uvm_component parent = null);
       		 super.new(name,parent);
        //cover group new
        group1 =new;
          endfunction
      function void build_phase(uvm_phase phase);
        $display("sub_build_phase");

      endfunction
      virtual function void write(my_seqi seqi);
        seqi_cov =seqi;
        $display("sub item %p",seqi_cov.dout);
        //sample
        group1.sample();
      endfunction
      
      
    endclass
        
	class my_env extends uvm_env;
      `uvm_component_utils(my_env)
      my_agent ag;
      my_subscriber sb;
      my_scoreboard sc;
      function new(string name = "my_env",uvm_component parent = null);
       		 super.new(name,parent);
          endfunction
      function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        ag = my_agent::type_id::create("ag",this);
        sb = my_subscriber::type_id::create("sb",this);
        sc = my_scoreboard::type_id::create("sc",this);
        $display("env_buildPhase");

      endfunction
      function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        ag.mon.my_port.connect(sb.analysis_export);
        ag.mon.my_port.connect(sc.fifo_port.analysis_export);
      endfunction
      
      
    endclass

	class my_test extends uvm_test;
      `uvm_component_utils(my_test)
      my_env env;
      my_sequence seq;
      function new(string name = "my_test ",uvm_component parent = null);
       		 super.new(name,parent);
          endfunction
      function void build_phase(uvm_phase phase);
        env = my_env::type_id::create("env",this);
        seq = my_sequence::type_id::create("seq");
        $display("test_buildPhase");
      endfunction
      task run_phase(uvm_phase phase);
        phase.raise_objection(this);
        seq.start(env.ag.sq);
        phase.drop_objection(this);
      endtask
      
    endclass
	
endpackage
      
module memory (
    input wire clk,
    input wire we, // Write enable
     input wire re, // read enable

    input wire [7:0] addr, // Address
    input wire [7:0] din, // Data input
    output reg [7:0] dout // Data output
);
    reg [7:0] mem [0:255]; // 256 x 8-bit memory

    always @(posedge clk) begin
        if (we) begin
            mem[addr] <= din; // Write operation
        end
      if(re)begin
        dout <= mem[addr]; // Read operation
      end
    end
endmodule

interface mem_intf(input bit clk);
    logic we;
    logic re; 
  logic [7:0] addr;
  logic [7:0] din; 
  logic [7:0] dout; 
endinterface

import uvm_pkg::*;
import pkg1::*;
module top; 
  
  bit clk;
  initial begin
    clk = 1'b0;
  forever #5 clk=~clk;
  end
  
  mem_intf in1(.clk(clk));
  memory mem1(
    .clk(in1.clk),
    .we(in1.we),
    .re(in1.re),
    .addr(in1.addr),
    .din(in1.din),
    .dout(in1.dout)
  );
  initial begin
    $dumpfile("dump.vcd"); $dumpvars(0,top);
    uvm_config_db#(virtual mem_intf)::set(null,"*","vif",in1);
    run_test("my_test");
  end
endmodule
