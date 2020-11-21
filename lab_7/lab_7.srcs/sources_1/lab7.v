`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Dept. of Computer Science, National Chiao Tung University
// Engineer: Chun-Jen Tsai
// 
// Create Date: 2018/11/01 11:16:50
// Design Name: 
// Module Name: lab6
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: This is a sample circuit to show you how to initialize an SRAM
//              with a pre-defined data file. Hit BTN0/BTN1 let you browse
//              through the data.
// 
// Dependencies: LCD_module, debounce
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module lab6(
  // General system I/O ports
  input  clk,
  input  reset_n,
  input  [3:0] usr_btn,
  output [3:0] usr_led,

  // 1602 LCD Module Interface
  output LCD_RS,
  output LCD_RW,
  output LCD_E,
  output [3:0] LCD_D,
  
  //uart Module Interface
  input uart_rx,
  output uart_tx
);

localparam [1:0] S_MAIN_ADDR = 3'b000, S_MAIN_READ = 3'b001,
                 S_MAIN_SHOW = 3'b010, S_MAIN_WAIT = 3'b011;
localparam [1:0] S_UART_IDLE = 0, S_UART_WAIT = 1,
                 S_UART_SEND = 2, S_UART_INCR = 3;

localparam l1_len = 35;
localparam l2_len = 6;
localparam l3_len = 33;
localparam l4_len = 33;
localparam l5_len = 33;
localparam l6_len = 33;
localparam MEM_SIZE = l1_len + l2_len + l3_len + l4_len + l5_len + l6_len;

// declare system variables
wire [1:0]  btn_level, btn_pressed;
reg  [1:0]  prev_btn_level;
reg  [1:0]  P, P_next;
reg  [11:0] user_addr;
reg  [7:0]  user_data;

reg  [127:0] row_A, row_B;

wire print_enable, print_done;
reg [$clog2(MEM_SIZE):0] send_counter;
reg [7:0] data[0:MEM_SIZE-1];
reg Q, Q_next;

reg [0:l1_len*8-1] line1 = {"\015\012The matrix multiplication result",8'h00};
reg [0:l2_len*8-1] line2 = {"\015\012is:",8'h00};
reg [0:l3_len*8-1] line3 = {"\015\012[ -----, -----, -----, ----- ]",8'h00};
reg [0:l4_len*8-1] line4 = {"\015\012[ -----, -----, -----, ----- ]",8'h00};
reg [0:l5_len*8-1] line5 = {"\015\012[ -----, -----, -----, ----- ]",8'h00};
reg [0:l6_len*8-1] line6 = {"\015\012[ -----, -----, -----, ----- ]",8'h00};

// declare UART signals
wire transmit;
wire received;
wire [7:0] rx_byte;
//reg  [7:0] rx_temp;  // if received is true, rx_temp latches rx_byte for ONLY ONE CLOCK CYCLE!
wire [7:0] tx_byte;
//wire [7:0] echo_key; // keystrokes to be echoed to the terminal
//wire is_num_key;
wire is_receiving;
wire is_transmitting;
wire recv_error;

// declare SRAM control signals
wire [10:0] sram_addr;
wire [7:0]  data_in;
wire [7:0]  data_out;
wire        sram_we, sram_en;

assign usr_led = 4'h00;

LCD_module lcd0( 
  .clk(clk),
  .reset(~reset_n),
  .row_A(row_A),
  .row_B(row_B),
  .LCD_E(LCD_E),
  .LCD_RS(LCD_RS),
  .LCD_RW(LCD_RW),
  .LCD_D(LCD_D)
);
  
debounce btn_db0(
  .clk(clk),
  .btn_input(usr_btn[0]),
  .btn_output(btn_level[0])
);

debounce btn_db1(
  .clk(clk),
  .btn_input(usr_btn[1]),
  .btn_output(btn_level[1])
);

uart uart(
  .clk(clk),
  .rst(~reset_n),
  .rx(uart_rx),
  .tx(uart_tx),
  .transmit(transmit),
  .tx_byte(tx_byte),
  .received(received),
  .rx_byte(rx_byte),
  .is_receiving(is_receiving),
  .is_transmitting(is_transmitting),
  .recv_error(recv_error)
);

//
// Enable one cycle of btn_pressed per each button hit
//
always @(posedge clk) begin
  if (~reset_n)
    prev_btn_level <= 2'b00;
  else
    prev_btn_level <= btn_level;
end

assign btn_pressed = (btn_level & ~prev_btn_level);

// ------------------------------------------------------------------------
// The following code creates an initialized SRAM memory block that
// stores an 1024x8-bit unsigned numbers.
sram ram0(.clk(clk), .we(sram_we), .en(sram_en),
          .addr(sram_addr), .data_i(data_in), .data_o(data_out));

assign sram_we = usr_btn[3]; // In this demo, we do not write the SRAM. However,
                             // if you set 'we' to 0, Vivado fails to synthesize
                             // ram0 as a BRAM -- this is a bug in Vivado.
assign sram_en = (P == S_MAIN_ADDR || P == S_MAIN_READ); // Enable the SRAM block.
assign sram_addr = user_addr[11:0];
assign data_in = 8'b0; // SRAM is read-only so we tie inputs to zeros.
// End of the SRAM memory block.
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// FSM of the main controller
always @(posedge clk) begin
  if (~reset_n) begin
    P <= S_MAIN_ADDR; // read samples at 000 first
  end
  else begin
    P <= P_next;
  end
end

always @(*) begin // FSM next-state logic
  case (P)
    S_MAIN_ADDR: // send an address to the SRAM 
      P_next = S_MAIN_READ;
    S_MAIN_READ: // fetch the sample from the SRAM
      P_next = S_MAIN_SHOW;
    S_MAIN_SHOW:
      P_next = S_MAIN_WAIT;
    S_MAIN_WAIT: // wait for a button click
      if (| btn_pressed == 1) P_next = S_MAIN_ADDR;
      else P_next = S_MAIN_WAIT;
  endcase
end

// FSM ouput logic: Fetch the data bus of sram[] for display
always @(posedge clk) begin
  if (~reset_n) user_data <= 8'b0;
  else if (sram_en && !sram_we) user_data <= data_out;
end
// End of the main controller
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// The following code updates the 1602 LCD text messages.
always @(posedge clk) begin
  if (~reset_n) begin
    row_A <= "Data at [0x---] ";
  end
  else if (P == S_MAIN_SHOW) begin
    row_A[39:32] <= ((user_addr[11:08] > 9)? "7" : "0") + user_addr[11:08];
    row_A[31:24] <= ((user_addr[07:04] > 9)? "7" : "0") + user_addr[07:04];
    row_A[23:16] <= ((user_addr[03:00] > 9)? "7" : "0") + user_addr[03:00];
  end
end

always @(posedge clk) begin
  if (~reset_n) begin
    row_B <= "is equal to 0x--";
  end
  else if (P == S_MAIN_SHOW) begin
    row_B[15:08] <= ((user_data[7:4] > 9)? "7" : "0") + user_data[7:4];
    row_B[07: 0] <= ((user_data[3:0] > 9)? "7" : "0") + user_data[3:0];
  end
end
// End of the 1602 LCD text-updating code.
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// The circuit block that processes the user's button event.
always @(posedge clk) begin
  if (~reset_n)
    user_addr <= 12'h000;
  else if (btn_pressed[1])
    user_addr <= (user_addr < 2048)? user_addr + 1 : user_addr;
  else if (btn_pressed[0])
    user_addr <= (user_addr > 0)? user_addr - 1 : user_addr;
end
// End of the user's button control.
// ------------------------------------------------------------------------
// FSM output logics: UART transmission control signals
assign transmit = (Q_next == S_UART_WAIT || 
//                  (P == S_MAIN_READ_NUM && received) || 
//                  (P == S_SECOND_READ_NUM && received) ||
                  print_enable);
// ------------------------------------------------------------------------
// FSM of the controller that sends a string to the UART.
always @(posedge clk) begin
  if (~reset_n) Q <= S_UART_IDLE;
  else Q <= Q_next;
end

always @(*) begin // FSM next-state logic
  case (Q)
    S_UART_IDLE: // wait for the print_string flag
      if (print_enable) Q_next = S_UART_WAIT;
      else Q_next = S_UART_IDLE;
    S_UART_WAIT: // wait for the transmission of current data byte begins
      if (is_transmitting == 1) Q_next = S_UART_SEND;
      else Q_next = S_UART_WAIT;
    S_UART_SEND: // wait for the transmission of current data byte finishes
      if (is_transmitting == 0) Q_next = S_UART_INCR; // transmit next character
      else Q_next = S_UART_SEND;
    S_UART_INCR:
      if (tx_byte == 8'h0) Q_next = S_UART_IDLE; // string transmission ends
      else Q_next = S_UART_WAIT;
  endcase
end

assign transmit = (Q_next == S_UART_WAIT || print_enable);
assign tx_byte = (received) ? 0 : data[send_counter]; 


endmodule
