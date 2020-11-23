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

localparam [2:0] S_MAIN_ADDR = 3'b000, S_MAIN_READ = 3'b001,
                 S_MAIN_SHOW = 3'b010, S_MAIN_WAIT = 3'b011,
                 S_MAIN_COMPUTE = 3'b100, S_MAIN_UART = 3'b101,
                 S_MAIN_UART_DISPLAY = 3'b110, S_REPEAT = 3'b111;
localparam [1:0] S_UART_IDLE = 0, S_UART_WAIT = 1,
                 S_UART_SEND = 2, S_UART_INCR = 3;

localparam l1_len = 34;
localparam l2_len = 5;
localparam l3_len = 32;
localparam l4_len = 32;
localparam l5_len = 32;
localparam l6_len = 33;
localparam MEM_SIZE = l1_len + l2_len + l3_len + l4_len + l5_len + l6_len;

// declare system variables
wire [1:0]  btn_level, btn_pressed;
reg  [1:0]  prev_btn_level;
reg  [2:0]  P, P_next;
reg  [11:0] user_addr;
reg  [7:0]  user_data;

reg  [127:0] row_A, row_B;

wire print_enable, print_done;
reg [$clog2(MEM_SIZE):0] send_counter;
reg [7:0] data[0:MEM_SIZE-1];
reg [1:0] Q, Q_next;

reg [0:l1_len*8-1] line1 = {"\015\012The matrix multiplication result"};
reg [0:l2_len*8-1] line2 = {"\015\012is:"};
reg [0:l3_len*8-1] line3 = {"\015\012[ -----, -----, -----, ----- ]"}; // start index 39
reg [0:l4_len*8-1] line4 = {"\015\012[ -----, -----, -----, ----- ]"}; // start index 71
reg [0:l5_len*8-1] line5 = {"\015\012[ -----, -----, -----, ----- ]"}; // start index 103
reg [0:l6_len*8-1] line6 = {"\015\012[ -----, -----, -----, ----- ]",8'h00}; // start index 135

integer idx,idx1,idx2,idx3;
integer p1,p2,p3,p4;
reg [7:0] mtx1[0:15];
reg [7:0] mtx2[0:15]; 
reg [17:0] mtx3[0:15]; // result matrix
reg [17:0] pipe1[0:3], pipe2[0:3], pipe3[0:3], pipe4[0:3];
reg start;
reg result_ready;
reg mtx1_ready;
reg mtx2_ready;
reg pipe1_ready, pipe2_ready, pipe3_ready, pipe4_ready;

// declare UART signals
wire transmit;
wire received;
wire [7:0] rx_byte;
wire [7:0] tx_byte;
wire is_receiving;
wire is_transmitting;
wire recv_error;

// declare SRAM control signals
wire [10:0] sram_addr;
wire [7:0]  data_in;
wire [7:0]  data_out;
wire        sram_we, sram_en;

assign usr_led = P;

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
      if (start) P_next = S_MAIN_READ;
      else P_next = S_MAIN_ADDR;
    S_MAIN_READ: // fetch the sample from the SRAM
      P_next = S_MAIN_SHOW;
    S_MAIN_SHOW:
      P_next <= S_MAIN_WAIT;
//      if (btn_pressed[1]) P_next = S_MAIN_WAIT;
//      else P_next = S_MAIN_SHOW;
    S_MAIN_WAIT: // wait for a button click
      // if (| btn_pressed == 1) P_next = S_MAIN_ADDR;
      // else P_next = S_MAIN_WAIT;
      if (mtx1_ready && mtx2_ready) P_next = S_MAIN_COMPUTE;
      else P_next = S_MAIN_ADDR;
    S_MAIN_COMPUTE:
      if (result_ready) P_next = S_MAIN_UART;
      else P_next = S_MAIN_COMPUTE;
    S_MAIN_UART:
      P_next = S_MAIN_UART_DISPLAY;
    S_MAIN_UART_DISPLAY:
      if (print_done) P_next = S_REPEAT; 
      else P_next = S_MAIN_UART_DISPLAY;
//      else P_next = S_MAIN_UART_DISPLAY;
    S_REPEAT:
      P_next = S_MAIN_ADDR;
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
//     row_A <= "Data at [0x---] ";
     row_A <= "                ";
   end
//   else if (P == S_MAIN_SHOW) begin
//     row_A[39:32] <= ((user_addr[11:08] > 9)? "7" : "0") + user_addr[11:08];
//     row_A[31:24] <= ((user_addr[07:04] > 9)? "7" : "0") + user_addr[07:04];
//     row_A[23:16] <= ((user_addr[03:00] > 9)? "7" : "0") + user_addr[03:00];
//   end
    else if (P == S_MAIN_COMPUTE) begin
        row_A <= "mtx1, mtx2 ready";
    end
     else if (P == S_MAIN_UART) begin
        row_A <= "   mtx3 ready   ";
    end
     else if (P == S_REPEAT) begin
        row_A <= "   mtx3 [0,1]   ";
    end
 end

 always @(posedge clk) begin
   if (~reset_n) begin
//     row_B <= "is equal to 0x--";
     row_B <= "                ";
   end
   else if (P == S_MAIN_COMPUTE) begin
        row_B <= " computing mtx3 ";
    end
    else if (P == S_MAIN_UART_DISPLAY) begin
        row_B <= "displaying mtx3 ";
    end
    else if (P == S_REPEAT) begin
        row_B <= {"     ",data[50],data[51],data[52],data[53],data[54],"      "};
    end
//   else if (P == S_MAIN_SHOW) begin
//   end
//     row_B[15:08] <= ((user_data[7:4] > 9)? "7" : "0") + user_data[7:4];
//     row_B[07: 0] <= ((user_data[3:0] > 9)? "7" : "0") + user_data[3:0];
 end
// End of the 1602 LCD text-updating code.
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// The circuit block that processes the user's button event.
// always @(posedge clk) begin
//   if (~reset_n)
//     user_addr <= 12'h000;
//   else if (btn_pressed[1])
//     user_addr <= (user_addr < 2048)? user_addr + 1 : user_addr;
//   else if (btn_pressed[0])
//     user_addr <= (user_addr > 0)? user_addr - 1 : user_addr;
// end
// End of the user's button control.
// ------------------------------------------------------------------------
// FSM output logics: UART transmission control signals
assign transmit = (Q_next == S_UART_WAIT || print_enable);
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

assign print_enable = (P == S_MAIN_UART && P_next == S_MAIN_UART_DISPLAY) ||
                  (P == S_MAIN_UART_DISPLAY && P_next == S_REPEAT);
//(P == S_MAIN_UART || P == S_MAIN_UART_DISPLAY || P == S_REPEAT);
assign print_done = (tx_byte == 8'h0);
//assign transmit = (Q_next == S_UART_WAIT || print_enable);
assign tx_byte = data[send_counter];
//assign result_ready = (pipe1_ready && pipe2_ready && pipe3_ready && pipe4_ready);

always @(posedge clk) begin
  case (P_next)
    S_MAIN_UART: send_counter <= 0;
    default: send_counter <= send_counter + (Q_next == S_UART_INCR);
  endcase
end

always @(posedge clk) begin
    if (~reset_n) start <= 0;
    else if (P == S_MAIN_ADDR && btn_pressed[1]) start <= 1;
    else if (P == S_MAIN_COMPUTE) start <= 0;
end

always @(posedge clk) begin
  if (~reset_n) begin
    {data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], 
     data[9], data[10], data[11], data[12], data[13], data[14], data[15], data[16], 
     data[17], data[18], data[19], data[20], data[21], data[22], data[23], data[24], 
     data[25], data[26], data[27], data[28], data[29], data[30], data[31], data[32], 
     data[33]}
    <= {line1} ;

    {data[34], data[35], data[36], data[37], data[38]} 
    <= {line2};

    {data[39], data[40], data[41], data[42], data[43], data[44], data[45], data[46], data[47], data[48], 
     data[49], data[50], data[51], data[52], data[53], data[54], data[55], data[56], 
     data[57], data[58], data[59], data[60], data[61], data[62], data[63], data[64], 
     data[65], data[66], data[67], data[68], data[69], data[70]}
    <= {line3};

    {data[71], data[72], data[73], data[74], data[75], data[76], data[77], data[78], data[79], data[80], data[81], 
     data[82], data[83], data[84], data[85], data[86], data[87], data[88], data[89], 
     data[90], data[91], data[92], data[93], data[94], data[95], data[96], data[97], 
     data[98], data[99], data[100], data[101], data[102]}
    <= {line4};

    {data[103], data[104], data[105], data[106], data[107], data[108], data[109], data[110], data[111], 
     data[112], data[113], data[114], data[115], data[116], data[117], data[118], data[119], 
     data[120], data[121], data[122], data[123], data[124], data[125], data[126], data[127], 
     data[128], data[129], data[130], data[131], data[132], data[133], data[134]}
    <= {line5};

    {data[135], data[136], data[137], data[138], 
     data[139], data[140], data[141], data[142], data[143], data[144], data[145], data[146], data[147], 
     data[148], data[149], data[150], data[151], data[152], data[153], data[154], data[155],
     data[156], data[157], data[158], data[159], data[160], data[161], data[162], data[163], 
     data[164], data[165], data[166], data[167]}
     <= {line6};
  end
  else if (P == S_MAIN_UART_DISPLAY) begin
    for (idx=0; idx<4; idx=idx+1) begin
      data[43+idx*32] <= ((mtx3[idx*4+0][17:16] > 9) ? "7" : "0") + mtx3[idx*4+0][17:16];
      data[44+idx*32] <= ((mtx3[idx*4+0][15:12] > 9) ? "7" : "0") + mtx3[idx*4+0][15:12];
      data[45+idx*32] <= ((mtx3[idx*4+0][11: 8] > 9) ? "7" : "0") + mtx3[idx*4+0][11: 8];
      data[46+idx*32] <= ((mtx3[idx*4+0][ 7: 4] > 9) ? "7" : "0") + mtx3[idx*4+0][ 7: 4];
      data[47+idx*32] <= ((mtx3[idx*4+0][ 3: 0] > 9) ? "7" : "0") + mtx3[idx*4+0][ 3: 0];

      data[50+idx*32] <= ((mtx3[idx*4+1][17:16] > 9) ? "7" : "0") + mtx3[idx*4+1][17:16];
      data[51+idx*32] <= ((mtx3[idx*4+1][15:12] > 9) ? "7" : "0") + mtx3[idx*4+1][15:12];
      data[52+idx*32] <= ((mtx3[idx*4+1][11: 8] > 9) ? "7" : "0") + mtx3[idx*4+1][11: 8];
      data[53+idx*32] <= ((mtx3[idx*4+1][ 7: 4] > 9) ? "7" : "0") + mtx3[idx*4+1][ 7: 4];
      data[54+idx*32] <= ((mtx3[idx*4+1][ 3: 0] > 9) ? "7" : "0") + mtx3[idx*4+1][ 3: 0];

      data[57+idx*32] <= ((mtx3[idx*4+2][17:16] > 9) ? "7" : "0") + mtx3[idx*4+2][17:16];
      data[58+idx*32] <= ((mtx3[idx*4+2][15:12] > 9) ? "7" : "0") + mtx3[idx*4+2][15:12];
      data[59+idx*32] <= ((mtx3[idx*4+2][11: 8] > 9) ? "7" : "0") + mtx3[idx*4+2][11: 8];
      data[60+idx*32] <= ((mtx3[idx*4+2][ 7: 4] > 9) ? "7" : "0") + mtx3[idx*4+2][ 7: 4];
      data[61+idx*32] <= ((mtx3[idx*4+2][ 3: 0] > 9) ? "7" : "0") + mtx3[idx*4+2][ 3: 0];

      data[64+idx*32] <= ((mtx3[idx*4+3][17:16] > 9) ? "7" : "0") + mtx3[idx*4+3][17:16];
      data[65+idx*32] <= ((mtx3[idx*4+3][15:12] > 9) ? "7" : "0") + mtx3[idx*4+3][15:12];
      data[66+idx*32] <= ((mtx3[idx*4+3][11: 8] > 9) ? "7" : "0") + mtx3[idx*4+3][11: 8];
      data[67+idx*32] <= ((mtx3[idx*4+3][ 7: 4] > 9) ? "7" : "0") + mtx3[idx*4+3][ 7: 4];
      data[68+idx*32] <= ((mtx3[idx*4+3][ 3: 0] > 9) ? "7" : "0") + mtx3[idx*4+3][ 3: 0];
    end
  end
end 

always @(posedge clk) begin
  if (~reset_n) begin
    result_ready <= 0;
    // mtx1 <= 0;
    // mtx2 <= 0;
    // mtx3 <= 0;
    user_addr <= 12'h000;
    idx1 <= 0;
    idx2 <= 0;
    mtx1_ready <= 0;
    mtx2_ready <= 0;
    // idx3 <= 0;
  end
  else if (P == S_MAIN_WAIT) begin
    if (!mtx1_ready) begin
      mtx1[idx1] <= user_data;
      idx1 = idx1+1;
      if (idx1>=16) begin
        mtx1_ready <= 1;
      end
    end
    else if (!mtx2_ready && mtx1_ready) begin
      mtx2[idx2] <= user_data;
      idx2 = idx2+1;
      if (idx2>=16) mtx2_ready <= 1;
    end
    user_addr = user_addr + 1;
  end
  else if (P == S_MAIN_UART) begin
    {mtx3[ 0], mtx3[ 1], mtx3[ 2], mtx3[ 3], 
     mtx3[ 4], mtx3[ 5], mtx3[ 6], mtx3[ 7], 
     mtx3[ 8], mtx3[ 9], mtx3[10], mtx3[11],
     mtx3[12], mtx3[13], mtx3[14], mtx3[15]}
    <= {pipe1[0], pipe1[1], pipe1[2], pipe1[3],
        pipe2[0], pipe2[1], pipe2[2], pipe2[3],
        pipe3[0], pipe3[1], pipe3[2], pipe3[3],
        pipe4[0], pipe4[1], pipe4[2], pipe4[3]};
  end
  else if (P == S_MAIN_COMPUTE) begin
    if (pipe1_ready && pipe2_ready && pipe3_ready && pipe4_ready) result_ready <= 1;
  end
 // else if (P == S_MAIN_COMPUTE) begin
  //   case (idx3)
  //      0: mtx3[idx3] <= mtx1[0]*mtx2[0] + mtx1[1]*mtx2[4] + mtx1[2]*mtx2[ 8] + mtx1[3]*mtx2[12];
  //      1: mtx3[idx3] <= mtx1[0]*mtx2[1] + mtx1[1]*mtx2[5] + mtx1[2]*mtx2[ 9] + mtx1[3]*mtx2[13];
  //      2: mtx3[idx3] <= mtx1[0]*mtx2[2] + mtx1[1]*mtx2[6] + mtx1[2]*mtx2[10] + mtx1[3]*mtx2[14];
  //      3: mtx3[idx3] <= mtx1[0]*mtx2[3] + mtx1[1]*mtx2[7] + mtx1[2]*mtx2[11] + mtx1[3]*mtx2[15];

  //      4: mtx3[idx3] <= mtx1[4]*mtx2[0] + mtx1[5]*mtx2[4] + mtx1[6]*mtx2[ 8] + mtx1[7]*mtx2[12];
  //      5: mtx3[idx3] <= mtx1[4]*mtx2[1] + mtx1[5]*mtx2[5] + mtx1[6]*mtx2[ 9] + mtx1[7]*mtx2[13];
  //      6: mtx3[idx3] <= mtx1[4]*mtx2[2] + mtx1[5]*mtx2[6] + mtx1[6]*mtx2[10] + mtx1[7]*mtx2[14];
  //      7: mtx3[idx3] <= mtx1[4]*mtx2[3] + mtx1[5]*mtx2[7] + mtx1[6]*mtx2[11] + mtx1[7]*mtx2[15];

  //      8: mtx3[idx3] <= mtx1[8]*mtx2[0] + mtx1[9]*mtx2[4] + mtx1[10]*mtx2[ 8] + mtx1[11]*mtx2[12];
  //      9: mtx3[idx3] <= mtx1[8]*mtx2[1] + mtx1[9]*mtx2[5] + mtx1[10]*mtx2[ 9] + mtx1[11]*mtx2[13];
  //     10: mtx3[idx3] <= mtx1[8]*mtx2[2] + mtx1[9]*mtx2[6] + mtx1[10]*mtx2[10] + mtx1[11]*mtx2[14];
  //     11: mtx3[idx3] <= mtx1[8]*mtx2[3] + mtx1[9]*mtx2[7] + mtx1[10]*mtx2[11] + mtx1[11]*mtx2[15];

  //     12: mtx3[idx3] <= mtx1[12]*mtx2[0] + mtx1[13]*mtx2[4] + mtx1[14]*mtx2[ 8] + mtx1[15]*mtx2[12];
  //     13: mtx3[idx3] <= mtx1[12]*mtx2[1] + mtx1[13]*mtx2[5] + mtx1[14]*mtx2[ 9] + mtx1[15]*mtx2[13];
  //     14: mtx3[idx3] <= mtx1[12]*mtx2[2] + mtx1[13]*mtx2[6] + mtx1[14]*mtx2[10] + mtx1[15]*mtx2[14];
  //     15: mtx3[idx3] <= mtx1[12]*mtx2[3] + mtx1[13]*mtx2[7] + mtx1[14]*mtx2[11] + mtx1[15]*mtx2[15];
  //   endcase
  //   idx3 = idx3+1;
  //   if (idx3>=16) result_ready <= 1;
  // end
end

always @(posedge clk) begin // calculate first row 
  if (~reset_n) begin
    p1 <= 0;
    pipe1_ready <= 0;
  end
  else if (P == S_MAIN_COMPUTE && !pipe1_ready) begin 
    case (p1)
      0: pipe1[p1] <= mtx1[0]*mtx2[0] + mtx1[4]*mtx2[1] + mtx1[8]*mtx2[ 2] + mtx1[12]*mtx2[ 3];
      1: pipe1[p1] <= mtx1[0]*mtx2[4] + mtx1[4]*mtx2[5] + mtx1[8]*mtx2[ 6] + mtx1[12]*mtx2[ 7];
      2: pipe1[p1] <= mtx1[0]*mtx2[8] + mtx1[4]*mtx2[9] + mtx1[8]*mtx2[10] + mtx1[12]*mtx2[11];
      3: pipe1[p1] <= mtx1[0]*mtx2[12] + mtx1[4]*mtx2[13] + mtx1[8]*mtx2[14] + mtx1[12]*mtx2[15];
    endcase
    p1 = p1+1;
    if (p1>=4) pipe1_ready = 1;
  end
end

always @(posedge clk) begin // calculate second row 
  if (~reset_n) begin
    p2 <= 0;
    pipe2_ready <= 0;
  end
  else if (P == S_MAIN_COMPUTE && !pipe2_ready) begin
    case (p2)
      0: pipe2[p2] <= mtx1[1]*mtx2[0] + mtx1[5]*mtx2[1] + mtx1[9]*mtx2[ 2] + mtx1[13]*mtx2[ 3];
      1: pipe2[p2] <= mtx1[1]*mtx2[4] + mtx1[5]*mtx2[5] + mtx1[9]*mtx2[ 6] + mtx1[13]*mtx2[ 7];
      2: pipe2[p2] <= mtx1[1]*mtx2[8] + mtx1[5]*mtx2[9] + mtx1[9]*mtx2[10] + mtx1[13]*mtx2[11];
      3: pipe2[p2] <= mtx1[1]*mtx2[12] + mtx1[5]*mtx2[13] + mtx1[9]*mtx2[14] + mtx1[13]*mtx2[15];
    endcase
    p2 = p2+1;
    if (p2>=4) pipe2_ready = 1;
  end
end

always @(posedge clk) begin // calculate 3rd row 
  if (~reset_n) begin
    p3 <= 0;
    pipe3_ready <= 0;
  end
  else if (P == S_MAIN_COMPUTE && !pipe3_ready) begin
    case (p3)
      0: pipe3[p3] <= mtx1[2]*mtx2[0] + mtx1[6]*mtx2[1] + mtx1[10]*mtx2[ 2] + mtx1[14]*mtx2[ 3];
      1: pipe3[p3] <= mtx1[2]*mtx2[4] + mtx1[6]*mtx2[5] + mtx1[10]*mtx2[ 6] + mtx1[14]*mtx2[ 7];
      2: pipe3[p3] <= mtx1[2]*mtx2[8] + mtx1[6]*mtx2[9] + mtx1[10]*mtx2[10] + mtx1[14]*mtx2[11];
      3: pipe3[p3] <= mtx1[2]*mtx2[12] + mtx1[6]*mtx2[13] + mtx1[10]*mtx2[14] + mtx1[14]*mtx2[15];
    endcase
    p3 = p3+1;
    if (p3>=4) pipe3_ready = 1;
  end
end

always @(posedge clk) begin // calculate 4th row 
  if (~reset_n) begin
    p4 <= 0;
    pipe4_ready <= 0;
  end
  else if (P == S_MAIN_COMPUTE && !pipe4_ready) begin
    case (p4)
      0: pipe4[p4] <= mtx1[3]*mtx2[0] + mtx1[7]*mtx2[1] + mtx1[11]*mtx2[ 2] + mtx1[15]*mtx2[ 3];
      1: pipe4[p4] <= mtx1[3]*mtx2[4] + mtx1[7]*mtx2[5] + mtx1[11]*mtx2[ 6] + mtx1[15]*mtx2[ 7];
      2: pipe4[p4] <= mtx1[3]*mtx2[8] + mtx1[7]*mtx2[9] + mtx1[11]*mtx2[10] + mtx1[15]*mtx2[11];
      3: pipe4[p4] <= mtx1[3]*mtx2[12] + mtx1[7]*mtx2[13] + mtx1[11]*mtx2[14] + mtx1[15]*mtx2[15];
    endcase
    p4 = p4+1;
    if (p4>=4) pipe4_ready = 1;
  end
end

endmodule
