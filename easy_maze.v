 /*
*First author:Zhao Yongkang
*Second auther:Yan Jingwei
*Last modification time:2018/12/25 18:01
*Merry Christmas!
*Generated automatically by Antimony
 */
module easy_maze
(
	clk_in,
	rst,
	key,
	map,
	row,
	col_red,
	col_green,
	cat,
	seg_led,
	pause,
	beep,
	key_s
);
input clk_in;//Input clock
input rst;//Rreset
input [3:0] key;//Deriction
input map;//Up 1,down 0,default 0
input pause;//Pause the game
input beep;//Warn of brick
input key_s;

output [7:0]row,col_red,col_green;//Point output
output [7:0]cat,seg_led;//Show segment

wire time_judge;//Overtime 1,otherwise 0
reg win_judge;//Success 1,otherwise 0
wire [7:0] movecnt;//Movement counter
wire [63:0] pos;//Positon storage

always@(posedge clk_in or posedge rst)//Set end position
begin
	if(rst)//Reset
		win_judge = 1'b0;
	else if(!map)//Map0
		win_judge = pos[15];
	else//Map1
		win_judge = pos[58];
end

time_move_cnt u1
(
	.clk_in(clk_in),
	.rst(rst),
	.seg_data7(movecnt[7:4]),
	.seg_data6(movecnt[3:0]),
	.seg_led(seg_led),.cat(cat),
	.win_judge(win_judge),
	.flag(time_judge),
	.pause(pause),
	.key_s(key_s)
);
direction_judge u2
(
	.clk_in(clk_in),
	.rst(rst),
	.flag(time_judge),
	.win_judge(win_judge),
	.key(key),.pos(pos),
	.movecnt(movecnt),
	.map(map),
	.pause(pause),
	.beep(beep)
);
led u3//Labyrinth lattice display module
(
	.clk_in(clk_in),
	.rst(rst),.pos(pos),
	.row(row),
	.col_red(col_red),
	.col_green(col_green),
	.time_judge(time_judge),
	.win_judge(win_judge),
	.map(map),
	.key_s(key_s)
);
	
endmodule


module led
(
	clk_in,
	rst,
	pos,
	row,
	col_red,
	col_green,
	time_judge,
	win_judge,
	map,
	key_s
);
input clk_in,rst;
input time_judge,win_judge;
input  [63:0] pos;
input map;
input key_s;
output reg [7:0] row,col_red,col_green;
reg [2:0] cnt;
wire clk_1k;
reg st_judge;
wire key_pulse_s;

defparam divd_1k.divdN=50000,divd_1k.divdWIDTH=16;//Redefining Divider Module Parameters
divd_frequency divd_1k(.rst(rst),.clk_in(clk_in),.clk_out(clk_1k));//Frequency divider 1 kHz

debounce us (.clk(clk_in),.rst(rst),.key(key_s),.key_pulse(key_pulse_s));

initial
begin
	st_judge=1'b0;
end

always@(posedge clk_in)
begin
	if(key_pulse_s)
		st_judge=1'b1;
end

always@(posedge clk_1k or posedge rst)
begin
	if(rst)//cnt clear
		cnt<=3'b000;
	else//cnt++
		cnt<=cnt+1'b1;
end

always@(cnt or rst)
begin
	case(st_judge)
	1'b0:
	begin
	if(rst)//rst clear 8*8 lattice
		begin
		case(cnt)
			3'b000:begin row=8'b01111111;col_red=8'b00000000;col_green=pos[63:56];end
			3'b001:begin row=8'b10111111;col_red=8'b00000000;col_green=pos[55:48];end
			3'b010:begin row=8'b11011111;col_red=8'b00000000;col_green=pos[47:40];end
			3'b011:begin row=8'b11101111;col_red=8'b00000000;col_green=pos[39:32];end
			3'b100:begin row=8'b11110111;col_red=8'b00000000;col_green=pos[31:24];end
			3'b101:begin row=8'b11111011;col_red=8'b00000000;col_green=pos[23:16];end
			3'b110:begin row=8'b11111101;col_red=8'b00000000;col_green=pos[15: 8];end
			3'b111:begin row=8'b11111110;col_red=8'b00000000;col_green=pos[ 7: 0];end
			default:;
			endcase
		end
	else
		case(cnt)
			3'b000:begin row=8'b01111111;col_red=8'b11111111;col_green=8'b00000000;end
			3'b001:begin row=8'b10111111;col_red=8'b10000000;col_green=8'b00000000;end
			3'b010:begin row=8'b11011111;col_red=8'b10111111;col_green=8'b00000000;end
			3'b011:begin row=8'b11101111;col_red=8'b10100001;col_green=8'b00000000;end
			3'b100:begin row=8'b11110111;col_red=8'b10101101;col_green=8'b00000000;end
			3'b101:begin row=8'b11111011;col_red=8'b10111101;col_green=8'b00000000;end
			3'b110:begin row=8'b11111101;col_red=8'b10000001;col_green=8'b00000000;end
			3'b111:begin row=8'b11111110;col_red=8'b11111111;col_green=8'b00000000;end
			default:;
		endcase
	end
	1'b1:
	begin
	if(rst)//rst clear 8*8 lattice
		begin
		case(cnt)
			3'b000:begin row=8'b01111111;col_red=8'b00000000;col_green=pos[63:56];end
			3'b001:begin row=8'b10111111;col_red=8'b00000000;col_green=pos[55:48];end
			3'b010:begin row=8'b11011111;col_red=8'b00000000;col_green=pos[47:40];end
			3'b011:begin row=8'b11101111;col_red=8'b00000000;col_green=pos[39:32];end
			3'b100:begin row=8'b11110111;col_red=8'b00000000;col_green=pos[31:24];end
			3'b101:begin row=8'b11111011;col_red=8'b00000000;col_green=pos[23:16];end
			3'b110:begin row=8'b11111101;col_red=8'b00000000;col_green=pos[15: 8];end
			3'b111:begin row=8'b11111110;col_red=8'b00000000;col_green=pos[ 7: 0];end
			default:;
			endcase
		end
	else if( (!time_judge) & (!win_judge) & (!map))//Show map0
	begin
		case(cnt)
			3'b000:begin row=8'b01111111;col_red=8'b01111111;col_green=pos[63:56];end
			3'b001:begin row=8'b10111111;col_red=8'b01000001;col_green=pos[55:48];end
			3'b010:begin row=8'b11011111;col_red=8'b01011101;col_green=pos[47:40];end
			3'b011:begin row=8'b11101111;col_red=8'b01010101;col_green=pos[39:32];end
			3'b100:begin row=8'b11110111;col_red=8'b01010101;col_green=pos[31:24];end
			3'b101:begin row=8'b11111011;col_red=8'b11000101;col_green=pos[23:16];end
			3'b110:begin row=8'b11111101;col_red=8'b00011100;col_green=pos[15: 8];end
			3'b111:begin row=8'b11111110;col_red=8'b11111111;col_green=pos[ 7: 0];end
			default:;
		endcase
	end
	else if( (!time_judge) & (!win_judge) & (map))//Show map1
	begin
		case(cnt)
			3'b000:begin row=8'b01111111;col_red=8'b00011010;col_green=pos[63:56];end
			3'b001:begin row=8'b10111111;col_red=8'b01111010;col_green=pos[55:48];end
			3'b010:begin row=8'b11011111;col_red=8'b01000011;col_green=pos[47:40];end
			3'b011:begin row=8'b11101111;col_red=8'b11011100;col_green=pos[39:32];end
			3'b100:begin row=8'b11110111;col_red=8'b10010101;col_green=pos[31:24];end
			3'b101:begin row=8'b11111011;col_red=8'b10110101;col_green=pos[23:16];end
			3'b110:begin row=8'b11111101;col_red=8'b10000001;col_green=pos[15: 8];end
			3'b111:begin row=8'b11111110;col_red=8'b11111111;col_green=pos[ 7: 0];end
			default:;
		endcase
	end
	else if( (~time_judge) & win_judge )//Victory
	begin
		case(cnt)
			3'b000:begin row=8'b01111111;col_red=8'b00000000;end
			3'b001:begin row=8'b10111111;col_red=8'b01100110;end
			3'b010:begin row=8'b11011111;col_red=8'b11111111;end
			3'b011:begin row=8'b11101111;col_red=8'b11111111;end
			3'b100:begin row=8'b11110111;col_red=8'b11111111;end
			3'b101:begin row=8'b11111011;col_red=8'b01111110;end
			3'b110:begin row=8'b11111101;col_red=8'b00111100;end
			3'b111:begin row=8'b11111110;col_red=8'b00011000;end
			default:;
		endcase
	end
	else if( time_judge & (~win_judge) )//Fail
	begin
		case(cnt)
			3'b000:begin row=8'b01111111;col_red=8'b00000000;end
			3'b001:begin row=8'b10111111;col_red=8'b01100110;end
			3'b010:begin row=8'b11011111;col_red=8'b11100111;end
			3'b011:begin row=8'b11101111;col_red=8'b11100111;end
			3'b100:begin row=8'b11110111;col_red=8'b11100111;end
			3'b101:begin row=8'b11111011;col_red=8'b01100110;end
			3'b110:begin row=8'b11111101;col_red=8'b00100100;end
			3'b111:begin row=8'b11111110;col_red=8'b00000000;end
			default:;
		endcase
	end
	end
	endcase
end

endmodule

//Countdown & pedometer
module time_move_cnt(
	clk_in,
	rst,
	seg_data7,
	seg_data6,
	seg_led,
	cat,
	win_judge,
	flag,
	pause,
	key_s
);
input clk_in,rst;
input win_judge;
input pause;
input key_s;
input wire [3:0] seg_data7,seg_data6;//Stepper Data Transfer Direction Controls the Number of Steps Input
output  wire [7:0] seg_led;//Digital Tube Display Signal
output  wire [7:0] cat;
output reg flag;//Judge when countdowns to 0,success 0,fail 1

wire clk_1;
reg [3:0] seg_data1,seg_data0;//Countdown data
reg st_judge;
wire key_pulse_s;
//reg [1:0] three;
//reg jump;
defparam divd_1.divdN=50000000,divd_1.divdWIDTH=28;//Redefining Divider Module Parameters
divd_frequency divd_1(.rst(rst),.clk_in(clk_in),.clk_out(clk_1));//Frequency divider 1 Hz

/*initial
begin
	three<=2'd0;
end

always@(posedge clk_1 or posedge rst)
begin
case(win_judge)
	1'b1:
		begin
			if(rst)
			begin
				three<=2'd0;
				jump=1'b0;
			end
			else if(three!=2'd3)
				three<=three+1'b1;
			else
				begin
					three<=three;
					jump<=1'b1;
				end
		end
	endcase
end
*/
debounce us (.clk(clk_in),.rst(rst),.key(key_s),.key_pulse(key_pulse_s));
initial
begin
	st_judge<=1'b0;
	//jump=1'b0;
end

always@(posedge clk_in)
//begin
	//case(jump)
	//1'b0:
		begin
			if(key_pulse_s)
			st_judge<=1'b1;
		end
	/*1'b1:
		begin
			st_judge<=1'b0;
		end
		endcase
end*/


always@(posedge clk_1 or posedge rst)//Timing counter
begin
	if(rst)//Reset
	begin 
		seg_data1 <= 4'd3;
		seg_data0 <= 4'd0;
		flag <= 1'b0;
	end
	else if(!win_judge)
		case(st_judge)
		1'b0:
			begin
				seg_data1 <= seg_data1;
				seg_data0 <= seg_data0;
			end
		1'b1:
		begin
		case(pause)//Whether the game is paused
		1'b0://Working
			begin
				if(flag)
					begin seg_data1 <= 4'd0;seg_data0 <= 4'd0;end
				else
				begin
					if(seg_data1 == 4'd0 && seg_data0 == 4'd0)
						flag <= 1'b1;
					else
					begin
						if(seg_data0 == 4'd0)//countdown to 0
						begin
							seg_data1 <= seg_data1 - 1'b1;
							seg_data0 <= 4'd9;
						end
						else
							seg_data0 <= seg_data0 - 1'b1;
					end
				end	
			end
		1'b1://Pausing
			begin
				seg_data1 <= seg_data1;
				seg_data0 <= seg_data0;
			end
		endcase
		end
	endcase
end

segment s1
(
	.clk_in(clk_in),
	.rst(rst),
	.seg_data7(seg_data7),
	.seg_data6(seg_data6),
	.seg_data1(seg_data1),
	.seg_data0(seg_data0),
	.seg_led(seg_led),
	.cat(cat),
	.pause(pause),
	.key_s(key_s)
);

endmodule

//Seven-segment digital tube display
module segment
(
	clk_in,
	rst,
	seg_data7,//nput data, 76 pedometer data, 10 countdown data
	seg_data6,
	seg_data1,
	seg_data0,
	seg_led,
	cat,
	pause,
	key_s
);
input pause;
input key_s;
input clk_in,rst;
input wire [3:0] seg_data7,seg_data6,seg_data1,seg_data0;
output reg [7:0] cat;//Scan the low level
output reg [7:0] seg_led;//Control High Level,pabcdefg
reg st_judge;
wire key_pulse_s;

reg [7:0] seg [9:0];// Define a display of 10 registers SEG with 10 to 9 digits, 8 bits per register

debounce us (.clk(clk_in),.rst(rst),.key(key_s),.key_pulse(key_pulse_s));
initial
begin
	st_judge=1'b0;
end

always@(posedge clk_in)
begin
	if(key_pulse_s)
		st_judge=1'b1;
end

initial
begin
	seg[0] = 8'b01111110;
	seg[1] = 8'b00110000;
	seg[2] = 8'b01101101;
	seg[3] = 8'b01111001;
	seg[4] = 8'b00110011;
	seg[5] = 8'b01011011;
	seg[6] = 8'b01011111;
	seg[7] = 8'b01110000;
	seg[8] = 8'b01111111;
	seg[9] = 8'b01111011;
end

reg [1:0]cnt;
wire clk_1k;

defparam divd_1k.divdN=50000,divd_1k.divdWIDTH=16;//Redefining Divider Module Parameters(Important things always repeats three times)
divd_frequency divd_1k(.rst(rst),.clk_in(clk_in),.clk_out(clk_1k));//Frequency divider 1 kHz(Human nature is a repeater,you know)

always@(posedge clk_1k or posedge rst)
begin
	if(rst)
		cnt <= 2'b0;
	else
		cnt <= cnt + 1'b1;
end

reg[7:0] segtemp[3:0];//Store tempetary datas
always@(posedge clk_in)
begin
	case(st_judge)
		1'b0:
		begin
			segtemp[3]<=segtemp[3];
			segtemp[2]<=segtemp[2];
			segtemp[1]<=segtemp[1];
			segtemp[0]<=segtemp[0];
		end
		1'b1:
		begin
			case(pause)
				1'b0://Working
				begin
					segtemp[3]<=seg[seg_data7];
					segtemp[2]<=seg[seg_data6];
					segtemp[1]<=seg[seg_data1];
					segtemp[0]<=seg[seg_data0];
				end
				1'b1://Storing
				begin
					segtemp[3]<=segtemp[3];
					segtemp[2]<=segtemp[2];
					segtemp[1]<=segtemp[1];
					segtemp[0]<=segtemp[0];
				end
			endcase
		end
	endcase
end


always@(cnt)
	begin
		case(st_judge)
			1'b0:
			begin
				case(cnt)
					2'b00:begin cat=8'b01111111;seg_led = segtemp[3];end
					2'b01:begin cat=8'b10111111;seg_led = segtemp[2];end
					2'b10:begin cat=8'b11111101;seg_led = segtemp[1];end
					2'b11:begin cat=8'b11111110;seg_led = segtemp[0];end
					endcase
				end
			1'b1:
			begin
			case(pause)
			1'b0://working
			begin
				case(cnt)
					2'b00:begin cat=8'b01111111;
						case(rst)
							1'b1:seg_led = 8'b0;
							1'b0:seg_led = seg[seg_data7];
						endcase
						end
					2'b01:begin cat=8'b10111111;seg_led = seg[seg_data6];end
					2'b10:begin cat=8'b11111101;seg_led = seg[seg_data1];end
					2'b11:begin cat=8'b11111110;seg_led = seg[seg_data0];end
					default:;
				endcase
				end
			1'b1://Assign datas
			begin
				case(cnt)
					2'b00:begin cat=8'b01111111;seg_led = segtemp[3];end
					2'b01:begin cat=8'b10111111;seg_led = segtemp[2];end
					2'b10:begin cat=8'b11111101;seg_led = segtemp[1];end
					2'b11:begin cat=8'b11111110;seg_led = segtemp[0];end
					endcase
				end
		endcase
	end
	endcase
	end
endmodule

//Direction control module
module direction_judge
(
	clk_in,
	rst,
	flag,//Fail 1,game over
	win_judge,
	key,
	pos,
	movecnt,//Connect with seg_data in step counter
	map,
	pause,
	beep
);
input clk_in,rst;
input flag,win_judge;
input [3:0] key;//LRUD
input map;
input pause;
input beep;
output reg [63:0] pos;
output reg [7:0]movecnt;//[7:4][3:0]

wire [3:0] key_pulse;//Jitter thread
reg [3:0] gold,goru;//Shift
reg judge;//Bump 1,otherwise 0
reg [63:0] brick,temp;
reg [3:0]high;
reg [3:0]med;
reg [3:0]low;

/*
initial
begin
	{high,med,low}<=12'b0;
end
*/

always@(posedge clk_in)//Defining Obstacles by maps
begin
	case(map)
	1'b0:
		begin
			brick[63:56]=8'b01111111;
			brick[55:48]=8'b01000001;
			brick[47:40]=8'b01011101;
			brick[39:32]=8'b01010101;
			brick[31:24]=8'b01010101;
			brick[23:16]=8'b11000101;
			brick[15: 8]=8'b00011100;
			brick[ 7: 0]=8'b11111111;
		end
	1'b1:	
		begin
			brick[63:56]=8'b00011010;
			brick[55:48]=8'b01111010;
			brick[47:40]=8'b01000011;
			brick[39:32]=8'b11011100;
			brick[31:24]=8'b10010101;
			brick[23:16]=8'b10110101;
			brick[15: 8]=8'b10000001;
			brick[ 7: 0]=8'b11111111;
		end
	endcase
end

always@(posedge clk_in or posedge rst)
begin
	if(rst)
		//case(pause)
		//1'b0:
			case(map)
			1'b0:
				begin
					movecnt <= 8'h00;
					pos[63:56] <= 8'b00000000;
					pos[55:48] <= 8'b00000000;
					pos[47:40] <= 8'b00000000;
					pos[39:32] <= 8'b00000000;
					pos[31:24] <= 8'b00000000;
					pos[23:16] <= 8'b00000000;
					pos[15: 8] <= 8'b00000001;
					pos[ 7: 0] <= 8'b00000000;
				end
			1'b1:
				begin
					movecnt <= 8'h00;
					pos[63:56] <= 8'b00000000;
					pos[55:48] <= 8'b00000000;
					pos[47:40] <= 8'b00000000;
					pos[39:32] <= 8'b00000001;
					pos[31:24] <= 8'b00000000;
					pos[23:16] <= 8'b00000000;
					pos[15: 8] <= 8'b00000000;
					pos[ 7: 0] <= 8'b00000000;
				end
			endcase
		//endcase
	else if(!flag && !win_judge &&!pause)
		if(key_pulse)
			begin
				case(key_pulse)
					4'b1000: begin gold = 4'd1; goru = 4'd0;end 
					4'b0100: begin goru = 4'd1; gold = 4'd0;end
					4'b0010: begin goru = 4'd8; gold = 4'd0;end
					4'b0001: begin gold = 4'd8; goru = 4'd0;end
					default:;
				endcase
			
				temp = pos >> gold << goru;//Judgment of Hitting Obstacles
				if(temp & brick)
					judge = 1;
				else 
					judge = 0;
				
				if(!judge)//Shift
				begin
					pos = pos >> gold << goru;
					//Pedometer
					if(movecnt[3:0] == 4'd9)//Unit's digit counts to 9
					begin
						movecnt[3:0] = 4'd0;
						if(movecnt[7:4] == 9)//Ten's digit counts to 9
							movecnt[7:4] = 4'd0;
						else
							movecnt[7:4] = movecnt[7:4] + 1'b1;
					end
					else
						movecnt[3:0] = movecnt[3:0] + 1'b1;
				end				
			end
end
/*
always@(posedge judge)
begin
	{high,med,low}<=12'b011100000000;
end
*/
debounce ul (.clk(clk_in),.rst(rst),.key(key[3]),.key_pulse(key_pulse[3]));//Debouncing keys
debounce ur (.clk(clk_in),.rst(rst),.key(key[2]),.key_pulse(key_pulse[2]));
debounce uu (.clk(clk_in),.rst(rst),.key(key[1]),.key_pulse(key_pulse[1]));
debounce ud (.clk(clk_in),.rst(rst),.key(key[0]),.key_pulse(key_pulse[0]));

endmodule

//Frequency divider module
module divd_frequency(rst,clk_in,clk_out);

input rst,clk_in;
output reg clk_out;
parameter divdWIDTH=1;
parameter divdN=1;//Frequency division multiplier
reg [divdWIDTH:0] cnt;

always@(posedge clk_in or posedge rst)
	if(rst)
		begin
		cnt<=0;
		clk_out<=0;
		end
	else
		begin
		cnt<=cnt+1'b1;
		if(cnt==(divdN/2-1))
			begin
			cnt<=0;
			clk_out<=~clk_out;
			end
		end
endmodule

//Keys debouncing module 
module debounce (clk,rst,key,key_pulse);
 
parameter       N  =  1;                      //Number of keys for debouncing
 
input clk;
input rst;
input [N-1:0] key;                        //Input buttons					
output [N-1:0] key_pulse;                  //Pulse generated by keystroke action
 
reg [N-1:0] key_rst_pre;                //Define a register variable to store the key value of a trigger
reg [N-1:0] key_rst;                    //Define a register variable to store key values triggered at the current time
 
wire [N-1:0] key_edge;                   //Detection of key changes from high to low produces a high pulse
 
//Using the non-blocking assignment feature, the key states of two clock triggers are stored in two register variables.
always @(posedge clk  or  posedge rst)
begin
	if (rst) begin
		key_rst <= {N{1'b0}};                //Initialization assigns key_rst to all 1£¬{}means there are N pieces of 1
		key_rst_pre <= {N{1'b0}};
		end
	else begin
		key_rst <= key;                     //The value of key is assigned to key_rst after the first clock rising edge triggers, and the value of key_rst is assigned to key_rst_pre.
		key_rst_pre <= key_rst;             //Non-blocking assignment. Equivalent to two clock triggers, key_rst stores the value of the key at the current time, and key_rst_prestores the value of the key at the previous clock.
		end    
end
 
assign  key_edge = (~key_rst_pre) & key_rst;//Pulse edge detection. When key detects the rising edge, key_edge produces a high level of a clock cycle
 
reg	[19:0]	  cnt;                       //The system clock is 50MHz. To delay about 20ms, at least 20-bit counter is needed.
 
//Generating 20ms delay, when key_edge validity is detected, counter start counting
always @(posedge clk or posedge rst)
begin
	if(rst)
		cnt <= 20'h0;
	else if(key_edge)
		cnt <= 20'h0;
	else
		cnt <= cnt + 1'h1;
end  
 
reg [N-1:0] key_sec_pre;                //Delay Detection Level Register Variables
reg [N-1:0] key_sec;                    
 
 
//Delay detection key produces a high pulse of the clock if the key state decreases. If the key state is high, the key is invalid.
always @(posedge clk  or  posedge rst)
begin
	if (rst) 
		key_sec <= {N{1'b0}};                
	else if (cnt==20'hfffff)
		key_sec <= key;  
end

always @(posedge clk  or  posedge rst)
begin
	if (rst)
		key_sec_pre <= {N{1'b0}};
	else                   
		key_sec_pre <= key_sec;             
end      
assign  key_pulse = (~key_sec_pre) & key_sec;     
 
endmodule
