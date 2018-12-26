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
	beep
);
input clk_in;//时钟输入
input rst;//复位信号
input [3:0] key;//方向输入
input map;//up1,down0,default0
input pause;
input beep;
output [7:0]row,col_red,col_green;//点阵输出
output [7:0]cat,seg_led;//数码管显示

wire time_judge;//判断时间的 =1超时 =0未超时
reg win_judge;// =1胜利 =0未胜利
wire [7:0] movecnt;//计步计数器器
wire [63:0] pos;//位置信号
//wire map_judge;

always@(posedge clk_in or posedge rst)
begin
	if(rst)
		win_judge = 1'b0;
	else if(!map)
		win_judge = pos[15];
	else
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
	.pause(pause)
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
led88 u3
(
	.clk_in(clk_in),
	.rst(rst),.pos(pos),
	.row(row),
	.col_red(col_red),
	.col_green(col_green),
	.time_judge(time_judge),
	.win_judge(win_judge),
	.map(map)
);
	
endmodule


//迷宫点阵显示模块
module led88
(
	clk_in,
	rst,
	pos,
	row,
	col_red,
	col_green,
	time_judge,
	win_judge,
	map
);
input clk_in,rst;
input time_judge,win_judge;
input  [63:0] pos;
input map;
output reg [7:0] row,col_red,col_green;
reg [2:0] cnt;
wire clk_1k;

defparam divd_1k.divdN=50000,divd_1k.divdWIDTH=16;//重定义分频器模块参数
divd_frequency divd_1k(.rst(rst),.clk_in(clk_in),.clk_out(clk_1k));//分频器 分频1kHz

always@(posedge clk_1k or posedge rst)
begin
	if(rst)//置位判断
		cnt<=3'b000;
	else
		cnt<=cnt+1'b1;
end


always@(cnt or rst)
begin
	if(rst)
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
	else if( (!time_judge) & (!win_judge) & (!map))//map0进行中
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
	else if( (!time_judge) & (!win_judge) & (map))
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
	else if( (~time_judge) & win_judge )//胜利
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
	else if( time_judge & (~win_judge) )//失败
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

endmodule

//倒计时 计步器
module time_move_cnt(
	clk_in,
	rst,
	seg_data7,
	seg_data6,
	seg_led,
	cat,
	win_judge,
	flag,
	pause
);
input clk_in,rst;
input win_judge;
input pause;
input wire [3:0] seg_data7,seg_data6;//计步器数据传递方向控制传入的步数
output  wire [7:0] seg_led;//数码管显示电路信号
output  wire [7:0] cat;
output reg flag;//倒计时到0时产生flag停止计数 并且作为判断游戏失败的信号 1为失败

wire clk_1;
reg [3:0] seg_data1,seg_data0;//倒计时数据

defparam divd_1.divdN=50000000,divd_1.divdWIDTH=28;//重定义分频器模块参数
divd_frequency divd_1(.rst(rst),.clk_in(clk_in),.clk_out(clk_1));//分频器 分频1Hz

always@(posedge clk_1 or posedge rst)//计时计数器
begin
	if(rst)
	begin 
		seg_data1 <= 4'd3;
		seg_data0 <= 4'd0;
		flag <= 1'b0;
	end
	else if(!win_judge)
		case(pause)
		1'b0:
			begin
				if(flag)
					begin seg_data1 <= 4'd0;seg_data0 <= 4'd0;end
				else
				begin
					if(seg_data1 == 4'd0 && seg_data0 == 4'd0)
						flag <= 1'b1;
					else
					begin
						if(seg_data0 == 4'd0)//个位为0？
						begin
							seg_data1 <= seg_data1 - 1'b1;
							seg_data0 <= 4'd9;
						end
						else
							seg_data0 <= seg_data0 - 1'b1;
					end
				end	
			end
		1'b1:
			begin
				seg_data1 <= seg_data1;
				seg_data0 <= seg_data0;
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
	.pause(pause)
);

endmodule

//七段数码管显示
module segment
(
	clk_in,
	rst,
	seg_data7,//输入数据，76为计步器数据 10为倒计时数据
	seg_data6,
	seg_data1,
	seg_data0,
	seg_led,
	cat,
	pause
);
input pause;
input clk_in,rst;
input wire [3:0] seg_data7,seg_data6,seg_data1,seg_data0;
output reg [7:0] cat;//扫描低电平
output reg [7:0] seg_led;//控制高电平  高位到低位（左到右）分别为pabcdefg

reg [7:0] seg [9:0];// 定义0~9十个数字的显示 10个寄存器seg 每个寄存器8位

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

defparam divd_1k.divdN=50000,divd_1k.divdWIDTH=16;//重定义分频器模块参数
divd_frequency divd_1k(.rst(rst),.clk_in(clk_in),.clk_out(clk_1k));//分频器 分频1kHz

always@(posedge clk_1k or posedge rst)
begin
	if(rst)
		cnt <= 2'b0;
//	else if(pause)
//		cnt <= cnt;
	else
		cnt <= cnt + 1'b1;
end

reg[7:0] segtemp[3:0];
always@(posedge clk_in)
begin
	case(pause)
		1'b0:begin
		segtemp[3]<=seg[seg_data7];
		segtemp[2]<=seg[seg_data6];
		segtemp[1]<=seg[seg_data1];
		segtemp[0]<=seg[seg_data0];
		end
		1'b1:begin
		segtemp[3]<=segtemp[3];
		segtemp[2]<=segtemp[2];
		segtemp[1]<=segtemp[1];
		segtemp[0]<=segtemp[0];
		end
	endcase
end


always@(cnt)
	begin
		case(pause)
			1'b0:begin
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
			1'b1:
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
endmodule

//方向控制模块
module direction_judge
(
	clk_in,
	rst,
	flag,//为1时 游戏失败 停止计步和移动
	win_judge,
	key,
	pos,
	movecnt,//计步计数器与计步器中seg_data连接
	map,
	pause,
	beep
);
input clk_in,rst;
input flag,win_judge;
input [3:0] key;//高位到低位分别表示左右上下
input map;
input pause;
input beep;
output reg [63:0] pos;
output reg [7:0]movecnt;//[7:4][3:0]个位

wire [3:0] key_pulse;//消抖线程
reg [3:0] gold,goru;//移位
reg judge;//judge=1撞墙 =0不撞墙
reg [63:0] brick,temp;

always@(posedge clk_in)//定义墙
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
	else if(!flag && !win_judge &&!pause)//判断游戏是否失败
		if(key_pulse)
		begin
			case(key_pulse)
			4'b1000: begin gold = 4'd1; goru = 4'd0;end 
			4'b0100: begin goru = 4'd1; gold = 4'd0;end
			4'b0010: begin goru = 4'd8; gold = 4'd0;end
			4'b0001: begin gold = 4'd8; goru = 4'd0;end
			default:;
			endcase
			
			temp = pos >> gold << goru;//撞墙判断
			if(temp & brick)
				judge = 1;
			else 
				judge = 0;
			
			if(!judge)//位移
			begin
				pos = pos >> gold << goru;
				//计步器
				if(movecnt[3:0] == 4'd9)//个位满9?
				begin
					movecnt[3:0] = 4'd0;
					if(movecnt[7:4] == 9)//十位满9？
						movecnt[7:4] = 4'd0;
					else
						movecnt[7:4] = movecnt[7:4] + 1'b1;
				end
				else
					movecnt[3:0] = movecnt[3:0] + 1'b1;
			end
			//else beep;
		end
end

debounce ul (.clk(clk_in),.rst(rst),.key(key[3]),.key_pulse(key_pulse[3]));//按键消抖
debounce ur (.clk(clk_in),.rst(rst),.key(key[2]),.key_pulse(key_pulse[2]));
debounce uu (.clk(clk_in),.rst(rst),.key(key[1]),.key_pulse(key_pulse[1]));
debounce ud (.clk(clk_in),.rst(rst),.key(key[0]),.key_pulse(key_pulse[0]));

endmodule

//分频器模块
module divd_frequency(rst,clk_in,clk_out);

input rst,clk_in;
output reg clk_out;
parameter divdWIDTH=1;
parameter divdN=1;//分频倍数
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

//按键消抖模块
module debounce (clk,rst,key,key_pulse);
 
parameter       N  =  1;                      //要消除的按键的数量
 
input clk;
input rst;
input [N-1:0] key;                        //输入的按键					
output [N-1:0] key_pulse;                  //按键动作产生的脉冲	
 
reg [N-1:0] key_rst_pre;                //定义一个寄存器型变量存储上一个触发时的按键值
reg [N-1:0] key_rst;                    //定义一个寄存器变量储存储当前时刻触发的按键值
 
wire [N-1:0] key_edge;                   //检测到按键由高到低变化是产生一个高脉冲
 
//利用非阻塞赋值特点，将两个时钟触发时按键状态存储在两个寄存器变量中
always @(posedge clk  or  posedge rst)
begin
	if (rst) begin
		key_rst <= {N{1'b0}};                //初始化时给key_rst赋值全为1，{}中表示N个1
		key_rst_pre <= {N{1'b0}};
		end
	else begin
		key_rst <= key;                     //第一个时钟上升沿触发之后key的值赋给key_rst,同时key_rst的值赋给key_rst_pre
		key_rst_pre <= key_rst;             //非阻塞赋值。相当于经过两个时钟触发，key_rst存储的是当前时刻key的值，key_rst_pre存储的是前一个时钟的key的值
		end    
end
 
assign  key_edge = (~key_rst_pre) & key_rst;//脉冲边沿检测。当key检测到上升沿时，key_edge产生一个时钟周期的高电平
 
reg	[19:0]	  cnt;                       //产生延时所用的计数器，系统时钟50MHz，要延时20ms左右时间，至少需要20位计数器     
 
//产生20ms延时，当检测到key_edge有效是计数器清零开始计数
always @(posedge clk or posedge rst)
begin
	if(rst)
		cnt <= 20'h0;
	else if(key_edge)
		cnt <= 20'h0;
	else
		cnt <= cnt + 1'h1;
end  
 
reg [N-1:0] key_sec_pre;                //延时后检测电平寄存器变量
reg [N-1:0] key_sec;                    
 
 
//延时后检测key，如果按键状态变低产生一个时钟的高脉冲。如果按键状态是高的话说明按键无效
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
